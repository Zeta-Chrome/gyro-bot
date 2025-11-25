#include "gyro_context.hpp"
#include "utils.hpp"
#include <wifi_station.hpp>
#include <imu.hpp>
#include <ultrasound_sensor.hpp>
#include <motor_driver.hpp>
#include <servo.hpp>
#include <tcp_client.hpp>
#include <udp_client.hpp>
#include <cstring>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

constexpr gpio_num_t BLUE_LED_GPIO = GPIO_NUM_2;
constexpr gpio_num_t CAM_TRIG_GPIO = GPIO_NUM_33;
constexpr uint16_t UDP_RX_PORT = 9000;
constexpr uint16_t UDP_TX_PORT = 9001;
constexpr uint16_t TCP_RX_PORT = 9002;
constexpr float MAX_TILT_ANGLE = 5.0f;
constexpr float MAX_TURN = 0.5f;

void led_wifi_notifier_task(void* args)
{
    GyroContext* ctx = static_cast<GyroContext*>(args);
    if (ctx == NULL)
    {
        ESP_LOGE("LED_TASK", "NULL context!");
        vTaskDelete(NULL);
        return;
    }

    gpio_reset_pin(BLUE_LED_GPIO);
    gpio_set_direction(BLUE_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLUE_LED_GPIO, 0);

    EventBits_t bits;
    while (true)
    {
        bits = xEventGroupWaitBits(ctx->station.get_wifi_event(),
                                   WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_RETRY_BIT, pdTRUE,
                                   pdFALSE, portMAX_DELAY);

        if (bits & WIFI_CONNECTED_BIT)
        {
            gpio_set_level(BLUE_LED_GPIO, 1);
        }
        else if (bits & WIFI_FAIL_BIT)
        {
            gpio_set_level(BLUE_LED_GPIO, 0);
        }
        else
        {
            while (true)
            {
                gpio_set_level(BLUE_LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(250));
                gpio_set_level(BLUE_LED_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(250));

                EventBits_t now = xEventGroupGetBits(ctx->station.get_wifi_event());
                if (now & (WIFI_CONNECTED_BIT | WIFI_FAIL_BIT))
                    break;
            }
        }
    }
}

void imu_read_task(void* args)
{
    GyroContext* ctx = static_cast<GyroContext*>(args);

    if (ctx == NULL)
    {
        ESP_LOGE("IMU_TASK", "FATAL: NULL context!");
        vTaskDelete(NULL);
        return;
    }

    while (!ctx->queues_initialized)
        vTaskDelay(pdMS_TO_TICKS(10));

    if (!ctx->is_queue_valid(ctx->imu_queue) || !ctx->is_queue_valid(ctx->imu_motor_queue))
    {
        ESP_LOGE("IMU_TASK", "FATAL: Invalid queues!");
        ESP_LOGE("IMU_TASK", "  imu_queue: %p", ctx->imu_queue);
        ESP_LOGE("IMU_TASK", "  imu_motor_queue: %p", ctx->imu_motor_queue);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("IMU_TASK", "Started successfully");

    const TickType_t sample_period = pdMS_TO_TICKS(30);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        const IMUData& data = ctx->imu.read_imu_data();
        TimestampedIMUData ts_data;
        ts_data.timestamp_us = esp_timer_get_time();
        ts_data.ax = data.ax;
        ts_data.ay = data.ay;
        ts_data.az = data.az;
        ts_data.gx = data.gx;
        ts_data.gy = data.gy;
        ts_data.gz = data.gz;

        if (ctx->is_queue_valid(ctx->imu_queue))
        {
            if (xQueueSend(ctx->imu_queue, &ts_data, 0) != pdTRUE)
            {
                // Queue full - pop oldest and retry
                TimestampedIMUData dummy;
                if (xQueueReceive(ctx->imu_queue, &dummy, 0) == pdTRUE)
                {
                    xQueueSend(ctx->imu_queue, &ts_data, 0);
                }
            }
        }

        if (ctx->is_queue_valid(ctx->imu_motor_queue))
        {
            xQueueOverwrite(ctx->imu_motor_queue, &ts_data);
        }
        else
        {
            ESP_LOGE("IMU_TASK", "Motor queue became invalid!");
        }

        vTaskDelayUntil(&last_wake_time, sample_period);
    }
}

void motor_control_task(void* args)
{
    GyroContext* ctx = static_cast<GyroContext*>(args);
    MotorDriver motor_driver(GPIO_NUM_4, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_13);
    motor_driver.set_motor_speeds(0.0f, 0.0f);

    if (ctx == NULL)
    {
        ESP_LOGE("MOTOR_TASK", "FATAL: NULL context!");
        vTaskDelete(NULL);
        return;
    }

    // Wait for queues
    while (!ctx->queues_initialized)
        vTaskDelay(pdMS_TO_TICKS(10));

    if (!ctx->is_queue_valid(ctx->imu_motor_queue) || !ctx->is_queue_valid(ctx->motor_queue))
    {
        ESP_LOGE("MOTOR_TASK", "FATAL: Invalid queues!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("MOTOR_TASK", "Started successfully");

    float prev_error = 0.0f;
    float integral = 0.0f;

    ControlData control = {0.0f, 0.0f, 0};
    TimestampedIMUData imu_data;
    int64_t prev_timestamp = 0;

    while (true)
    {
        // FIXED: Read PID values every iteration (they can be updated via TCP)
        float Kp = ctx->pid_params.kp;
        float Ki = ctx->pid_params.ki;
        float Kd = ctx->pid_params.kd;

        // Safe queue receive with validation
        if (!ctx->is_queue_valid(ctx->imu_motor_queue))
        {
            ESP_LOGE("MOTOR_TASK", "Queue corrupted!");
            motor_driver.set_motor_speeds(0.0f, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (xQueueReceive(ctx->imu_motor_queue, &imu_data, portMAX_DELAY) != pdTRUE)
            continue;

        if (imu_data.timestamp_us == prev_timestamp)
            continue;

        float dt = 0.01f;
        if (prev_timestamp != 0)
            dt = (imu_data.timestamp_us - prev_timestamp) / 1e6f;
        dt = fmaxf(0.001f, fminf(dt, 0.05f));
        prev_timestamp = imu_data.timestamp_us;

        const Pitch& pitch = ctx->imu.get_pitch(dt, FilterType::COMPLEMENTARY);

        // Safe peek at control queue
        if (ctx->is_queue_valid(ctx->motor_queue))
            xQueuePeek(ctx->motor_queue, &control, 0);

        float forward = control.magnitude * cos(control.angle * M_PI / 180.0f);
        float turn = control.magnitude * sin(control.angle * M_PI / 180.0f);

        float target_pitch = forward * MAX_TILT_ANGLE;

        // Error is positive when robot tips forward (needs backward correction)
        float error = target_pitch - pitch.angle;

        integral += error * dt;
        integral = fmaxf(-0.5f, fminf(0.5f, integral));  // Anti-windup

        float derivative = (error - prev_error) / dt;
        prev_error = error;

        float balance_output = Kp * error + Ki * integral + Kd * derivative;

        // Apply turning
        float left_speed = balance_output - (turn * MAX_TURN);
        float right_speed = balance_output + (turn * MAX_TURN);

        // Clamp to valid range
        left_speed = fmaxf(-1.0f, fminf(1.0f, left_speed));
        right_speed = fmaxf(-1.0f, fminf(1.0f, right_speed));

        // motor_driver.set_motor_speeds(left_speed, right_speed);

        // ✓ ADDED: Debug logging (optional, remove if too verbose)
        ESP_LOGI("MOTOR", "pitch=%.2f, target=%.2f, err=%.2f, out=%.2f, L=%.2f, R=%.2f",
                 pitch.angle, target_pitch, error, balance_output, left_speed, right_speed);
    }
}

void servo_control_task(void* args)
{
    GyroContext* ctx = static_cast<GyroContext*>(args);

    if (ctx == NULL)
    {
        ESP_LOGE("SERVO_TASK", "FATAL: NULL context!");
        vTaskDelete(NULL);
        return;
    }

    while (!ctx->queues_initialized)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (!ctx->is_queue_valid(ctx->servo_queue))
    {
        ESP_LOGE("SERVO_TASK", "FATAL: Invalid queue!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("SERVO_TASK", "Started successfully");

    Servo servo(GPIO_NUM_18);
    float current_angle = 0.0f;
    float target_angle = 0.0f;
    int32_t new_target;

    servo.set_angle(current_angle);

    const float alpha = 0.15f;  // Increased from 0.1 for slightly faster response
    const TickType_t step_delay = pdMS_TO_TICKS(20);  // 50Hz update rate

    while (true)
    {
        if (xQueueReceive(ctx->servo_queue, &new_target, 0) == pdTRUE)
        {
            target_angle = (float)new_target;
            ESP_LOGD("SERVO_TASK", "New target: %.1f", target_angle);
        }

        if (fabs(current_angle - target_angle) > 0.5f)
        {
            current_angle = current_angle * (1.0f - alpha) + target_angle * alpha;
            servo.set_angle(current_angle);
        }

        vTaskDelay(step_delay);
    }
}

void ultrasound_read_task(void* args)
{
    GyroContext* ctx = static_cast<GyroContext*>(args);

    if (ctx == NULL)
    {
        ESP_LOGE("US_TASK", "FATAL: NULL context!");
        vTaskDelete(NULL);
        return;
    }

    // camera trigger gpio setup
    gpio_reset_pin(CAM_TRIG_GPIO);
    gpio_set_direction(CAM_TRIG_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CAM_TRIG_GPIO, 0);

    // Wait for queues
    while (!ctx->queues_initialized)
        vTaskDelay(pdMS_TO_TICKS(10));

    if (!ctx->is_queue_valid(ctx->ultrasound_queue))
    {
        ESP_LOGE("US_TASK", "FATAL: Invalid queue!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("US_TASK", "Started successfully");

    ctx->ultrasound.init();
    TimestampedUltrasound ts_data;

    while (true)
    {
        ts_data.distance_cm = ctx->ultrasound.measure_cm();

        if (ts_data.distance_cm < 0)
        {
            ESP_LOGW("US_TASK", "Measurement error");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        ts_data.timestamp_us = esp_timer_get_time();

        // Safe send with validation
        if (ctx->is_queue_valid(ctx->ultrasound_queue))
        {
            if (xQueueSend(ctx->ultrasound_queue, &ts_data, 0) != pdTRUE)
            {
                TimestampedUltrasound dummy;
                if (xQueueReceive(ctx->ultrasound_queue, &dummy, 0) == pdTRUE)
                {
                    xQueueSend(ctx->ultrasound_queue, &ts_data, 0);
                }
            }
        }

        gpio_set_level(CAM_TRIG_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(CAM_TRIG_GPIO, 0);

        vTaskDelay(pdMS_TO_TICKS(90));
    }
}

void udp_tx_task(void* args)
{
    GyroContext* ctx = static_cast<GyroContext*>(args);

    if (ctx == NULL)
    {
        ESP_LOGE("UDP_TX_TASK", "FATAL: NULL context!");
        vTaskDelete(NULL);
        return;
    }

    // Wait for queues AND WiFi
    while (!ctx->queues_initialized)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ctx->station.wait_for_wifi();

    ESP_LOGI("UDP_TX_TASK", "Starting...");

    const char* bcast_ip = "255.255.255.255";
    UDPClient udp_tx(UDPMode::TRANSMITTER, bcast_ip, UDP_TX_PORT);

    int err = udp_tx.start();
    if (err != 0)
    {
        ESP_LOGE("UDP_TX_TASK", "Failed to start: %d", err);
        vTaskDelete(NULL);
        return;
    }

    TimestampedIMUData imu_batch[15];
    TimestampedUltrasound us_batch[5];

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t send_period = pdMS_TO_TICKS(100);

    while (true)
    {
        int imu_count = 0;

        // Safe batch receive
        if (ctx->is_queue_valid(ctx->imu_queue))
        {
            while (xQueueReceive(ctx->imu_queue, &imu_batch[imu_count], 0) == pdTRUE
                   && imu_count < 15)
            {
                imu_count++;
            }
        }

        int us_count = 0;

        if (ctx->is_queue_valid(ctx->ultrasound_queue))
        {
            while (xQueueReceive(ctx->ultrasound_queue, &us_batch[us_count], 0) == pdTRUE
                   && us_count < 5)
            {
                us_count++;
            }
        }

        if (imu_count > 0 || us_count > 0)
        {
            uint8_t buffer[sizeof(uint8_t) + sizeof(imu_batch) + sizeof(uint8_t)
                           + sizeof(us_batch)];
            size_t offset = 0;

            buffer[offset++] = (uint8_t)imu_count;
            memcpy(buffer + offset, imu_batch, imu_count * sizeof(TimestampedIMUData));
            offset += imu_count * sizeof(TimestampedIMUData);

            buffer[offset++] = (uint8_t)us_count;
            memcpy(buffer + offset, us_batch, us_count * sizeof(TimestampedUltrasound));
            offset += us_count * sizeof(TimestampedUltrasound);

            int sent = udp_tx.send_data(buffer, offset);
            if (sent <= 0)
            {
                ESP_LOGW("UDP_TX_TASK", "Send failed: %d", sent);
            }
        }

        vTaskDelayUntil(&last_wake_time, send_period);
    }
}

void udp_rx_task(void* args)
{
    GyroContext* ctx = static_cast<GyroContext*>(args);

    if (ctx == NULL)
    {
        ESP_LOGE("UDP_RX_TASK", "FATAL: NULL context!");
        vTaskDelete(NULL);
        return;
    }

    while (!ctx->queues_initialized)
        vTaskDelay(pdMS_TO_TICKS(10));

    ctx->station.wait_for_wifi();

    ESP_LOGI("UDP_RX_TASK", "Starting...");

    UDPClient udp_receiver(UDPMode::RECEIVER, UDP_RX_PORT, sizeof(ControlData));

    int err = udp_receiver.start();
    if (err != 0)
    {
        ESP_LOGE("UDP_RX_TASK", "Failed to start: %d", err);
        vTaskDelete(NULL);
        return;
    }

    uint8_t buffer[sizeof(ControlData)];
    ControlData control;

    while (true)
    {
        int ret = udp_receiver.receive_data(buffer, sizeof(buffer));

        if (ret == sizeof(ControlData))
        {
            memcpy(&control, buffer, sizeof(ControlData));
            ESP_LOGI("UDP_RX", "Control: mag=%.2f, ang=%.1f, servo=%d", control.magnitude,
                     control.angle, control.servo_angle);

            // Safe overwrite with validation
            if (ctx->is_queue_valid(ctx->motor_queue))
            {
                xQueueOverwrite(ctx->motor_queue, &control);
            }

            if (ctx->is_queue_valid(ctx->servo_queue))
            {
                xQueueOverwrite(ctx->servo_queue, &control.servo_angle);
            }
        }
        else if (ret < 0)
        {
            ESP_LOGW("UDP_RX", "Receive error: %d", ret);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else if (ret > 0)
        {
            ESP_LOGW("UDP_RX", "Unexpected size: %d", ret);
        }
    }
}

void tcp_rx_task(void* args)
{
    GyroContext* ctx = static_cast<GyroContext*>(args);

    ctx->station.wait_for_wifi();

    while (true)
    {
        ESP_LOGI("TCP_RX_TASK", "Starting TCP server on port %d...", TCP_RX_PORT);

        TCPClient tcp_receiver(TCPMode::RECEIVER, TCP_RX_PORT, 128);
        int err = tcp_receiver.start();

        if (err != 0)
        {
            ESP_LOGE("TCP_RX_TASK", "Failed to start TCP server: %d", err);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        ESP_LOGI("TCP_RX_TASK", "TCP server ready, waiting for client...");

        uint8_t buffer[128] = {0};
        int expected_size = sizeof(PIDParams) + 32 + 64;  // 108 bytes

        while (tcp_receiver.is_connected())
        {
            int ret = tcp_receiver.receive_data(buffer, expected_size);

            if (ret == expected_size)
            {
                memcpy(&ctx->pid_params, buffer, sizeof(PIDParams));

                char ssid[33] = {0};
                char passwd[65] = {0};

                memcpy(ssid, buffer + sizeof(PIDParams), 32);
                memcpy(passwd, buffer + sizeof(PIDParams) + 32, 64);

                ESP_LOGI("TCP_RX_TASK", "Received PID: kp=%.2f, ki=%.2f, kd=%.2f | Wi-Fi: %s, Passwd: %s",
                         ctx->pid_params.kp, ctx->pid_params.ki, ctx->pid_params.kd, ssid, passwd);

                ctx->save_pid_to_nvs();

                // Change WiFi in background
                ctx->station.change_wifi(ssid, passwd);

                // Close connection after successful receive
                tcp_receiver.end();
                break;
            }
            else if (ret < 0)
            {
                ESP_LOGW("TCP_RX_TASK", "Receive failed (%d), connection lost", ret);
                break;
            }
            else if (ret > 0)
            {
                ESP_LOGW("TCP_RX_TASK", "Unexpected size: %d (expected %d)", ret, expected_size);
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }

        tcp_receiver.end();
        ESP_LOGI("TCP_RX_TASK", "Connection closed, restarting server...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void)
{
    initialize_nvs();

    // Create context with all queues
    GyroContext* ctx = new GyroContext();

    // Verify queues before starting tasks
    if (!ctx->queues_initialized)
    {
        ESP_LOGE("MAIN", "FATAL: Queue initialization failed!");
        abort();
    }

    ESP_LOGI("MAIN", "Starting tasks...");

    // Start local tasks first (don't need WiFi)
    xTaskCreatePinnedToCore(imu_read_task, "IMU_READ", 3072, ctx, 6, NULL, 1);
    xTaskCreatePinnedToCore(motor_control_task, "MOTOR_CONTROL", 3072, ctx, 6, NULL, 0);
    xTaskCreatePinnedToCore(servo_control_task, "SERVO_CONTROL", 2048, ctx, 4, NULL, 1);
    xTaskCreatePinnedToCore(led_wifi_notifier_task, "LED_NOTIFIER", 1024, ctx, 1, NULL, 1);

    // Wait for WiFi before starting network tasks
    ESP_LOGI("MAIN", "Waiting for WiFi...");
    ctx->station.wait_for_wifi();
    ESP_LOGI("MAIN", "WiFi connected, starting network tasks");

    xTaskCreatePinnedToCore(ultrasound_read_task, "ULTRASOUND_READ", 2048, ctx, 3, NULL, 1);
    xTaskCreatePinnedToCore(udp_tx_task, "UDP_TX", 4096, ctx, 5, NULL, 0);
    xTaskCreatePinnedToCore(udp_rx_task, "UDP_RX", 3072, ctx, 7, NULL, 0);
    xTaskCreatePinnedToCore(tcp_rx_task, "TCP_RX", 3072, ctx, 2, NULL, 1);

    ESP_LOGI("MAIN", "All tasks started successfully");
}

