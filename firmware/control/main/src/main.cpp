#include "gyro_context.hpp"
#include "common.hpp"
#include <cstdint>
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

constexpr gpio_num_t BLUE_LED_PIN = GPIO_NUM_2;
constexpr gpio_num_t SERVO_PIN = GPIO_NUM_32;
constexpr gpio_num_t CAM_TRIG_PIN = GPIO_NUM_4;
constexpr uint16_t UDP_RX_PORT = 9000;
constexpr uint16_t UDP_TX_PORT = 9001;
constexpr uint16_t TCP_RX_PORT = 9002;
constexpr float MAX_TILT_ANGLE = 5.0f;
constexpr float MAX_TURN = 0.2f;

void status_blink(GyroContext *ctx, uint16_t high, uint16_t low, uint32_t status_bits)
{
    while (true)
    {
        gpio_set_level(BLUE_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(high));
        gpio_set_level(BLUE_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(low));

        EventBits_t now = xEventGroupGetBits(ctx->station.get_wifi_event());
        if (now & status_bits)
            return;
    }
}

void led_wifi_notifier_task(void *args)
{
    GyroContext *ctx = static_cast<GyroContext *>(args);
    if (ctx == NULL)
    {
        ESP_LOGE("LED_TASK", "NULL context!");
        vTaskDelete(NULL);
        return;
    }

    gpio_reset_pin(BLUE_LED_PIN);
    gpio_set_direction(BLUE_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BLUE_LED_PIN, 0);

    EventBits_t bits;
    while (true)
    {
        bits = xEventGroupWaitBits(ctx->station.get_wifi_event(),
                                   WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_RETRY_BIT, pdTRUE,
                                   pdFALSE, portMAX_DELAY);

        if (bits & WIFI_CONNECTED_BIT)
            gpio_set_level(BLUE_LED_PIN, 1);
        else if (bits & WIFI_FAIL_BIT)
            gpio_set_level(BLUE_LED_PIN, 0);
        else if (bits & WIFI_RETRY_BIT)
            status_blink(ctx, 500, 500, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    }
}

void imu_read_task(void *args)
{
    const char *TAG = "IMU_TASK";
    GyroContext *ctx = static_cast<GyroContext *>(args);

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
        ESP_LOGE(TAG, "FATAL: Invalid queues!");
        ESP_LOGE(TAG, "  imu_queue: %p", ctx->imu_queue);
        ESP_LOGE(TAG, "  imu_motor_queue: %p", ctx->imu_motor_queue);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Started successfully");

    const TickType_t sample_period = pdMS_TO_TICKS(30);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        const IMUData &data = ctx->imu.read_imu_data();
        TimestampedIMUData ts_data;
        ts_data.timestamp_ms = (uint32_t)esp_timer_get_time() / 1000;
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
            ESP_LOGE(TAG, "Motor queue became invalid!");
        }

        vTaskDelayUntil(&last_wake_time, sample_period);
    }
}

void motor_control_task(void *args)
{
    const char *TAG = "MOTOR_TASK";
    GyroContext *ctx = static_cast<GyroContext *>(args);

    if (ctx == NULL)
    {
        ESP_LOGE(TAG, "FATAL: NULL context!");
        vTaskDelete(NULL);
        return;
    }

    while (!ctx->queues_initialized)
        vTaskDelay(pdMS_TO_TICKS(10));

    if (!ctx->is_queue_valid(ctx->motor_queue))
    {
        ESP_LOGE(TAG, "FATAL: Invalid motor queue!");
        vTaskDelete(NULL);
        return;
    }

#if TESTING == 0
    if (!ctx->is_queue_valid(ctx->imu_motor_queue))
    {
        ESP_LOGE(TAG, "FATAL: Invalid IMU queue!");
        vTaskDelete(NULL);
        return;
    }
#endif

    ESP_LOGI(TAG, "Started in %s mode", TESTING ? "TESTING" : "BALANCE");

#if TESTING == 0
    float prev_error = 0.0f;
    float integral = 0.0f;
    float velocity = 0.0f;  // Track robot velocity
    TimestampedIMUData imu_data;
    int64_t prev_timestamp = 0;
#endif

    ControlData control = {0.0f, 0.0f, 0};

    while (true)
    {
#if TESTING == 0
        float Kp = ctx->pid_params.kp;
        float Ki = ctx->pid_params.ki;
        float Kd = ctx->pid_params.kd;

        if (!ctx->is_queue_valid(ctx->imu_motor_queue))
        {
            ESP_LOGE(TAG, "Queue corrupted!");
            ctx->motor_driver.set_motor_speeds(0.0f, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (xQueueReceive(ctx->imu_motor_queue, &imu_data, pdMS_TO_TICKS(500)) != pdTRUE)
        {
            ESP_LOGW(TAG, "IMU timeout - stopping motors");
            ctx->motor_driver.set_motor_speeds(0.0f, 0.0f);
            prev_error = 0.0f;
            integral = 0.0f;
            velocity = 0.0f;  // Reset velocity on timeout
            continue;
        }

        if (imu_data.timestamp_ms == prev_timestamp)
            continue;

        // Calculate dt from timestamps
        float dt = 0.01f;  // Default 10ms
        if (prev_timestamp != 0)
        {
            dt = (imu_data.timestamp_ms - prev_timestamp) / 1000.0f;
            dt = fmaxf(0.001f, fminf(dt, 0.05f));  // Clamp to 1-50ms
        }
        prev_timestamp = imu_data.timestamp_ms;

        // MAHONY filter for better response
        const Pitch &pitch = ctx->imu.get_pitch(dt, FilterType::MAHONY);

        float pitch_deg = pitch.angle * 180.0f / M_PI;
        float pitch_rate_deg = pitch.rate * 180.0f / M_PI;

        // Log pitch in degrees
        ESP_LOGI(TAG, "Pitch: %.2f deg, Rate: %.2f deg/s, Vel: %.2f", 
                 pitch_deg, pitch_rate_deg, velocity);

        // Get control input
        if (ctx->is_queue_valid(ctx->motor_queue))
            xQueuePeek(ctx->motor_queue, &control, 0);

        float forward = control.magnitude * cosf(control.angle * M_PI / 180.0f);
        float turn = control.magnitude * sinf(control.angle * M_PI / 180.0f);

        // FIX: Velocity-based control
        // Estimate velocity by integrating motor output (or use encoders if available)
        // For now, we'll use a simple approximation based on pitch angle over time
        velocity += pitch_deg * dt * 0.5f;  // Approximate velocity accumulation
        velocity *= 0.95f;  // Add damping to prevent runaway

        // Target pitch based on forward command MINUS velocity compensation
        float target_pitch = forward * MAX_TILT_ANGLE - velocity * 0.3f;  // Velocity feedback
        
        // PID control (now everything is in degrees)
        float error = target_pitch - pitch_deg;
        integral += error * dt;
        integral = fmaxf(-10.0f, fminf(10.0f, integral));  // Anti-windup
        float derivative = (error - prev_error) / dt;
        prev_error = error;

        // Add derivative term based on pitch RATE for better damping
        float balance_output = Kp * error + Ki * integral + Kd * derivative - (pitch_rate_deg * 0.02f);

        // Apply turn and clamp
        float left_speed = balance_output - (turn * MAX_TURN);
        float right_speed = balance_output + (turn * MAX_TURN);

        left_speed = fmaxf(-1.0f, fminf(1.0f, left_speed));
        right_speed = fmaxf(-1.0f, fminf(1.0f, right_speed));

        ctx->motor_driver.set_motor_speeds(left_speed, right_speed);

        // Debug output
        if (fabsf(pitch_deg) > 1.0f)  // Only log when tilted
        {
            ESP_LOGI(TAG, "Tgt: %.2f, Err: %.2f, Bal: %.2f, L: %.2f, R: %.2f", 
                     target_pitch, error, balance_output, left_speed, right_speed);
        }

#else
        // Testing mode unchanged
        if (xQueueReceive(ctx->motor_queue, &control, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            float forward = control.magnitude * cosf(control.angle * M_PI / 180.0f);
            float turn = control.magnitude * sinf(control.angle * M_PI / 180.0f);

            float left_speed = forward - turn;
            float right_speed = forward + turn;

            left_speed = fmaxf(-1.0f, fminf(1.0f, left_speed));
            right_speed = fmaxf(-1.0f, fminf(1.0f, right_speed));

            ctx->motor_driver.set_motor_speeds(left_speed, right_speed);
        }
#endif
        taskYIELD();
    }
}
void servo_control_task(void *args)
{
    GyroContext *ctx = static_cast<GyroContext *>(args);

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

    Servo servo(SERVO_PIN);
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

void ultrasound_read_task(void *args)
{
    const char *TAG = "US_TASK";
    GyroContext *ctx = static_cast<GyroContext *>(args);

    if (ctx == NULL)
    {
        ESP_LOGE(TAG, "FATAL: NULL context!");
        vTaskDelete(NULL);
        return;
    }

    // // camera trigger gpio setup
    // gpio_reset_pin(CAM_TRIG_PIN);
    // gpio_set_direction(CAM_TRIG_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_level(CAM_TRIG_PIN, 0);

    // Wait for queues
    while (!ctx->queues_initialized)
        vTaskDelay(pdMS_TO_TICKS(10));

    if (!ctx->is_queue_valid(ctx->ultrasound_queue))
    {
        ESP_LOGE(TAG, "FATAL: Invalid queue!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Started successfully");

    ctx->ultrasound.init();
    vTaskDelay(pdMS_TO_TICKS(750));
    TimestampedUltrasound ts_data;

    while (true)
    {
        ts_data.distance_cm = ctx->ultrasound.measure_cm();

        if (ts_data.distance_cm < 0)
        {
            ESP_LOGW(TAG, "Measurement error");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        ts_data.timestamp_ms = (uint32_t)esp_timer_get_time() / 1000;

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

        // gpio_set_level(CAM_TRIG_PIN, 1);
        // vTaskDelay(pdMS_TO_TICKS(10));
        // gpio_set_level(CAM_TRIG_PIN, 0);

        vTaskDelay(pdMS_TO_TICKS(90));
    }
}

void udp_tx_task(void *args)
{
    const char *TAG = "UDP_TX_TASK";
    GyroContext *ctx = static_cast<GyroContext *>(args);

    if (ctx == NULL)
    {
        ESP_LOGE(TAG, "FATAL: NULL context!");
        vTaskDelete(NULL);
        return;
    }

    // Wait for queues AND WiFi
    while (!ctx->queues_initialized)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ctx->station.wait_for_wifi();

    ESP_LOGI(TAG, "Starting...");

    UDPClient udp_tx(UDPMode::TRANSMITTER, ctx->dest_ip, UDP_TX_PORT);

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
                ESP_LOGW(TAG, "Send failed: %d", sent);
            }
        }

        vTaskDelayUntil(&last_wake_time, send_period);
    }
}

void udp_rx_task(void *args)
{
    const char *TAG = "UDP_RX_TASK";
    GyroContext *ctx = static_cast<GyroContext *>(args);

    if (ctx == NULL)
    {
        ESP_LOGE(TAG, "FATAL: NULL context!");
        vTaskDelete(NULL);
        return;
    }

    while (!ctx->queues_initialized)
        vTaskDelay(pdMS_TO_TICKS(10));

    ctx->station.wait_for_wifi();

    ESP_LOGI(TAG, "Starting...");

    UDPClient udp_receiver(UDPMode::RECEIVER, UDP_RX_PORT, 4096);

    int err = udp_receiver.start();
    if (err != 0)
    {
        ESP_LOGE(TAG, "Failed to start: %d", err);
        vTaskDelete(NULL);
        return;
    }

    uint8_t buffer[sizeof(ControlData)];
    ControlData control;
    udp_receiver.set_timeout(3000);

    while (true)
    {
        int ret = udp_receiver.receive_data(buffer, sizeof(buffer));

        if (ret == sizeof(ControlData))
        {
            memcpy(&control, buffer, sizeof(ControlData));
            ESP_LOGI(TAG, "Control: mag=%.2f, ang=%.1f, servo=%d", control.magnitude, control.angle,
                     control.servo_angle);

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
        else if (ret == -11)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else if (ret < 0)
        {
            ESP_LOGW(TAG, "Receive error: %d", ret);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else if (ret > 0)
        {
            ESP_LOGW(TAG, "Unexpected size: %d", ret);
        }
    }
}

void tcp_rx_task(void *args)
{
    const char *TAG = "TCP_RX_TASK";
    GyroContext *ctx = static_cast<GyroContext *>(args);

    ctx->station.wait_for_wifi();

    while (true)
    {
        ESP_LOGI(TAG, "Starting TCP server on port %d...", TCP_RX_PORT);

        TCPClient tcp_receiver(TCPMode::RECEIVER, TCP_RX_PORT, 128);
        int err = tcp_receiver.start();

        if (err != 0)
        {
            ESP_LOGE(TAG, "Failed to start TCP server: %d", err);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        ESP_LOGI(TAG, "TCP server ready, waiting for client...");

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

                ESP_LOGI(TAG, "Received PID: kp=%.2f, ki=%.2f, kd=%.2f | Wi-Fi: %s, Passwd: %s",
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
                ESP_LOGW(TAG, "Receive failed (%d), connection lost", ret);
                break;
            }
            else if (ret > 0)
            {
                ESP_LOGW(TAG, "Unexpected size: %d (expected %d)", ret, expected_size);
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }

        tcp_receiver.end();
        ESP_LOGI(TAG, "Connection closed, restarting server...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void)
{
    initialize_nvs();

    // Create context with all queues
    GyroContext *ctx = new GyroContext();

    // Verify queues before starting tasks
    if (!ctx->queues_initialized)
    {
        ESP_LOGE("MAIN", "FATAL: Queue initialization failed!");
        abort();
    }

    ESP_LOGI("MAIN", "Starting tasks...");

    xTaskCreatePinnedToCore(imu_read_task, "IMU_READ", 3072, ctx, 6, NULL, 1);
    xTaskCreatePinnedToCore(motor_control_task, "MOTOR_CONTROL", 3072, ctx, 6, NULL, 0);
    xTaskCreatePinnedToCore(servo_control_task, "SERVO_CONTROL", 2048, ctx, 4, NULL, 1);
    xTaskCreatePinnedToCore(led_wifi_notifier_task, "LED_NOTIFIER", 1024, ctx, 1, NULL, 1);

    ESP_LOGI("MAIN", "Waiting for WiFi...");
    ctx->station.wait_for_wifi();
    ctx->station.get_broadcast_ip(ctx->dest_ip, sizeof(ctx->dest_ip));
    ESP_LOGI("MAIN", "WiFi connected, starting network tasks");

    xTaskCreatePinnedToCore(ultrasound_read_task, "ULTRASOUND_READ", 2048, ctx, 3, NULL, 1);
    xTaskCreatePinnedToCore(udp_tx_task, "UDP_TX", 4096, ctx, 5, NULL, 0);
    xTaskCreatePinnedToCore(udp_rx_task, "UDP_RX", 3072, ctx, 7, NULL, 0);
    xTaskCreatePinnedToCore(tcp_rx_task, "TCP_RX", 3072, ctx, 2, NULL, 1);

    ESP_LOGI("MAIN", "All tasks started successfully");
}
