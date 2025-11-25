#include "gyro_context.hpp"
#include <nvs_flash.h>

GyroContext::GyroContext() : imu(), ultrasound(GPIO_NUM_25, GPIO_NUM_19), queues_initialized(false)
{
    load_pid_from_nvs();

    // Create mutex first
    ctx_mutex = xSemaphoreCreateMutex();
    if (ctx_mutex == NULL)
    {
        ESP_LOGE("GYRO_CTX", "Failed to create mutex!");
        abort();
    }

    // Create all queues with validation
    imu_queue = xQueueCreate(15, sizeof(TimestampedIMUData));
    imu_motor_queue = xQueueCreate(1, sizeof(TimestampedIMUData));
    ultrasound_queue = xQueueCreate(5, sizeof(TimestampedUltrasound));
    motor_queue = xQueueCreate(1, sizeof(ControlData));
    servo_queue = xQueueCreate(1, sizeof(int32_t));

    // Validate all queues created successfully
    if (imu_queue == NULL || imu_motor_queue == NULL || ultrasound_queue == NULL
        || motor_queue == NULL || servo_queue == NULL)
    {
        ESP_LOGE("GYRO_CTX", "FATAL: Failed to create queues!");
        abort();
    }

    queues_initialized = true;

    ESP_LOGI("GYRO_CTX", "All queues initialized successfully");
    ESP_LOGI("GYRO_CTX", "  imu_queue: %p", imu_queue);
    ESP_LOGI("GYRO_CTX", "  imu_motor_queue: %p", imu_motor_queue);
    ESP_LOGI("GYRO_CTX", "  ultrasound_queue: %p", ultrasound_queue);
    ESP_LOGI("GYRO_CTX", "  motor_queue: %p", motor_queue);
    ESP_LOGI("GYRO_CTX", "  servo_queue: %p", servo_queue);
}

void GyroContext::save_pid_to_nvs()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("pid", NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Failed to open NVS for PID save: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(handle, "pid", &pid_params, sizeof(PIDParams));
    if (err == ESP_OK)
    {
        nvs_commit(handle);
        ESP_LOGI("NVS", "PID parameters saved successfully");
    }
    else
    {
        ESP_LOGE("NVS", "Failed to save PID parameters: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
}

void GyroContext::load_pid_from_nvs()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("pid", NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGW("NVS", "No saved PID parameters found, using defaults");
        return;
    }

    size_t required_size = sizeof(PIDParams);
    err = nvs_get_blob(handle, "pid", &pid_params, &required_size);
    nvs_close(handle);

    if (err == ESP_OK)
    {
        ESP_LOGI("NVS", "Loaded PID: kp=%.2f, ki=%.2f, kd=%.2f", pid_params.kp, pid_params.ki,
                 pid_params.kd);
    }
    else
    {
        ESP_LOGW("NVS", "Failed to read PID parameters: %s", esp_err_to_name(err));
    }
}

// Helper to safely check queue validity
bool GyroContext::is_queue_valid(QueueHandle_t queue)
{
    if (!queues_initialized)
        return false;
    if (queue == NULL)
        return false;

    // Try to get queue info (will fail if corrupted)
    UBaseType_t spaces = uxQueueSpacesAvailable(queue);
    return (spaces != 0xFFFFFFFF);  // 0xFFFFFFFF indicates corrupted queue
}
