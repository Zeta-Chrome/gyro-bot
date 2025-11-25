#pragma once

#include <wifi_station.hpp>
#include <imu.hpp>
#include <ultrasound_sensor.hpp>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>

struct TimestampedIMUData
{
    int64_t timestamp_us;
    float ax, ay, az;
    float gx, gy, gz;
};

struct TimestampedUltrasound
{
    int64_t timestamp_us;
    float distance_cm;
};

struct ControlData
{
    float magnitude;
    float angle;
    int32_t servo_angle;
};

struct PIDParams
{
    float kp = 1.0f;
    float kd = 0.0f;
    float ki = 0.0f;
};

class GyroContext
{
public:
    GyroContext();
    
    void save_pid_to_nvs();
    void load_pid_from_nvs();
    bool is_queue_valid(QueueHandle_t queue);

public:
    WiFiStation station;
    IMU imu;
    UltrasoundSensor ultrasound;
    PIDParams pid_params;

    QueueHandle_t imu_queue;
    QueueHandle_t imu_motor_queue;
    QueueHandle_t ultrasound_queue;
    QueueHandle_t motor_queue;
    QueueHandle_t servo_queue;

    SemaphoreHandle_t ctx_mutex;  // Protect context access
    volatile bool queues_initialized;
};
