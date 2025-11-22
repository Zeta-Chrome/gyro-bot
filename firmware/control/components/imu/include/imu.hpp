#pragma once

#include "config.hpp"
#include <esp_log.h>

struct IMUData
{
    // Accelerometer (g's)
    float ax = 0;
    float ay = 0;
    float az = 0;
    // Gyroscope (deg/s)
    float gx = 0;
    float gy = 0;
    float gz = 0;
};

struct Pitch 
{
    float angle = 0;  // y-axis rotation (radians)
    float rate = 0; // For PID
};

// MPU6500 IMU with Mahony filter for 3-axis orientation
class IMU
{
public:
    IMU();
    ~IMU();
    
    inline esp_err_t read_register(uint8_t reg_addr, uint8_t* data, size_t len)
    {
        return i2c_master_transmit_receive(m_dev_handle, &reg_addr, 1, data, len,
                                           I2C_MASTER_TIMEOUT_MS);
    }
    
    inline esp_err_t write_register(uint8_t reg_addr, uint8_t data)
    {
        uint8_t write_buf[2] = {reg_addr, data};
        return i2c_master_transmit(m_dev_handle, write_buf, sizeof(write_buf),
                                   I2C_MASTER_TIMEOUT_MS);
    }
    
    bool is_initialized();
    esp_err_t set_dlpf_bw(DLPFConfig bw);
    float accel_sensitivity(AccelRange range);
    float gyro_sensitivity(GyroRange range);
    esp_err_t set_gyro_sample_rate(uint8_t sample_ms);
    esp_err_t set_accel_range(AccelRange range);
    esp_err_t set_gyro_range(GyroRange range);
    void calibrate();
    
    // Thread-safe read methods
    const IMUData& read_imu_data();
    const Pitch& get_pitch(float delay_s, FilterType type = FilterType::MAHONY);
    
    // Get raw IMU data without filtering
    inline const IMUData& get_raw_data() const { return m_data; }
    
private:
    void i2c_master_init();
    esp_err_t load_calibration();
    esp_err_t save_calibration();
    void mahony_filter_update(float dt);
    
private:
    const char* m_TAG = "IMU";
    bool m_is_initialized = false;
    bool m_is_calibrated = false;
    bool m_dlpf_en = false;
    i2c_master_bus_handle_t m_bus_handle;
    i2c_master_dev_handle_t m_dev_handle;
    
    IMUData m_data;
    IMUData m_bias;
    Pitch m_pitch;

    float m_accelSens = 16384;
    float m_gyroSens = 131;
    
    // Mahony filter quaternion (orientation)
    float m_q0 = 1.0f;
    float m_q1 = 0.0f;
    float m_q2 = 0.0f;
    float m_q3 = 0.0f;
    
    // Mahony filter integral feedback terms
    float m_ex_int = 0.0f;
    float m_ey_int = 0.0f;
    float m_ez_int = 0.0f;
    
    // High-pass filter for acceleration (remove gravity/bias drift)
    float m_ax_filtered = 0.0f;
    float m_ay_filtered = 0.0f;
    float m_az_filtered = 0.0f;
};
