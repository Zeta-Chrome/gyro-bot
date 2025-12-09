#include "imu.hpp"
#include "config.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <math.h>
#include <nvs.h>
#include <sys/param.h>

#define NVS_NAMESPACE "imu"

IMU::IMU()
{
    i2c_master_init();
    ESP_LOGI(m_TAG, "I2C initialized successfully");

    uint8_t buffer;
    ESP_ERROR_CHECK(read_register(WHO_AM_I_REG, &buffer, 1));
    if (buffer != 0x70)
    {
        ESP_LOGE(m_TAG, "Failed to initialize the IMU");
        return;
    }

    ESP_ERROR_CHECK(write_register(PWR_MGMT_1_REG, 0x00));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(m_TAG, "MPU6500 woken up from sleep mode");

    m_is_initialized = true;
    ESP_LOGI(m_TAG, "IMU initialized successfully");

    if (load_calibration() == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(m_TAG, "IMU is not calibrated");
    }

    // Initialize quaternion to identity
    m_q0 = 1.0f;
    m_q1 = 0.0f;
    m_q2 = 0.0f;
    m_q3 = 0.0f;
}

IMU::~IMU()
{
    if (m_dev_handle)
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(m_dev_handle));
    if (m_bus_handle)
        ESP_ERROR_CHECK(i2c_del_master_bus(m_bus_handle));
    ESP_LOGI(m_TAG, "I2C de-initialized successfully");
}

void IMU::i2c_master_init()
{
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = I2C_MASTER_NUM;
    bus_config.sda_io_num = I2C_MASTER_SDA_IO;
    bus_config.scl_io_num = I2C_MASTER_SCL_IO;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = false;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &m_bus_handle));

    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = SENSOR_ADDR;
    dev_config.scl_speed_hz = I2C_MASTER_FREQ_HZ;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(m_bus_handle, &dev_config, &m_dev_handle));
}

bool IMU::is_initialized()
{
    return m_is_initialized;
}

esp_err_t IMU::set_dlpf_bw(DLPFConfig bw)
{
    m_dlpf_en = false;
    if (bw != DLPFConfig::BW_256Hz)
        m_dlpf_en = true;
    return write_register(CFG_REG, (uint8_t)bw);
}

esp_err_t IMU::set_gyro_sample_rate(uint8_t sample_ms)
{
    size_t max_gyro_rate = m_dlpf_en ? 1000 : 8000;  // 1kHz : 8kHz
    int32_t smplrt_div = sample_ms * max_gyro_rate / 1000 - 1;
    smplrt_div = MAX(0, MIN(255, smplrt_div));
    return write_register(SMPLRT_DIV_REG, (uint8_t)smplrt_div);
}

float IMU::accel_sensitivity(AccelRange range)
{
    switch (range)
    {
        case AccelRange::FS_2G:
            return 16384.0f;
        case AccelRange::FS_4G:
            return 8192.0f;
        case AccelRange::FS_8G:
            return 4096.0f;
        case AccelRange::FS_16G:
            return 2048.0f;
    }
    return 16384.0f;
}

float IMU::gyro_sensitivity(GyroRange range)
{
    switch (range)
    {
        case GyroRange::FS_250:
            return 131.0f;
        case GyroRange::FS_500:
            return 65.5f;
        case GyroRange::FS_1000:
            return 32.8f;
        case GyroRange::FS_2000:
            return 16.4f;
    }
    return 131.0f;
}

esp_err_t IMU::set_accel_range(AccelRange range)
{
    m_accelSens = accel_sensitivity(range);
    return write_register(ACCEL_CFG_REG, (uint8_t)range);
}

esp_err_t IMU::set_gyro_range(GyroRange range)
{
    m_gyroSens = gyro_sensitivity(range);
    return write_register(GYRO_CFG_REG, (uint8_t)range);
}

void IMU::calibrate()
{
    uint8_t buffer[14];
    for (size_t i = 0; i < MAX_CALIBRATION_COUNT; i++)
    {
        if (read_register(ACCEL_OUT_REG, buffer, 14) == ESP_OK)
        {
            // FIXED: Correct axis order
            m_bias.ax += (float)(int16_t)((buffer[0] << 8) | buffer[1]) / m_accelSens;
            m_bias.ay += (float)(int16_t)((buffer[2] << 8) | buffer[3]) / m_accelSens;
            m_bias.az += (float)(int16_t)((buffer[4] << 8) | buffer[5]) / m_accelSens;
            m_bias.gx += (float)(int16_t)((buffer[8] << 8) | buffer[9]) / m_gyroSens;
            m_bias.gy += (float)(int16_t)((buffer[10] << 8) | buffer[11]) / m_gyroSens;
            m_bias.gz += (float)(int16_t)((buffer[12] << 8) | buffer[13]) / m_gyroSens;
        }

        vTaskDelay(pdMS_TO_TICKS(10));

        if (i % 100 == 0 && i > 0)
        {
            ESP_LOGI(m_TAG, "Progress: %zu/%d", i, MAX_CALIBRATION_COUNT);
        }
    }

    m_bias.ax /= MAX_CALIBRATION_COUNT;
    m_bias.ay /= MAX_CALIBRATION_COUNT;
    m_bias.az /= MAX_CALIBRATION_COUNT;
    m_bias.gx /= MAX_CALIBRATION_COUNT;
    m_bias.gy /= MAX_CALIBRATION_COUNT;
    m_bias.gz /= MAX_CALIBRATION_COUNT;

    m_bias.az -= 1.0f;

    ESP_ERROR_CHECK(save_calibration());
    ESP_LOGI(m_TAG, "Calibration complete!");
    ESP_LOGI(m_TAG, "Accel bias: ax=%.4f, ay=%.4f, az=%.4f g", m_bias.ax, m_bias.ay, m_bias.az);
    ESP_LOGI(m_TAG, "Gyro bias:  gx=%.4f, gy=%.4f, gz=%.4f deg/s", m_bias.gx, m_bias.gy, m_bias.gz);
}

esp_err_t IMU::load_calibration()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK)
        return err;

    size_t size = sizeof(m_bias);
    err = nvs_get_blob(handle, "bias", &m_bias, &size);
    nvs_close(handle);

    if (err == ESP_OK)
    {
        ESP_LOGI(m_TAG, "Calibration loaded from NVS");
    }

    return err;
}

esp_err_t IMU::save_calibration()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    nvs_set_blob(handle, "bias", &m_bias, sizeof(m_bias));

    err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

const IMUData& IMU::read_imu_data()
{
    uint8_t buffer[14];
    if (read_register(ACCEL_OUT_REG, buffer, 14) == ESP_OK)
    {
        m_data.ax = (float)(int16_t)((buffer[0] << 8) | buffer[1]) / m_accelSens - m_bias.ax;
        m_data.ay = (float)(int16_t)((buffer[2] << 8) | buffer[3]) / m_accelSens - m_bias.ay;
        m_data.az = (float)(int16_t)((buffer[4] << 8) | buffer[5]) / m_accelSens - m_bias.az;
        m_data.gx = (float)(int16_t)((buffer[8] << 8) | buffer[9]) / m_gyroSens - m_bias.gx;
        m_data.gy = (float)(int16_t)((buffer[10] << 8) | buffer[11]) / m_gyroSens - m_bias.gy;
        m_data.gz = (float)(int16_t)((buffer[12] << 8) | buffer[13]) / m_gyroSens - m_bias.gz;
    }
    else
    {
        ESP_LOGW(m_TAG, "Failed to read IMU");
    }

    return m_data;
}

void IMU::mahony_filter_update(float dt)
{
    // Convert gyro from deg/s to rad/s
    float gx = m_data.gx * M_PI / 180.0f;
    float gy = m_data.gy * M_PI / 180.0f;
    float gz = m_data.gz * M_PI / 180.0f;

    // Normalize accelerometer
    float norm = sqrtf(m_data.ax * m_data.ax + m_data.ay * m_data.ay + m_data.az * m_data.az);
    if (norm == 0.0f)
        return;
    float ax = m_data.ax / norm;
    float ay = m_data.ay / norm;
    float az = m_data.az / norm;

    // Estimated direction of gravity from quaternion
    float half_vx = m_q1 * m_q3 - m_q0 * m_q2;
    float half_vy = m_q0 * m_q1 + m_q2 * m_q3;
    float half_vz = m_q0 * m_q0 - 0.5f + m_q3 * m_q3;

    // Error is cross product between estimated and measured gravity
    float ex = ay * half_vz - az * half_vy;
    float ey = az * half_vx - ax * half_vz;
    float ez = ax * half_vy - ay * half_vx;

    // Integral feedback (anti-windup)
    m_ex_int += ex * KI * dt;
    m_ey_int += ey * KI * dt;
    m_ez_int += ez * KI * dt;
    
    // Clamp integral terms to prevent windup
    const float INT_LIMIT = 0.5f;
    m_ex_int = fmaxf(-INT_LIMIT, fminf(INT_LIMIT, m_ex_int));
    m_ey_int = fmaxf(-INT_LIMIT, fminf(INT_LIMIT, m_ey_int));
    m_ez_int = fmaxf(-INT_LIMIT, fminf(INT_LIMIT, m_ez_int));

    // Apply proportional and integral feedback
    float gx_c = gx + KP * ex + m_ex_int;
    float gy_c = gy + KP * ey + m_ey_int;
    float gz_c = gz + KP * ez + m_ez_int;

    // Quaternion rate of change
    float q_dot0 = 0.5f * (-m_q1 * gx_c - m_q2 * gy_c - m_q3 * gz_c);
    float q_dot1 = 0.5f * (m_q0 * gx_c + m_q2 * gz_c - m_q3 * gy_c);
    float q_dot2 = 0.5f * (m_q0 * gy_c - m_q1 * gz_c + m_q3 * gx_c);
    float q_dot3 = 0.5f * (m_q0 * gz_c + m_q1 * gy_c - m_q2 * gx_c);

    // Integrate quaternion
    m_q0 += q_dot0 * dt;
    m_q1 += q_dot1 * dt;
    m_q2 += q_dot2 * dt;
    m_q3 += q_dot3 * dt;

    // Normalize quaternion
    norm = sqrtf(m_q0 * m_q0 + m_q1 * m_q1 + m_q2 * m_q2 + m_q3 * m_q3);
    if (norm == 0.0f)
        return;
    m_q0 /= norm;
    m_q1 /= norm;
    m_q2 /= norm;
    m_q3 /= norm;
    
    m_pitch.rate = gy_c;
}

const Pitch& IMU::get_pitch(float delay_s, FilterType type)
{
    if (type == FilterType::MAHONY)
    {
        mahony_filter_update(delay_s);

        // Calculate pitch (y-axis rotation) from quaternion
        float sinp = 2.0f * (m_q0 * m_q2 - m_q3 * m_q1);
        if (fabsf(sinp) >= 1.0f)
            m_pitch.angle = copysignf(M_PI / 2.0f, sinp);
        else
            m_pitch.angle = asinf(sinp);
    }
    else
    {
        float accel_pitch = atan2f(-m_data.ax, sqrtf(m_data.ay * m_data.ay + m_data.az * m_data.az));
        
        // Integrate gyro
        m_pitch.angle += m_data.gy * delay_s;
        
        // Complementary filter fusion
        m_pitch.angle = ALPHA * m_pitch.angle + (1.0f - ALPHA) * accel_pitch;
        m_pitch.rate = m_data.gy;
    }

    return m_pitch;
}
