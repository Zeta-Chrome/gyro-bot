#pragma once

#include <cstdint>
#include "driver/i2c_master.h"

// Configs
constexpr size_t MAX_CALIBRATION_COUNT = 1000;
constexpr float ALPHA = 0.9f;

// Ports and Pins
constexpr i2c_port_t I2C_MASTER_NUM = I2C_NUM_0;
constexpr gpio_num_t I2C_MASTER_SDA_IO = GPIO_NUM_21;
constexpr gpio_num_t I2C_MASTER_SCL_IO = GPIO_NUM_22;
constexpr size_t I2C_MASTER_FREQ_HZ = 400000;
constexpr size_t I2C_MASTER_TIMEOUT_MS = 1000;

// Registers
constexpr uint8_t SMPLRT_DIV_REG = 0x19;
constexpr uint8_t CFG_REG = 0x1A;
constexpr uint8_t GYRO_CFG_REG = 0x1B;
constexpr uint8_t ACCEL_CFG_REG = 0x1C;
constexpr uint8_t ACCEL_OUT_REG = 0x3B;
constexpr uint8_t GYRO_OUT_REG = 0x43;
constexpr uint8_t PWR_MGMT_1_REG = 0x6B;
constexpr uint8_t PWR_MGMT_2_REG = 0x6C;
constexpr uint8_t SENSOR_ADDR = 0x68;
constexpr uint8_t WHO_AM_I_REG = 0x75;

// Mahony filter config
// Higher KP = faster convergence, better for dynamic movement
// Higher KI = better long-term stability, removes gyro drift
constexpr float KP = 2.0f;   // Increased from 1.0 for faster response
constexpr float KI = 0.5f;   // Increased from 0.3 for better drift correction

enum class DLPFConfig : uint8_t
{
    BW_256Hz = 0,
    BW_188Hz = 1,
    BW_98Hz = 2,
    BW_42Hz = 3,
    BW_20Hz = 4,
    BW_10Hz = 5,
    BW_5Hz = 6,
};

enum class GyroRange : uint8_t
{
    FS_250 = 0 << 3,
    FS_500 = 1 << 3,
    FS_1000 = 2 << 3,
    FS_2000 = 3 << 3
};

enum class AccelRange : uint8_t
{
    FS_2G = 0 << 3,
    FS_4G = 1 << 3,
    FS_8G = 2 << 3,
    FS_16G = 3 << 3
};

enum class FilterType : uint8_t
{
    COMPLEMENTARY = 0,
    MAHONY = 1
};
