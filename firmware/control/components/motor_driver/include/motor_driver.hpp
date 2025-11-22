#pragma once

#include <driver/ledc.h>
#include <driver/gpio.h>

constexpr ledc_timer_t MOTOR_LEDC_TIMER = LEDC_TIMER_0;
constexpr ledc_mode_t MOTOR_LEDC_MODE  = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_bit_t MOTOR_PWM_DUTY_RES = LEDC_TIMER_8_BIT; // range = 0 - 2^8-1
constexpr size_t MOTOR_PWM_MIN_DUTY = 76;
constexpr size_t MOTOR_PWM_MAX_DUTY = 255;
constexpr size_t MOTOR_PWM_FREQUENCY = 50000; // 5kHz
constexpr size_t MOTOR_MAX_RPM = 100*10/12;
constexpr float MOTOR_FACTOR = 0.965; // to reduce the left motor speed as its faster

class MotorDriver
{
public:
    MotorDriver(gpio_num_t lin1, gpio_num_t lin2, gpio_num_t rin1, gpio_num_t rin2);
    ~MotorDriver() = default;

    // speed is from -1 -> 1
    void set_motor_speeds(float leftSpeed, float rightSpeed);
    void brake_left_motor();
    void brake_right_motor();
    void brake();

private:
    void set_channel_speed(float speed, ledc_channel_t channel_positive,
                                  ledc_channel_t channel_negative, float factor);
    void brake_motor(ledc_channel_t channel_positive, ledc_channel_t channel_negative);

private:
    gpio_num_t lmotor_in1;
    gpio_num_t lmotor_in2;
    gpio_num_t rmotor_in1;
    gpio_num_t rmotor_in2;
};
