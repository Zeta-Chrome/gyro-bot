#pragma once

#include <driver/ledc.h>
#include <driver/gpio.h>

constexpr ledc_timer_t SERVO_LEDC_TIMER = LEDC_TIMER_1;
constexpr ledc_channel_t SERVO_LEDC_CHANNEL = LEDC_CHANNEL_5;
constexpr ledc_mode_t SERVO_LEDC_MODE  = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_bit_t SERVO_PWM_DUTY_RES = LEDC_TIMER_12_BIT; // range = 0 - 2^12-1
constexpr size_t SERVO_PWM_1MS = 85;
constexpr size_t SERVO_PWM_2MS = 529;
constexpr size_t SERVO_PWM_FREQUENCY = 50; // 50

class Servo
{
public:
    Servo(gpio_num_t pwm);
    ~Servo() = default; 
    
    void set_angle(float angle);

private:
    size_t map_angle_to_pwm(float angle);

private:
    gpio_num_t m_pwm;
    volatile float m_angle;
};

