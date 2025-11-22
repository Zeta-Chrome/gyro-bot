#include "servo.hpp"
#include <driver/ledc.h>

Servo::Servo(gpio_num_t pwm)
:m_pwm(pwm)
{
    // Timer configuration (same as before)
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = SERVO_LEDC_MODE;
    ledc_timer.duty_resolution = SERVO_PWM_DUTY_RES;
    ledc_timer.timer_num = SERVO_LEDC_TIMER;
    ledc_timer.freq_hz = SERVO_PWM_FREQUENCY;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer)); 

    ledc_channel_config_t channel = {};
    channel.gpio_num = pwm;
    channel.speed_mode = SERVO_LEDC_MODE;
    channel.channel = SERVO_LEDC_CHANNEL;
    channel.timer_sel = SERVO_LEDC_TIMER;
    channel.duty = map_angle_to_pwm(0);
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

size_t Servo::map_angle_to_pwm(float angle) 
{
    // Clamp angle to [-90, 90]
    if (angle < -90.0f) angle = -90.0f;
    if (angle >  90.0f) angle =  90.0f;
    
    // Map -90° to 90° linearly to min-max duty
    // -90° → SERVO_PWM_MIN (205)
    //   0° → midpoint (307.5)
    //  90° → SERVO_PWM_MAX (410)
    float normalized = (90.0f + angle) / 180.0f;  // 0.0 to 1.0
    return SERVO_PWM_1MS + (size_t)(normalized * (SERVO_PWM_2MS - SERVO_PWM_1MS));
}

void Servo::set_angle(float angle)
{
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 110.0f) angle = 110.0f;
    m_angle = angle;

    size_t duty = map_angle_to_pwm(20-angle);
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}

