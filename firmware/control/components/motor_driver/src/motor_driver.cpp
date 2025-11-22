#include "motor_driver.hpp"
#include <math.h>

MotorDriver::MotorDriver(gpio_num_t in1, gpio_num_t in2, gpio_num_t in3, gpio_num_t in4)
    : lmotor_in1(in1), lmotor_in2(in2), rmotor_in1(in3), rmotor_in2(in4)
{
    // Timer configuration (same as before)
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = MOTOR_LEDC_MODE;
    ledc_timer.duty_resolution = MOTOR_PWM_DUTY_RES;
    ledc_timer.timer_num = MOTOR_LEDC_TIMER;
    ledc_timer.freq_hz = MOTOR_PWM_FREQUENCY;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure all 4 pins as PWM channels
    ledc_channel_config_t channels[4] = {};

    channels[0].gpio_num = lmotor_in1;
    channels[0].speed_mode = LEDC_LOW_SPEED_MODE;
    channels[0].channel = LEDC_CHANNEL_0;
    channels[0].timer_sel = LEDC_TIMER_0;

    channels[1].gpio_num = lmotor_in2;
    channels[1].speed_mode = LEDC_LOW_SPEED_MODE;
    channels[1].channel = LEDC_CHANNEL_1;
    channels[1].timer_sel = LEDC_TIMER_0;

    channels[2].gpio_num = rmotor_in1;
    channels[2].speed_mode = LEDC_LOW_SPEED_MODE;
    channels[2].channel = LEDC_CHANNEL_2;
    channels[2].timer_sel = LEDC_TIMER_0;

    channels[3].gpio_num = rmotor_in2;
    channels[3].speed_mode = LEDC_LOW_SPEED_MODE;
    channels[3].channel = LEDC_CHANNEL_3;
    channels[3].timer_sel = LEDC_TIMER_0;

    for (int i = 0; i < 4; i++)
    {
        ESP_ERROR_CHECK(ledc_channel_config(&channels[i]));
    }
}

// Set a single motor pin pair using one PWM channel
void MotorDriver::set_channel_speed(float speed, ledc_channel_t channel_positive,
                                    ledc_channel_t channel_negative, float factor)
{
    uint32_t duty = (MOTOR_PWM_MIN_DUTY +
                     (uint32_t)(fabsf(speed) * (MOTOR_PWM_MAX_DUTY - MOTOR_PWM_MIN_DUTY))) *
                    factor;

    if (speed == 0.0f)
        duty = 0;

    if (speed >= 0)
    {
        ledc_set_duty(MOTOR_LEDC_MODE, channel_positive, duty);
        ledc_set_duty(MOTOR_LEDC_MODE, channel_negative, 0);
    }
    else
    {
        ledc_set_duty(MOTOR_LEDC_MODE, channel_positive, 0);
        ledc_set_duty(MOTOR_LEDC_MODE, channel_negative, duty);
    }

    ledc_update_duty(MOTOR_LEDC_MODE, channel_positive);
    ledc_update_duty(MOTOR_LEDC_MODE, channel_negative);
}

void MotorDriver::set_motor_speeds(float left_speed, float right_speed)
{
    set_channel_speed(left_speed, LEDC_CHANNEL_0, LEDC_CHANNEL_1, MOTOR_FACTOR);
    set_channel_speed(right_speed, LEDC_CHANNEL_2, LEDC_CHANNEL_3, 1.0f);
}

void MotorDriver::brake_motor(ledc_channel_t channel_positive, ledc_channel_t channel_negative)
{
    ledc_set_duty(MOTOR_LEDC_MODE, channel_positive, MOTOR_PWM_MAX_DUTY);
    ledc_update_duty(MOTOR_LEDC_MODE, channel_positive);

    ledc_set_duty(MOTOR_LEDC_MODE, channel_negative, MOTOR_PWM_MAX_DUTY);
    ledc_update_duty(MOTOR_LEDC_MODE, channel_negative);
}

void MotorDriver::brake_left_motor()
{
    brake_motor(LEDC_CHANNEL_0, LEDC_CHANNEL_1);
}

void MotorDriver::brake_right_motor()
{
    brake_motor(LEDC_CHANNEL_2, LEDC_CHANNEL_3);
}

void MotorDriver::brake()
{
    brake_motor(LEDC_CHANNEL_0, LEDC_CHANNEL_1);
    brake_motor(LEDC_CHANNEL_2, LEDC_CHANNEL_3);
}
