#include "ultrasound_sensor.hpp"
#include <esp_log.h>

portMUX_TYPE UltrasoundSensor::s_mux = portMUX_INITIALIZER_UNLOCKED;

UltrasoundSensor::UltrasoundSensor(gpio_num_t trig, gpio_num_t echo) : m_trig(trig), m_echo(echo) {}

UltrasoundSensor::~UltrasoundSensor()
{
    gpio_reset_pin(m_trig);
    gpio_reset_pin(m_echo);
}

esp_err_t UltrasoundSensor::init()
{
    esp_err_t err;

    err = gpio_set_direction(m_trig, GPIO_MODE_OUTPUT);
    if (err != ESP_OK)
        return err;

    err = gpio_set_direction(m_echo, GPIO_MODE_INPUT);
    if (err != ESP_OK)
        return err;

    return gpio_set_level(m_trig, 0);
}

float UltrasoundSensor::measure_cm(uint32_t max_distance_cm)
{
    // Absolute maximum time this function can take
    int64_t function_start = esp_timer_get_time();
    const int64_t MAX_FUNCTION_TIME_US = 50000; // 50ms hard limit
    
    // Ensure minimum delay between measurements
    int64_t now = esp_timer_get_time();
    int64_t time_since_last = now - m_last_measurement_time;
    if (m_last_measurement_time != 0 && time_since_last < MIN_DELAY_BETWEEN_MEASUREMENTS_US)
    {
        int64_t wait_time = MIN_DELAY_BETWEEN_MEASUREMENTS_US - time_since_last;
        ets_delay_us(wait_time);
    }
    
    // Critical section ONLY for trigger pulse (keeps it short!)
    portENTER_CRITICAL(&s_mux);
    gpio_set_level(m_trig, 0);
    ets_delay_us(TRIGGER_LOW_DELAY);
    gpio_set_level(m_trig, 1);
    ets_delay_us(TRIGGER_HIGH_DELAY);
    gpio_set_level(m_trig, 0);
    portEXIT_CRITICAL(&s_mux);  // Exit critical section immediately
    
    // Check if echo is already high (error condition)
    if (gpio_get_level(m_echo))
    {
        m_last_measurement_time = esp_timer_get_time();
        ESP_LOGW("US", "Echo already high before measurement");
        return -1.0f;
    }
    
    // Wait for echo to start - with dual timeout protection
    int64_t start = esp_timer_get_time();
    while (!gpio_get_level(m_echo))
    {
        int64_t now = esp_timer_get_time();
        
        // Hard timeout check (absolute safety)
        if ((now - function_start) > MAX_FUNCTION_TIME_US)
        {
            m_last_measurement_time = now;
            ESP_LOGE("US", "Hard timeout waiting for echo start");
            return -1.0f;
        }
        
        // Normal timeout check
        if (timeout_expired(start, PING_TIMEOUT))
        {
            m_last_measurement_time = esp_timer_get_time();
            ESP_LOGW("US", "Timeout waiting for echo start");
            return -1.0f;
        }
    }
    
    // Measure echo duration - with dual timeout protection
    int64_t echo_start = esp_timer_get_time();
    int64_t time = echo_start;
    uint32_t max_time_us = max_distance_cm * ROUNDTRIP_CM;
    
    while (gpio_get_level(m_echo))
    {
        time = esp_timer_get_time();
        
        // Hard timeout check (absolute safety)
        if ((time - function_start) > MAX_FUNCTION_TIME_US)
        {
            m_last_measurement_time = time;
            ESP_LOGE("US", "Hard timeout measuring echo duration");
            return -1.0f;
        }
        
        // Normal timeout check
        if (timeout_expired(echo_start, max_time_us))
        {
            m_last_measurement_time = esp_timer_get_time();
            return 0.0f; // Object too close or invalid reading
        }
    }
    
    m_last_measurement_time = esp_timer_get_time();
    
    uint32_t time_us = time - echo_start;
    float distance_cm = time_us / ROUNDTRIP_CM;
    
    return distance_cm;
}
