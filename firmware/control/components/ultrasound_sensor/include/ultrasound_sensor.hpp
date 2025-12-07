#pragma once
#include "portmacro.h"
#include <driver/gpio.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>
#include <esp_err.h>

class UltrasoundSensor
{
public:
    UltrasoundSensor(gpio_num_t trig, gpio_num_t echo);
    ~UltrasoundSensor();
    
    esp_err_t init();
    
    // Measure distance in centimeters (returns -1.0f on error)
    float measure_cm(uint32_t max_distance_cm = 600);
    
private:
    gpio_num_t m_trig;
    gpio_num_t m_echo;
    
    static constexpr uint32_t TRIGGER_LOW_DELAY = 4;
    static constexpr uint32_t TRIGGER_HIGH_DELAY = 10;
    static constexpr uint32_t PING_TIMEOUT = 6000;
    static constexpr uint32_t MIN_DELAY_BETWEEN_MEASUREMENTS_US = 60000; // 60ms minimum
    static constexpr float ROUNDTRIP_CM = 58.0f;
    
    int64_t m_last_measurement_time = 0;
    
    static portMUX_TYPE s_mux;
    
    inline bool timeout_expired(int64_t start, uint32_t len) {
        return (esp_timer_get_time() - start) >= len;
    }
};

