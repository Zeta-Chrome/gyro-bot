#pragma once

#include <esp_event.h>
#include <esp_wifi.h>
#include <freertos/event_groups.h>
#include <esp_netif.h>

#define MAXIMUM_RETRY 30
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIFI_RETRY_BIT BIT2

// Default WiFi credentials (used if not provisioned)
#define DEFAULT_WIFI_SSID "NPhone"
#define DEFAULT_WIFI_PASSWORD "wifi@2.4"

class WiFiStation
{
public:
    WiFiStation();
    ~WiFiStation();
    const EventGroupHandle_t& get_wifi_event();
    
    void change_wifi(const char* new_ssid, const char* new_password);
    void wait_for_wifi();
    void get_broadcast_ip(char* buffer, size_t len);

private:
    void start_wifi();
    bool load_credentials(char* ssid, char* password);
    bool save_credentials(const char* ssid, const char* password);
    
    static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                                   int32_t event_id, void* event_data);
    
    esp_event_handler_instance_t m_wifi_event_instance;
    esp_event_handler_instance_t m_ip_event_instance;
    EventGroupHandle_t m_event_group;
    const char* m_TAG = "WIFI";
    esp_netif_t* staNetif;
    size_t m_retry_num = 0;
};


