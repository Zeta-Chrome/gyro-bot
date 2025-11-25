#include "wifi_station.hpp"
#include <lwip/ip4_addr.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs.h>
#include <nvs_flash.h>

#define NVS_NAMESPACE "wifi_cfg"
#define NVS_SSID "ssid"
#define NVS_PASS "pass"

WiFiStation::WiFiStation()
{
    m_event_group = xEventGroupCreate();
    m_event_conn = xEventGroupCreate();
    ESP_LOGI(m_TAG, "Initializing Station Mode...");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create netif BEFORE registering handlers
    staNetif = esp_netif_create_default_wifi_sta();

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &WiFiStation::wifi_event_handler, this,
                                                        &m_wifi_event_instance));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &WiFiStation::wifi_event_handler, this,
                                                        &m_ip_event_instance));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    start_wifi();
}

WiFiStation::~WiFiStation()
{
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, m_ip_event_instance);
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, m_wifi_event_instance);
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_netif_destroy(staNetif);
    vEventGroupDelete(m_event_group);
}

bool WiFiStation::load_credentials(char* ssid, char* password)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);

    if (err != ESP_OK)
    {
        // No saved credentials, use defaults
        ESP_LOGI(m_TAG, "No saved WiFi, using defaults");
        strcpy(ssid, DEFAULT_WIFI_SSID);
        strcpy(password, DEFAULT_WIFI_PASSWORD);
        return true;
    }

    size_t ssid_len = 32, pass_len = 64;
    esp_err_t e1 = nvs_get_str(handle, NVS_SSID, ssid, &ssid_len);
    esp_err_t e2 = nvs_get_str(handle, NVS_PASS, password, &pass_len);
    nvs_close(handle);

    if (e1 == ESP_OK && e2 == ESP_OK)
    {
        ESP_LOGI(m_TAG, "Loaded saved WiFi: %s", ssid);
        return true;
    }

    // Fallback to defaults
    ESP_LOGI(m_TAG, "Failed to load, using defaults");
    strcpy(ssid, DEFAULT_WIFI_SSID);
    strcpy(password, DEFAULT_WIFI_PASSWORD);
    return true;
}

bool WiFiStation::save_credentials(const char* ssid, const char* password)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(m_TAG, "Failed to open NVS");
        return false;
    }

    nvs_set_str(handle, NVS_SSID, ssid);
    nvs_set_str(handle, NVS_PASS, password);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(m_TAG, "WiFi credentials saved: %s, %s", ssid, password);
    return true;
}

void WiFiStation::start_wifi()
{
    char ssid[32] = {0};
    char password[64] = {0};

    load_credentials(ssid, password);

    wifi_config_t sta_config = {};
    memcpy(sta_config.sta.ssid, ssid, strlen(ssid));
    memcpy(sta_config.sta.password, password, strlen(password));
    sta_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    sta_config.sta.failure_retry_cnt = 0;
    sta_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    sta_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(m_TAG, "Connecting to: %s", ssid);
}

void WiFiStation::change_wifi(const char* new_ssid, const char* new_password)
{
    ESP_LOGI(m_TAG, "Changing WiFi to: %s", new_ssid);

    // Save new credentials
    save_credentials(new_ssid, new_password);
}

void WiFiStation::wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id,
                                     void* event_data)
{
    WiFiStation* self = static_cast<WiFiStation*>(arg);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
        ESP_LOGI(self->m_TAG, "Station started");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(self->m_TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        self->m_retry_num = 0;
        xEventGroupSetBits(self->m_event_group, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(self->m_event_conn, WIFI_CONNECTED_BIT);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (self->m_retry_num < MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            self->m_retry_num++;
            ESP_LOGI(self->m_TAG, "Retrying to connect to the AP, attempt %d", self->m_retry_num);
            xEventGroupSetBits(self->m_event_group, WIFI_RETRY_BIT);
            xEventGroupClearBits(self->m_event_conn, WIFI_CONNECTED_BIT);
        }
        else
        {
            ESP_LOGI(self->m_TAG, "Failed to connect after %d attempts", MAXIMUM_RETRY);
            xEventGroupSetBits(self->m_event_group, WIFI_FAIL_BIT);
            xEventGroupSetBits(self->m_event_conn, WIFI_CONNECTED_BIT);
        }
    }
}

void WiFiStation::wait_for_wifi()
{
    xEventGroupWaitBits(m_event_conn, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

const EventGroupHandle_t& WiFiStation::get_wifi_event()
{
    return m_event_group;
}

void WiFiStation::get_broadcast_ip(char* buffer, size_t len)
{
    esp_netif_ip_info_t ip_info;
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK)
    {
        uint32_t ip = ip_info.ip.addr;
        uint32_t netmask = ip_info.netmask.addr;

        uint32_t broadcast_ip = (ip & netmask) | (~netmask);

        ip4_addr_t bcast;
        bcast.addr = broadcast_ip;

        snprintf(buffer, len, IPSTR, IP2STR(&bcast));
        ESP_LOGI(m_TAG, "Broadcast IP: %s", buffer);
    }
}
