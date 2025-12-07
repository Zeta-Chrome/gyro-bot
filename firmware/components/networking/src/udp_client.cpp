#include "udp_client.hpp"
#include <esp_log.h>

UDPClient::UDPClient(UDPMode mode, const char* dest_ip, uint16_t port) :
    m_mode(mode), m_port(port), m_sock(-1)
{
    if (m_mode != UDPMode::TRANSMITTER)
    {
        ESP_LOGE(m_TAG, "Wrong constructor for mode");
        return;
    }

    strncpy(m_ip, dest_ip, sizeof(m_ip) - 1);
}

UDPClient::UDPClient(UDPMode mode, uint16_t listen_port, size_t buffer_size) :
    m_mode(mode), m_port(listen_port), m_buffer_size(buffer_size), m_sock(-1)
{
    if (m_mode != UDPMode::RECEIVER)
    {
        ESP_LOGE(m_TAG, "Wrong constructor for mode");
        return;
    }

    memset(m_ip, 0, sizeof(m_ip));
}

UDPClient::~UDPClient()
{
    end();
}

int UDPClient::start()
{
    m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (m_sock < 0)
    {
        ESP_LOGE(m_TAG, "Unable to create socket: errno %d", errno);
        return errno;
    }

    if (m_mode == UDPMode::TRANSMITTER)
    {
        inet_pton(AF_INET, m_ip, &m_addr.sin_addr);
        m_addr.sin_family = AF_INET;
        m_addr.sin_port = htons(m_port);

        int enable = 1;
        if (setsockopt(m_sock, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable)) < 0)
        {
            ESP_LOGE(m_TAG, "Failed to enable broadcast: errno %d", errno);
            close(m_sock);
            m_sock = -1;
            return errno;
        }

        ESP_LOGI(m_TAG, "UDP Broadcast Sender started - dest: %s:%d, sock: %d", m_ip, m_port, m_sock);
    }
    else
    {
        m_addr.sin_addr.s_addr = INADDR_ANY;
        m_addr.sin_family = AF_INET;
        m_addr.sin_port = htons(m_port);

        if (bind(m_sock, (struct sockaddr*)&m_addr, sizeof(m_addr)) < 0)
        {
            ESP_LOGE(m_TAG, "Bind failed: errno %d", errno);
            close(m_sock);
            m_sock = -1;
            return errno;
        }

        ESP_LOGI(m_TAG, "UDP Receiver started - port: %d, sock: %d, buffer: %d", m_port, m_sock, m_buffer_size);
    }

    return 0;
}

void UDPClient::end()
{
    if (m_sock >= 0)
    {
        close(m_sock);
        m_sock = -1;
    }
}

int UDPClient::set_timeout(size_t timeout_ms)
{
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    if (setsockopt(m_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
        ESP_LOGE(m_TAG, "Failed to set socket timeout: errno %d", errno);
        return -1;
    }

    return 0;
}

int UDPClient::send_data(const uint8_t* payload, size_t length)
{
    if (m_mode != UDPMode::TRANSMITTER)
    {
        ESP_LOGE(m_TAG, "Cannot send - not in TRANSMITTER mode");
        return -1;
    }

    if (!payload || length == 0)
    {
        ESP_LOGE(m_TAG, "Invalid payload or length");
        return 0;
    }

    if (m_sock < 0)
    {
        ESP_LOGE(m_TAG, "Socket not initialized");
        return -1;
    }

    int err = sendto(m_sock, payload, length, 0, (struct sockaddr*)&m_addr, sizeof(m_addr));

    if (err < 0)
    {
        ESP_LOGE(m_TAG, "Send failed: errno %d", errno);
        return -errno;
    }

    return err;
}

int UDPClient::receive_data(uint8_t* buffer, size_t buffer_len)
{
    if (m_mode != UDPMode::RECEIVER)
    {
        ESP_LOGE(m_TAG, "Cannot receive - not in RECEIVER mode");
        return -1;
    }

    if (!buffer || buffer_len == 0)
    {
        ESP_LOGE(m_TAG, "Invalid buffer or length");
        return -1;
    }

    if (m_sock < 0)
    {
        ESP_LOGE(m_TAG, "Socket not initialized");
        return -1;
    }

    int len = recvfrom(m_sock, buffer, buffer_len, 0, NULL, NULL);

    if (len < 0)
    {
        return -errno;
    }

    return len;
}

int UDPClient::receive_ip(char* buffer)
{
    if (m_mode != UDPMode::RECEIVER)
    {
        ESP_LOGE(m_TAG, "Cannot receive - not in RECEIVER mode");
        return -1;
    }

    if (!buffer)
    {
        ESP_LOGE(m_TAG, "Invalid buffer or length");
        return -1;
    }

    if (m_sock < 0)
    {
        ESP_LOGE(m_TAG, "Socket not initialized");
        return -1;
    }

    uint8_t temp_buf[64];
    socklen_t addr_len = sizeof(m_addr);

    int len = recvfrom(m_sock, temp_buf, sizeof(temp_buf), 0, (struct sockaddr*)&m_addr, &addr_len);

    if (len < 0)
    {
        ESP_LOGE(m_TAG, "IP Receive failed: errno %d", errno);
        return -errno;
    }

    const char* ip_str = inet_ntoa(((struct sockaddr_in*)&m_addr)->sin_addr);

    if (!ip_str)
    {
        ESP_LOGE(m_TAG, "inet_ntoa() failed");
        return -1;
    }

    strncpy(buffer, ip_str, 15);

    return len;
}

