#include "udp_client.hpp"
#include <esp_log.h>

UDPClient::UDPClient(UDPMode mode, const char* dest_ip, uint16_t port)
    : m_mode(mode), m_port(port), m_sock(-1)
{
    if (m_mode != UDPMode::TRANSMITTER)
    {
        ESP_LOGE(m_TAG, "Wrong constructor for mode");
        return;
    }

    strncpy(m_ip, dest_ip, sizeof(m_ip) - 1);
    m_ip[sizeof(m_ip) - 1] = '\0';
}

UDPClient::UDPClient(UDPMode mode, uint16_t listen_port, size_t buffer_size)
    : m_mode(mode), m_port(listen_port), m_buffer_size(buffer_size), m_sock(-1)
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

int UDPClient::start(bool non_blocking)
{
    m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (m_sock < 0)
    {
        ESP_LOGE(m_TAG, "Unable to create socket: errno %d", errno);
        return errno;
    }

    // Configure non-blocking mode if requested
    if (non_blocking)
    {
        int flags = fcntl(m_sock, F_GETFL, 0);
        if (flags < 0)
        {
            ESP_LOGE(m_TAG, "Failed to get socket flags: errno %d", errno);
            close(m_sock);
            m_sock = -1;
            return errno;
        }
        if (fcntl(m_sock, F_SETFL, flags | O_NONBLOCK) < 0)
        {
            ESP_LOGE(m_TAG, "Failed to set non-blocking: errno %d", errno);
            close(m_sock);
            m_sock = -1;
            return errno;
        }
        ESP_LOGI(m_TAG, "Socket set to non-blocking mode");
    }

    if (m_mode == UDPMode::TRANSMITTER)
    {
        // Destination setup
        inet_pton(AF_INET, m_ip, &m_addr.sin_addr);
        m_addr.sin_family = AF_INET;
        m_addr.sin_port = htons(m_port);

        // Enable broadcast
        int enable = 1;
        if (setsockopt(m_sock, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable)) < 0)
        {
            ESP_LOGE(m_TAG, "Failed to enable broadcast: errno %d", errno);
            close(m_sock);
            m_sock = -1;
            return errno;
        }

        ESP_LOGI(m_TAG, "UDP Sender started - dest: %s:%d, sock: %d, buffer: %d", 
                 m_ip, m_port, m_sock, m_buffer_size);
    }
    else
    {
        m_addr.sin_addr.s_addr = INADDR_ANY;
        m_addr.sin_family = AF_INET;
        m_addr.sin_port = htons(m_port);

        if (setsockopt(m_sock, SOL_SOCKET, SO_RCVBUF, &m_buffer_size, sizeof(m_buffer_size)) < 0)
        {
            ESP_LOGW(m_TAG, "Failed to set receive buffer size: errno %d", errno);
        }

        if (bind(m_sock, (struct sockaddr*)&m_addr, sizeof(m_addr)) < 0)
        {
            ESP_LOGE(m_TAG, "Bind failed: errno %d", errno);
            close(m_sock);
            m_sock = -1;
            return errno;
        }

        ESP_LOGI(m_TAG, "UDP Receiver started - port: %d, sock: %d, buffer: %d", 
                 m_port, m_sock, m_buffer_size);
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

    // Send with MSG_DONTWAIT for non-blocking behavior
    int err = sendto(m_sock, payload, length, MSG_DONTWAIT, 
                     (struct sockaddr*)&m_addr, sizeof(m_addr));

    if (err < 0)
    {
        // EWOULDBLOCK/EAGAIN is normal for non-blocking sockets
        if (errno == EWOULDBLOCK || errno == EAGAIN)
        {
            return 0;  // Try again later
        }
        
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

    socklen_t addr_len = sizeof(m_addr);

    int len = recvfrom(m_sock, buffer, buffer_len, 0, 
                       (struct sockaddr*)&m_addr, &addr_len);

    if (len < 0)
    {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
            return 0;

        ESP_LOGE(m_TAG, "Receive failed: errno %d", errno);
        return -errno;
    }

    return len;
}
