#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>

enum class TCPMode
{
    SENDER,
    RECEIVER
};

class TCPClient
{
public:
    TCPClient(TCPMode mode, const char* host_ip, uint16_t port, size_t buffer_size = 512);
    TCPClient(TCPMode mode, uint16_t listen_port, size_t buffer_size = 512);
    ~TCPClient();

    int start(bool is_blocking);
    int send_data(const uint8_t* payload, size_t length);
    int receive_data(uint8_t* buffer, size_t buffer_len);
    void end();

    bool is_connected() const
    {
        return m_sock >= 0;
    }

    int get_last_error() const
    {
        return m_last_errno;
    }

private:
    const char* m_TAG = "TCP_CLIENT";
    TCPMode m_mode;
    size_t m_buffer_size;
    char m_host_ip[16];
    uint16_t m_port;
    int m_sock;
    int m_client_sock;  // For RECEIVER mode (accepted client connection)
    int m_last_errno;
};
