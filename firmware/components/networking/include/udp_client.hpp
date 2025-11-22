#pragma once

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

enum class UDPMode {
    TRANSMITTER,
    RECEIVER
};

class UDPClient
{
public:
    UDPClient(UDPMode mode, const char* dest_ip, uint16_t port);
    UDPClient(UDPMode mode, uint16_t listen_port, size_t buffer_size = 512);
    ~UDPClient();
    
    int start(bool non_blocking = false);
    void end();
    
    int set_timeout(size_t timeout_ms);
    int send_data(const uint8_t* payload, size_t length);
    int receive_data(uint8_t* buffer, size_t buffer_len);
    UDPMode get_mode() const { return m_mode; }

private:
    UDPMode m_mode;
    char m_ip[16];              // For sender: destination IP, For receiver: ignored
    uint16_t m_port;            // Port number
    size_t m_buffer_size;
    int m_sock;                 // Single socket
    
    struct sockaddr_in m_addr;  // Address structure
    
    static constexpr const char* m_TAG = "UDPClient";
};
