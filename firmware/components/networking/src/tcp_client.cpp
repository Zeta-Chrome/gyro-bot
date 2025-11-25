#include "tcp_client.hpp"
#include <cstring>
#include <esp_log.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <sys/param.h>

TCPClient::TCPClient(TCPMode mode, const char* host_ip, uint16_t port, size_t buffer_size) :
    m_mode(mode), m_buffer_size(buffer_size), m_port(port), m_sock(-1), m_client_sock(-1),
    m_last_errno(0)
{
    if (m_mode != TCPMode::SENDER)
    {
        ESP_LOGE(m_TAG, "Wrong constructor for SENDER mode");
        return;
    }

    strncpy(m_host_ip, host_ip, sizeof(m_host_ip) - 1);
    m_host_ip[sizeof(m_host_ip) - 1] = '\0';
}

TCPClient::TCPClient(TCPMode mode, uint16_t listen_port, size_t buffer_size) :
    m_mode(mode), m_buffer_size(buffer_size), m_port(listen_port), m_sock(-1), m_client_sock(-1),
    m_last_errno(0)
{
    if (m_mode != TCPMode::RECEIVER)
    {
        ESP_LOGE(m_TAG, "Wrong constructor for RECEIVER mode");
        return;
    }
}

TCPClient::~TCPClient()
{
    end();
}

int TCPClient::start(bool is_blocking)
{
    if (m_mode == TCPMode::SENDER)
    {
        struct sockaddr_in dest_addr;
        inet_pton(AF_INET, m_host_ip, &dest_addr.sin_addr);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(m_port);
        m_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (m_sock < 0)
        {
            m_last_errno = errno;
            ESP_LOGE(m_TAG, "Unable to create socket: errno %d", errno);
            return errno;
        }

        // Set socket options
        int opt = 1;
        setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // Set blocking/non-blocking mode
        int flags = fcntl(m_sock, F_GETFL, 0);
        if (flags < 0)
        {
            m_last_errno = errno;
            ESP_LOGE(m_TAG, "Failed to get socket flags: errno %d", errno);
            close(m_sock);
            m_sock = -1;
            return errno;
        }

        if (!is_blocking)
        {
            if (fcntl(m_sock, F_SETFL, flags | O_NONBLOCK) < 0)
            {
                m_last_errno = errno;
                ESP_LOGE(m_TAG, "Failed to set non-blocking: errno %d", errno);
                close(m_sock);
                m_sock = -1;
                return errno;
            }
        }
        else
        {
            if (fcntl(m_sock, F_SETFL, flags & ~O_NONBLOCK) < 0)
            {
                m_last_errno = errno;
                ESP_LOGE(m_TAG, "Failed to set blocking: errno %d", errno);
                close(m_sock);
                m_sock = -1;
                return errno;
            }
        }

        ESP_LOGI(m_TAG, "Connecting to %s:%d... (mode: %s)", m_host_ip, m_port,
                 is_blocking ? "blocking" : "non-blocking");
        int err = connect(m_sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        if (err != 0 && errno != EINPROGRESS)
        {
            m_last_errno = errno;
            ESP_LOGE(m_TAG, "Connect failed: errno %d (%s)", errno, strerror(errno));
            close(m_sock);
            m_sock = -1;
            return errno;
        }
        ESP_LOGI(m_TAG, "TCP Sender connected - dest: %s:%d, sock: %d", m_host_ip, m_port, m_sock);
    }
    else  // RECEIVER MODE (SERVER)
    {
        struct sockaddr_in bind_addr;
        bind_addr.sin_addr.s_addr = INADDR_ANY;
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_port = htons(m_port);
        m_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (m_sock < 0)
        {
            m_last_errno = errno;
            ESP_LOGE(m_TAG, "Unable to create socket: errno %d", errno);
            return errno;
        }

        // Set socket options
        int opt = 1;
        setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // Set blocking/non-blocking mode for listening socket
        int flags = fcntl(m_sock, F_GETFL, 0);
        if (flags < 0)
        {
            m_last_errno = errno;
            ESP_LOGE(m_TAG, "Failed to get socket flags: errno %d", errno);
            close(m_sock);
            m_sock = -1;
            return errno;
        }

        if (!is_blocking)
        {
            if (fcntl(m_sock, F_SETFL, flags | O_NONBLOCK) < 0)
            {
                m_last_errno = errno;
                ESP_LOGE(m_TAG, "Failed to set non-blocking: errno %d", errno);
                close(m_sock);
                m_sock = -1;
                return errno;
            }
        }
        else
        {
            if (fcntl(m_sock, F_SETFL, flags & ~O_NONBLOCK) < 0)
            {
                m_last_errno = errno;
                ESP_LOGE(m_TAG, "Failed to set blocking: errno %d", errno);
                close(m_sock);
                m_sock = -1;
                return errno;
            }
        }

        // Bind to port
        if (bind(m_sock, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0)
        {
            m_last_errno = errno;
            ESP_LOGE(m_TAG, "Bind failed: errno %d (%s)", errno, strerror(errno));
            close(m_sock);
            m_sock = -1;
            return errno;
        }

        // Listen for connections
        if (listen(m_sock, 1) < 0)
        {
            m_last_errno = errno;
            ESP_LOGE(m_TAG, "Listen failed: errno %d (%s)", errno, strerror(errno));
            close(m_sock);
            m_sock = -1;
            return errno;
        }

        ESP_LOGI(m_TAG, "TCP Receiver listening on port %d, sock: %d (mode: %s)", m_port, m_sock,
                 is_blocking ? "blocking" : "non-blocking");

        // Accept incoming connection (blocking or non-blocking)
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        ESP_LOGI(m_TAG, "Waiting for client connection...");
        m_client_sock = accept(m_sock, (struct sockaddr*)&client_addr, &addr_len);
        if (m_client_sock < 0)
        {
            if (!is_blocking && (errno == EAGAIN || errno == EWOULDBLOCK))
            {
                // Non-blocking mode and no connection available yet
                ESP_LOGI(m_TAG, "No client connection available yet (non-blocking)");
                return EAGAIN;
            }

            m_last_errno = errno;
            ESP_LOGE(m_TAG, "Accept failed: errno %d (%s)", errno, strerror(errno));
            close(m_sock);
            m_sock = -1;
            return errno;
        }

        // Set the client socket to the same blocking mode
        flags = fcntl(m_client_sock, F_GETFL, 0);
        if (flags >= 0)
        {
            if (!is_blocking)
                fcntl(m_client_sock, F_SETFL, flags | O_NONBLOCK);
            else
                fcntl(m_client_sock, F_SETFL, flags & ~O_NONBLOCK);
        }

        char addr_str[16];
        inet_ntop(AF_INET, &client_addr.sin_addr, addr_str, sizeof(addr_str));
        ESP_LOGI(m_TAG, "Client connected from %s:%d", addr_str, ntohs(client_addr.sin_port));
    }
    return 0;
}

int TCPClient::send_data(const uint8_t* payload, size_t length)
{
    if (m_mode != TCPMode::SENDER)
    {
        ESP_LOGE(m_TAG, "Cannot send - not in SENDER mode");
        return -1;
    }

    if (!payload || length == 0)
    {
        ESP_LOGE(m_TAG, "Invalid payload or length");
        return 0;
    }

    if (m_sock < 0)
    {
        ESP_LOGE(m_TAG, "Socket not connected");
        return -1;
    }

    int err = send(m_sock, payload, length, 0);

    if (err < 0)
    {
        m_last_errno = errno;

        // Check if error is fatal
        if (errno == ECONNRESET || errno == ENOTCONN || errno == EPIPE || errno == ETIMEDOUT
            || errno == ECONNABORTED)
        {
            ESP_LOGE(m_TAG, "Fatal send error: errno %d (%s)", errno, strerror(errno));
            close(m_sock);
            m_sock = -1;
        }
        else
        {
            ESP_LOGE(m_TAG, "Send error: errno %d (%s)", errno, strerror(errno));
        }

        return -errno;
    }

    return err;  // Number of bytes sent
}

int TCPClient::receive_data(uint8_t* buffer, size_t buffer_len)
{
    if (!buffer || buffer_len == 0)
    {
        ESP_LOGE(m_TAG, "Invalid buffer or length");
        return -1;
    }

    int target_sock = (m_mode == TCPMode::RECEIVER) ? m_client_sock : m_sock;

    if (target_sock < 0)
    {
        ESP_LOGE(m_TAG, "Socket not connected");
        return -1;
    }

    int len = recv(target_sock, buffer, buffer_len, 0);

    if (len < 0)
    {
        m_last_errno = errno;

        // EAGAIN/EWOULDBLOCK is not fatal (just timeout)
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return 0;  // No data available
        }

        // EINTR is not fatal (interrupted by signal)
        if (errno == EINTR)
        {
            return 0;  // Just retry
        }

        // Fatal error - connection is broken
        ESP_LOGE(m_TAG, "Fatal recv error: errno %d (%s)", errno, strerror(errno));

        if (m_mode == TCPMode::RECEIVER)
        {
            close(m_client_sock);
            m_client_sock = -1;
        }
        else
        {
            close(m_sock);
            m_sock = -1;
        }

        return -errno;
    }
    else if (len == 0)
    {
        // Connection closed by peer
        ESP_LOGW(m_TAG, "Connection closed by peer");

        if (m_mode == TCPMode::RECEIVER)
        {
            close(m_client_sock);
            m_client_sock = -1;
        }
        else
        {
            close(m_sock);
            m_sock = -1;
        }

        return 0;  // Connection closed gracefully
    }

    return len;  // Bytes received
}

void TCPClient::end()
{
    if (m_client_sock >= 0)
    {
        shutdown(m_client_sock, SHUT_RDWR);
        close(m_client_sock);
        m_client_sock = -1;
    }

    if (m_sock >= 0)
    {
        shutdown(m_sock, SHUT_RDWR);
        close(m_sock);
        m_sock = -1;
        ESP_LOGI(m_TAG, "Socket closed");
    }
}
