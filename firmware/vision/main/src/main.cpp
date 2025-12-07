#include "camera.hpp"
#include "common.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "wifi_station.hpp"
#include "vision_context.hpp"
#include "tcp_client.hpp"
#include "udp_client.hpp"

#define TX_SUCCESS_BIT BIT3

constexpr gpio_num_t RED_LED_PIN = GPIO_NUM_33;
constexpr gpio_num_t INTERRUPT_PIN = GPIO_NUM_2;
constexpr uint8_t JPEG_QUALITY = 45;
constexpr TickType_t JPEG_TASK_DELAY = pdMS_TO_TICKS(15);
constexpr uint16_t DISCOVERY_PORT = 9009;
constexpr uint16_t TX_PORT = 9005;
constexpr size_t CHUNK_PAYLOAD_SIZE = 1024;
constexpr uint8_t STATS_WINDOW_SIZE = 50;
constexpr TickType_t TX_TASK_DELAY = pdMS_TO_TICKS(5);
constexpr uint16_t TCP_RX_PORT = 9002;
constexpr TickType_t TCP_RX_TASK_DELAY = pdMS_TO_TICKS(1000);
constexpr uint8_t FB_QUEUE_SAFE_THRESHOLD = 2;

static void IRAM_ATTR gpio_isr_handler(void* args)
{
    VisionContext* ctx = static_cast<VisionContext*>(args);
    BaseType_t task_woken = pdFALSE;

    if (!ctx->capture_in_progress)
        xTaskNotifyFromISR(ctx->cam_task, 1, eNoAction, &task_woken);

    if (task_woken)
        portYIELD_FROM_ISR();
}

void setup_interrupt(VisionContext* ctx)
{
    gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << INTERRUPT_PIN,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INTERRUPT_PIN, gpio_isr_handler, ctx);
}

void status_blink(VisionContext* ctx, uint16_t high, uint16_t low, uint32_t status_bits)
{
    while (true)
    {
        gpio_set_level(RED_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(high));
        gpio_set_level(RED_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(low));

        EventBits_t now = xEventGroupGetBits(ctx->station.get_wifi_event());
        if (now & status_bits)
            return;
    }
}

void led_wifi_notifier_task(void* args)
{
    VisionContext* ctx = static_cast<VisionContext*>(args);
    if (ctx == NULL)
    {
        ESP_LOGE("LED_TASK", "NULL context!");
        vTaskDelete(NULL);
        return;
    }

    gpio_reset_pin(RED_LED_PIN);
    gpio_set_direction(RED_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RED_LED_PIN, 1);

    EventBits_t bits;
    while (true)
    {
        bits = xEventGroupWaitBits(ctx->station.get_wifi_event(),
                                   WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_RETRY_BIT
                                           | TX_SUCCESS_BIT,
                                   pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & WIFI_CONNECTED_BIT)
            gpio_set_level(RED_LED_PIN, 0);
        else if (bits & WIFI_FAIL_BIT)
            gpio_set_level(RED_LED_PIN, 1);
        else if (bits & WIFI_RETRY_BIT)
            status_blink(ctx, 500, 500, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
        else if (bits & TX_SUCCESS_BIT)
            status_blink(ctx, 900, 100, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_RETRY_BIT);
    }
}

void cam_task(void* args)
{
    const char* TAG = "CAM_TASK";
    VisionContext* ctx = static_cast<VisionContext*>(args);
    if (!ctx)
    {
        ESP_LOGE(TAG, "NULL context!");
        vTaskDelete(NULL);
        return;
    }

    if (!ctx->camera.initialize(PIXFORMAT_YUV422, FB_QUEUE_SIZE, CAMERA_GRAB_LATEST, JPEG_QUALITY))
    {
        ESP_LOGE(TAG, "Camera initialization failed!");
        vTaskDelete(NULL);
        return;
    }

    while (true)
    {
        xTaskNotifyWait(0, 0xFFFFFFFF, NULL, portMAX_DELAY);

        ctx->capture_in_progress = true;
        camera_fb_t* fb = ctx->camera.capture_frame();

        if (!fb)
        {
            ESP_LOGW(TAG, "Failed to capture frame");
            ctx->capture_in_progress = false;

            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (fb->len != FRAME_SIZE)
        {
            ESP_LOGW(TAG, "Corrupted frame size: %u bytes (expected %u), discarding", fb->len,
                     FRAME_SIZE);
            if (fb)
                ctx->camera.return_frame(fb);
            ctx->capture_in_progress = false;
            continue;
        }

        if (ctx->cam_fb_queue)
        {
            if (xQueueSend(ctx->cam_fb_queue, &fb, pdMS_TO_TICKS(10)) != pdTRUE)
            {
                camera_fb_t* old_fb = nullptr;
                if (xQueueReceive(ctx->cam_fb_queue, &old_fb, 0) == pdTRUE)
                {
                    if (old_fb)
                        ctx->camera.return_frame(old_fb);

                    xQueueSend(ctx->cam_fb_queue, &fb, 0);
                }
                else
                {
                    ESP_LOGW(TAG, "Queue operation failed, returning frame");
                    if (fb)
                        ctx->camera.return_frame(fb);
                }
            }
        }
        else
        {
            ESP_LOGE(TAG, "Queue is invalid!");
            if (fb)
                ctx->camera.return_frame(fb);
        }

        ctx->capture_in_progress = false;
    }
}

void jpeg_enc_task(void* args)
{
    const char* TAG = "JPEG_ENC";
    VisionContext* ctx = static_cast<VisionContext*>(args);
    if (!ctx)
    {
        ESP_LOGE(TAG, "NULL context!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "JPEG Encoder task initialized");

    while (true)
    {
        UBaseType_t queue_count = uxQueueMessagesWaiting(ctx->cam_fb_queue);

        if (queue_count > FB_QUEUE_SAFE_THRESHOLD)
        {
            ESP_LOGW(TAG, "Queue backed up (%d frames), draining old frames", queue_count);

            for (UBaseType_t i = 0; i < queue_count - FB_QUEUE_SAFE_THRESHOLD; i++)
            {
                camera_fb_t* dropped_fb = nullptr;
                if (xQueueReceive(ctx->cam_fb_queue, &dropped_fb, 0) == pdTRUE)
                {
                    if (dropped_fb)
                        ctx->camera.return_frame(dropped_fb);
                }
            }
        }

        camera_fb_t* raw_fb = nullptr;
        if (xQueueReceive(ctx->cam_fb_queue, &raw_fb, portMAX_DELAY) != pdTRUE)
        {
            ESP_LOGE(TAG, "Invalid cam_fb_queue");
            vTaskDelete(NULL);
            return;
        }

        if (!raw_fb || !raw_fb->buf || raw_fb->len == 0)
        {
            ESP_LOGE(TAG, "Invalid frame received");
            if (raw_fb)
                ctx->camera.return_frame(raw_fb);
            vTaskDelay(JPEG_TASK_DELAY);
            continue;
        }

        JPEGEntry& buf = ctx->jpeg_ring.acquire_buffer();
        buf.size = 0;

        jpeg_error_t ret = ctx->camera.convert_to_jpeg(raw_fb, buf.data, &buf.size);

        struct timeval timestamp = raw_fb->timestamp;
        buf.ms = timestamp.tv_sec * 1000 + timestamp.tv_usec / 1000;

        ctx->camera.return_frame(raw_fb);

        if (ret != JPEG_ERR_OK)
        {
            ESP_LOGE(TAG, "JPEG encoding failed with error: %d", ret);
            ctx->jpeg_ring.release_buffer(buf);
            vTaskDelay(JPEG_TASK_DELAY);
            continue;
        }

        if (buf.size == 0)
        {
            ESP_LOGE(TAG, "JPEG encoding produced zero bytes");
            ctx->jpeg_ring.release_buffer(buf);
            vTaskDelay(JPEG_TASK_DELAY);
            continue;
        }

        ctx->jpeg_ring.push_buffer(buf);

        vTaskDelay(JPEG_TASK_DELAY);
    }
}

void udp_discover_task(void* args)
{
    const char* TAG = "DISCOVERY";
    VisionContext* ctx = static_cast<VisionContext*>(args);
    if (!ctx)
    {
        ESP_LOGE(TAG, "NULL context!");
        vTaskDelete(NULL);
        return;
    }

    UDPClient udp_rx(UDPMode::RECEIVER, DISCOVERY_PORT);
    if (udp_rx.start() != 0)
    {
        ESP_LOGE(TAG, "UDP init failed!");
        vTaskDelete(NULL);
        return;
    }

    char ip_buf[16];

    while (true)
    {
        int ret = udp_rx.receive_ip(ip_buf);
        if (ret > 0)
        {
            strncpy(ctx->dest_ip, ip_buf, 15);
            ctx->dest_ip[15] = '\0';
            xTaskNotify(ctx->tx_task, 1, eNoAction);

            ESP_LOGI(TAG, "Client discovered: %s", ctx->dest_ip);
            
            vTaskDelete(NULL);
            return;
        }
        else if (ret < 0)
        {
            ESP_LOGE(TAG, "Failed with discovery udp error: %d", ret);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void tcp_tx_task(void* args)
{
    const char* TAG = "TCP_TX";
    VisionContext* ctx = static_cast<VisionContext*>(args);
    if (!ctx)
    {
        ESP_LOGE(TAG, "NULL context!");
        vTaskDelete(NULL);
        return;
    }

    const size_t MAX_PACKET = sizeof(FrameHeader) + MAX_JPEG_SIZE;
    uint8_t* packet_buf = (uint8_t*)malloc(MAX_PACKET);
    if (!packet_buf)
    {
        ESP_LOGE(TAG, "Packet buffer malloc failed!");
        vTaskDelete(NULL);
        return;
    }

    // Wait for client discovery
    xTaskNotifyWait(0, 0xFFFFFFFF, NULL, portMAX_DELAY);

    TCPClient tcp_client(TCPMode::SENDER, ctx->dest_ip, TX_PORT, MAX_JPEG_SIZE);

    uint32_t frame_id = 0;
    size_t frame_count = 0;
    uint64_t total_time_us = 0;
    size_t failed_frames = 0;
    size_t total_bytes_sent = 0;

    while (true)
    {
        ctx->station.wait_for_wifi();
        
        if (tcp_client.start(true) != 0)
        {
            ESP_LOGE(TAG, "Failed to connect to client");
            xEventGroupSetBits(ctx->station.get_wifi_event(), WIFI_CONNECTED_BIT);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        ESP_LOGI(TAG, "Connected to client: %s", ctx->dest_ip);

        while (true)
        {
            uint64_t start_us = esp_timer_get_time();
            JPEGEntry& jpeg_buf = ctx->jpeg_ring.pop_buffer();

            FrameHeader* frame_header = (FrameHeader*)packet_buf;
            frame_header->frame_id = htonl(frame_id);
            frame_header->ms = htonl(jpeg_buf.ms);
            frame_header->size = htons(jpeg_buf.size);

            memcpy(packet_buf + sizeof(FrameHeader), jpeg_buf.data, jpeg_buf.size);

            size_t packet_size = sizeof(FrameHeader) + jpeg_buf.size;
            int result = tcp_client.send_data(packet_buf, packet_size);

            ctx->jpeg_ring.release_buffer(jpeg_buf);

            if (result < 0)
            {
                ESP_LOGW(TAG, "TX failed for frame %u, reconnecting...", frame_id);
                failed_frames++;
                xEventGroupSetBits(ctx->station.get_wifi_event(), WIFI_CONNECTED_BIT);
                break;
            }

            uint64_t end_us = esp_timer_get_time();
            total_time_us += end_us - start_us;
            total_bytes_sent += jpeg_buf.size;
            frame_count++;

            if (frame_count >= STATS_WINDOW_SIZE)
            {
                float avg_ms = (float)total_time_us / 1000.0f / frame_count;
                float fps = 1000000.0f * frame_count / total_time_us;
                float avg_jpeg_kb = (float)total_bytes_sent / 1024.0f / frame_count;
                float success_rate = 100.0f * frame_count / (frame_count + failed_frames);

                ESP_LOGI(TAG, "TX: %.2f ms/frame | %.2f FPS | %.1f KB | Success: %.1f%%",
                         avg_ms, fps, avg_jpeg_kb, success_rate);

                frame_count = 0;
                total_time_us = 0;
                total_bytes_sent = 0;
                failed_frames = 0;
            }

            xEventGroupSetBits(ctx->station.get_wifi_event(), TX_SUCCESS_BIT);
            vTaskDelay(TX_TASK_DELAY);

            frame_id++;
        }

        tcp_client.end();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    free(packet_buf);
}


void tcp_rx_task(void* args)
{
    const char* TAG = "TCP_RX";
    VisionContext* ctx = static_cast<VisionContext*>(args);
    if (!ctx)
    {
        ESP_LOGE(TAG, "NULL context!");
        vTaskDelete(NULL);
        return;
    }

    TCPClient tcp_receiver(TCPMode::RECEIVER, TCP_RX_PORT, 128);
    while (true)
    {
        ESP_LOGI(TAG, "Starting TCP server on port %d...", TCP_RX_PORT);

        ctx->station.wait_for_wifi();
        int err = tcp_receiver.start(true);

        if (err != 0)
        {
            ESP_LOGE(TAG, "Failed to start TCP server: %d", err);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        ESP_LOGI(TAG, "TCP server ready, waiting for client...");

        uint8_t buffer[128] = {0};
        int expected_size = 12 + 32 + 64;

        while (tcp_receiver.is_connected())
        {
            int ret = tcp_receiver.receive_data(buffer, expected_size);

            if (ret == expected_size)
            {
                char ssid[33] = {0};
                char passwd[65] = {0};

                memcpy(ssid, buffer + 12, 32);
                memcpy(passwd, buffer + 12 + 32, 64);

                ctx->station.change_wifi(ssid, passwd);

                tcp_receiver.end();
                break;
            }
            else if (ret < 0)
            {
                ESP_LOGW(TAG, "Receive failed (%d), connection lost", ret);
                break;
            }
            else if (ret > 0)
            {
                ESP_LOGW(TAG, "Unexpected size: %d (expected %d)", ret, expected_size);
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }

        tcp_receiver.end();
        ESP_LOGI(TAG, "Connection closed, restarting server...");
        vTaskDelay(TCP_RX_TASK_DELAY);
    }
}

extern "C" void app_main(void)
{
    initialize_nvs();
    VisionContext* ctx = new VisionContext();

    xTaskCreate(led_wifi_notifier_task, "LED_NOTIFIER", 1024, ctx, 1, NULL);

    ctx->station.wait_for_wifi();
    ctx->station.get_broadcast_ip(ctx->dest_ip, sizeof(ctx->dest_ip));

    xTaskCreate(cam_task, "CAM", 4096, ctx, 2, &ctx->cam_task);
    xTaskCreate(jpeg_enc_task, "JPEG_ENC", 8192, ctx, 3, NULL);
    xTaskCreate(udp_discover_task, "TCP_DISCOVER", 4096, ctx, 2, NULL);
    xTaskCreate(tcp_tx_task, "TCP_TX", 5120, ctx, 2, &ctx->tx_task);
    xTaskCreate(tcp_rx_task, "TCP_RX", 3072, ctx, 1, NULL);

#if TESTING
    while (true)
    {
        xTaskNotify(ctx->cam_task, 1, eNoAction);
        vTaskDelay(pdMS_TO_TICKS(90));
    }
#else
    setup_interrupt(ctx);
#endif
}
