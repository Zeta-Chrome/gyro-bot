#pragma once

#include "wifi_station.hpp"
#include "camera.hpp"

constexpr uint8_t MAX_CLIENTS = 2;
constexpr uint8_t FB_QUEUE_SIZE = 3;
constexpr uint8_t JPEG_RING_SIZE = 3;

#if HOTSPOT_MODE
struct FrameHeader
{
    uint32_t frame_id;
    uint32_t ms;
    uint16_t size;
} __attribute__((packed));
#else
struct ChunkHeader
{
    uint32_t frame_id;
    uint8_t total_chunks;
    uint32_t ms;
    uint8_t chunk_idx;
    uint16_t chunk_size;
} __attribute__((packed));
#endif

struct JPEGEntry
{
    uint8_t* data;
    uint32_t size;
    uint32_t ms;
};

class RingBuffer
{
public:
    RingBuffer(uint8_t size);
    ~RingBuffer();

    JPEGEntry& acquire_buffer();
    void push_buffer(JPEGEntry& entry);
    JPEGEntry& pop_buffer();
    void release_buffer(JPEGEntry& entry);

private:
    uint8_t m_size;
    JPEGEntry* m_buffer;
    QueueHandle_t m_fill_queue;
    QueueHandle_t m_free_queue;
};

class VisionContext
{
public:
    VisionContext();
    ~VisionContext();

    WiFiStation station;
    Camera camera;
    TaskHandle_t cam_task;
    TaskHandle_t tx_task;
    QueueHandle_t cam_fb_queue;
    volatile bool capture_in_progress = false;
    RingBuffer jpeg_ring;
    char dest_ip[16];
    SemaphoreHandle_t clients_mutex;
};

