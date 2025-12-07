#include "camera.hpp"
#include "esp_log.h"
#include "vision_context.hpp"

RingBuffer::RingBuffer(uint8_t size) : m_size(size)
{
    m_buffer = (JPEGEntry*)heap_caps_malloc(MAX_JPEG_SIZE, MALLOC_CAP_SPIRAM);

    for (int i = 0; i < size; i++)
    {
        m_buffer[i].data = (uint8_t*)heap_caps_malloc(MAX_JPEG_SIZE, MALLOC_CAP_SPIRAM);
        if (!m_buffer[i].data)
            ESP_LOGE("RingBuffer", "Failed to allocate JPEG buffer %d", i);

        m_buffer[i].size = 0;
        m_buffer[i].ms = 0;
    } 

    m_free_queue = xQueueCreate(size, sizeof(uint8_t));
    m_fill_queue = xQueueCreate(size, sizeof(uint8_t));

    for (uint8_t i = 0; i < size; i++)
        xQueueSend(m_free_queue, &i, 0);
}

RingBuffer::~RingBuffer()
{
    for (int i = 0; i < m_size; i++)
        free(m_buffer[i].data);

    delete[] m_buffer;

    vQueueDelete(m_free_queue);
    vQueueDelete(m_fill_queue);
}

JPEGEntry& RingBuffer::acquire_buffer()
{
    uint8_t idx;

    if (xQueueReceive(m_free_queue, &idx, 0) == pdTRUE)
        return m_buffer[idx];

    uint8_t oldest_idx;
    xQueueReceive(m_fill_queue, &oldest_idx, portMAX_DELAY);  // should always succeed

    return m_buffer[oldest_idx];
}

// Push buffer after writing JPEG data
void RingBuffer::push_buffer(JPEGEntry& entry)
{
    uint8_t idx = &entry - m_buffer;

    if (xQueueSend(m_fill_queue, &idx, 0) != pdTRUE)
    {
        uint8_t dropped_idx;
        xQueueReceive(m_fill_queue, &dropped_idx, 0);
        xQueueSend(m_free_queue, &dropped_idx, 0);
        xQueueSend(m_fill_queue, &idx, 0);
    }
}

JPEGEntry& RingBuffer::pop_buffer()
{
    uint8_t idx;
    xQueueReceive(m_fill_queue, &idx, portMAX_DELAY);
    return m_buffer[idx];
}

void RingBuffer::release_buffer(JPEGEntry& entry)
{
    uint8_t idx = &entry - m_buffer;
    xQueueSend(m_free_queue, &idx, portMAX_DELAY);
}

VisionContext::VisionContext() : jpeg_ring(JPEG_RING_SIZE)
{
    cam_fb_queue = xQueueCreate(FB_QUEUE_SIZE, sizeof(camera_fb_t*));
    
    clients_mutex = xSemaphoreCreateMutex();
    if (!clients_mutex)
        ESP_LOGE("VisionContext", "Failed to create client mutex!");
}

VisionContext::~VisionContext()
{
    if (clients_mutex)
        vSemaphoreDelete(clients_mutex);

    if (cam_fb_queue)
        vQueueDelete(cam_fb_queue);
}
