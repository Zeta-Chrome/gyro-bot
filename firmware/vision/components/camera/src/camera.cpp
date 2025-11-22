#include "camera.hpp"
#include "esp_camera.h"
#include "esp_jpeg_common.h"
#include "esp_log.h"
#include "sensor.h"
#include "esp_jpeg_enc.h"
#include <driver/i2c.h>

Camera::Camera() : m_initialized(false) {}

Camera::~Camera()
{
    if (m_initialized)
        esp_camera_deinit();

    if (m_jpeg_enc)
        jpeg_enc_close(m_jpeg_enc);
}

bool Camera::initialize(pixformat_t format, size_t fb_count, camera_grab_mode_t grab_mode,
                        uint8_t quality)
{
    if (m_initialized)
        return true;

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAM_PIN_D0;
    config.pin_d1 = CAM_PIN_D1;
    config.pin_d2 = CAM_PIN_D2;
    config.pin_d3 = CAM_PIN_D3;
    config.pin_d4 = CAM_PIN_D4;
    config.pin_d5 = CAM_PIN_D5;
    config.pin_d6 = CAM_PIN_D6;
    config.pin_d7 = CAM_PIN_D7;
    config.pin_vsync = CAM_PIN_VSYNC;
    config.pin_href = CAM_PIN_HREF;
    config.pin_pclk = CAM_PIN_PCLK;
    config.pin_xclk = CAM_PIN_XCLK;
    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_pwdn = CAM_PIN_PWDN;
    config.pin_reset = CAM_PIN_RESET;
    config.xclk_freq_hz = 10000000;
    config.grab_mode = grab_mode;
    config.pixel_format = format;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 0;
    config.fb_count = fb_count;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = grab_mode;

    if (esp_camera_init(&config) != ESP_OK)
        return false;

    m_initialized = true;
    ESP_LOGI("CAMERA", "Camera initialized successfully with %zu buffers", fb_count);

    if (quality > 0)
    {
        jpeg_enc_config_t jpeg_enc_cfg;

        // For QVGA resolution
        jpeg_enc_cfg.width = 320;
        jpeg_enc_cfg.height = 240;
        jpeg_enc_cfg.quality = quality;
        jpeg_enc_cfg.task_enable = true;
        jpeg_enc_cfg.hfm_task_priority = 5;
        jpeg_enc_cfg.hfm_task_core = 1;

        // Set pixel format based on camera output
        if (format == PIXFORMAT_YUV422)
        {
            jpeg_enc_cfg.src_type = JPEG_PIXEL_FORMAT_YCbYCr;  // OV7670 outputs UYVY
            jpeg_enc_cfg.subsampling = JPEG_SUBSAMPLE_422;
        }
        else if (format == PIXFORMAT_RGB565)
        {
            jpeg_enc_cfg.src_type = JPEG_PIXEL_FORMAT_RGB565_BE;  // OV7670 default big-endian
        }
        else
        {
            ESP_LOGE("CAMERA", "Unsupported pixel format for JPEG encoding");
            return false;
        }

        jpeg_error_t ret = JPEG_ERR_OK;
        ret = jpeg_enc_open(&jpeg_enc_cfg, &m_jpeg_enc);
        if (ret != JPEG_ERR_OK)
        {
            ESP_LOGE("CAMERA", "Failed to open jpeg encoder");
            return false;
        }

        ESP_LOGI("CAMERA", "JPEG encoder initialized with quality %d", quality);
    }

    return true;
}

camera_fb_t* Camera::capture_frame()
{
    return esp_camera_fb_get();
}

void Camera::return_frame(camera_fb_t* fb)
{
    if (fb)
        esp_camera_fb_return(fb);
}

jpeg_error_t Camera::convert_to_jpeg(camera_fb_t* fb, camera_fb_t* outbuf)
{
    jpeg_error_t ret = JPEG_ERR_OK;
    int len;
    ret = jpeg_enc_process(m_jpeg_enc, fb->buf, (int)fb->len, outbuf->buf, MAX_JPEG_SIZE, &len);
    if (ret != JPEG_ERR_OK) 
        return ret;

    outbuf->len = len;
    return JPEG_ERR_OK;
}
