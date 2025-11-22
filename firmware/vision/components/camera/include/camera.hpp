#pragma once
#include "esp_camera.h"
#include "esp_jpeg_enc.h"
#include <cstdint>

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22
constexpr size_t MAX_JPEG_SIZE = 30000;  // Max JPEG size for QVGA @ quality 80

class Camera
{
public:
    Camera();
    ~Camera();

    bool initialize(pixformat_t format, size_t fb_count, camera_grab_mode_t grab_mode,
                    uint8_t quality = 0);
    camera_fb_t* capture_frame();
    void return_frame(camera_fb_t* fb);

    jpeg_error_t convert_to_jpeg(camera_fb_t* fb, camera_fb_t* outbuf);

private:
    bool m_initialized;
    jpeg_enc_handle_t m_jpeg_enc = NULL;
    size_t m_img_size = 320 * 240 * 2;
};
