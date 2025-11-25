idf_component_register(
    SRCS "src/camera.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_psram esp32-camera esp_new_jpeg 
)
