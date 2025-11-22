idf_component_register(
    SRCS "src/motor_driver.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_ledc esp_driver_gpio 
)
