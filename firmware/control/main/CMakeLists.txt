idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS ""
                       REQUIRES networking imu motor_driver servo ultrasound_sensor
)

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++20)
