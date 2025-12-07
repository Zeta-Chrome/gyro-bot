file(GLOB_RECURSE sources "src/*.cpp")

idf_component_register(SRCS ${sources}
                       INCLUDE_DIRS "../../common" "include"
                       REQUIRES networking imu motor_driver servo ultrasound_sensor)

target_compile_definitions(${COMPONENT_TARGET} PUBLIC TESTING=0)

