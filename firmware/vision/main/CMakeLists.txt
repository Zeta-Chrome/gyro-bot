file(GLOB_RECURSE sources "src/*.cpp")

idf_component_register(SRCS ${sources}
                       INCLUDE_DIRS "../../common" "include"
                       REQUIRES esp_psram esp_driver_gpio esp_timer networking camera)
target_compile_definitions(${COMPONENT_TARGET} PUBLIC TESTING=1)
target_compile_definitions(${COMPONENT_TARGET} PUBLIC HOTSPOT_MODE=1)
