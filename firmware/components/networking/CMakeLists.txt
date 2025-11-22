file(GLOB_RECURSE sources "src/*.cpp")

idf_component_register(
    SRCS ${sources}
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_netif lwip nvs_flash esp_event
)
