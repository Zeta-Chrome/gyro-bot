cmake_minimum_required(VERSION 3.22)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(EXTRA_COMPONENT_DIRS "../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(gyro_control)

# Set path for the symlink in the parent directory
set(COMPILE_COMMANDS_SYMLINK "${CMAKE_SOURCE_DIR}/../compile_commands.json")

# Add a custom command to create a symlink after the project is configured
add_custom_target(
    symlink_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
            "${CMAKE_BINARY_DIR}/compile_commands.json"
            "${COMPILE_COMMANDS_SYMLINK}"
    COMMENT "Creating symlink to compile_commands.json in parent directory"
)

