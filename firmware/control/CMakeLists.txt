cmake_minimum_required(VERSION 3.22)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

idf_build_set_property(MINIMAL_BUILD ON)

project(gyro_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
