cmake_minimum_required(VERSION 3.13)

# Initialize the SDK based on `PICO_SDK_PATH`.
# Note: this must happen before `project()`.
include(cmake/pico_sdk_import.cmake)

project(azbutton-pico)
set(CMAKE_C_STANDARD 11)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Initialize the Raspberry Pi Pico SDK.
pico_sdk_init()

add_subdirectory(srcs)
