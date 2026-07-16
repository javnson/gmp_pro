cmake_minimum_required(VERSION 3.13)

if(NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()

if(NOT PICO_SDK_PATH)
    message(FATAL_ERROR "Set PICO_SDK_PATH to a Raspberry Pi Pico SDK installation.")
endif()

include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")

project(pico_gmp_demo C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_gmp_demo main.c)

include("${CMAKE_CURRENT_LIST_DIR}/gmp_pico_src/gmp_config.cmake")

pico_enable_stdio_usb(pico_gmp_demo 0)
pico_enable_stdio_uart(pico_gmp_demo 0)
pico_add_extra_outputs(pico_gmp_demo)
