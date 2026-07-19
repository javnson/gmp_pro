cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "PICO_SDK_PATH must point to the Raspberry Pi Pico SDK")
endif()

if(DEFINED PICO_SDK_PATH)
    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
else()
    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
endif()
project(gmp_peripheral_router C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(gmp_peripheral_router
    src/main.c
    ../../protocol/src/gmp_router_protocol.c
)
target_include_directories(gmp_peripheral_router PRIVATE
    include
    ../../protocol/include
)
target_link_libraries(gmp_peripheral_router PRIVATE
    pico_stdlib
    pico_unique_id
    hardware_gpio
    hardware_i2c
    hardware_spi
    hardware_uart
)
pico_enable_stdio_usb(gmp_peripheral_router 1)
pico_enable_stdio_uart(gmp_peripheral_router 0)
pico_add_extra_outputs(gmp_peripheral_router)
