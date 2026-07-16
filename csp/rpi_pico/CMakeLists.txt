cmake_minimum_required(VERSION 3.13)

get_filename_component(GMP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

if(NOT GMP_XPLT_CONFIG_DIR)
    set(GMP_XPLT_CONFIG_DIR "${CMAKE_SOURCE_DIR}")
endif()

add_library(gmp_csp_rpi_pico INTERFACE)

target_sources(gmp_csp_rpi_pico INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/src/rpi_pico_common.c"
    "${CMAKE_CURRENT_LIST_DIR}/src/rpi_pico_peripheral.c"
)

target_include_directories(gmp_csp_rpi_pico INTERFACE
    "${GMP_XPLT_CONFIG_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}"
    "${GMP_ROOT}"
)

target_link_libraries(gmp_csp_rpi_pico INTERFACE
    pico_stdlib
    hardware_gpio
    hardware_i2c
    hardware_spi
    hardware_uart
    hardware_watchdog
)
