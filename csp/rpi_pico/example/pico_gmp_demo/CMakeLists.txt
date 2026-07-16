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

get_filename_component(GMP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(GMP_PICO_CSP "${GMP_ROOT}/csp/rpi_pico")

add_executable(pico_gmp_demo main.c)

add_subdirectory("${GMP_PICO_CSP}" "${CMAKE_BINARY_DIR}/gmp_csp_rpi_pico")

target_include_directories(pico_gmp_demo PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${GMP_PICO_CSP}"
    "${GMP_ROOT}"
)

target_link_libraries(pico_gmp_demo PRIVATE
    gmp_csp_rpi_pico
)

pico_enable_stdio_usb(pico_gmp_demo 0)
pico_enable_stdio_uart(pico_gmp_demo 0)
pico_add_extra_outputs(pico_gmp_demo)
