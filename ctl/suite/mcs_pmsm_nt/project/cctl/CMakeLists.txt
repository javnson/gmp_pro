cmake_minimum_required(VERSION 3.21)
project(mcs_pmsm_nt_cctl LANGUAGES C CXX)

if("$ENV{GMP_PRO_LOCATION}" STREQUAL "")
    message(FATAL_ERROR "GMP_PRO_LOCATION is not defined. Run a GMP installer first.")
endif()
file(TO_CMAKE_PATH "$ENV{GMP_PRO_LOCATION}" GMP_PRO_ROOT)

add_executable(mcs_pmsm_nt_cctl
    cctl_pmsm_drive.cpp
    ../../src/ctl_main.c)

set(GMP_CMAKE_TARGET mcs_pmsm_nt_cctl)
set(GMP_CMAKE_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/gmp_src_mgr/gmp_config.cmake")
if(NOT EXISTS "${GMP_CMAKE_CONFIG}")
    message(FATAL_ERROR
        "gmp_src_mgr/gmp_config.cmake is missing. Run build_test.bat or the "
        "project source-manager generation flow before configuring.")
endif()
include("${GMP_CMAKE_CONFIG}")

find_package(Threads REQUIRED)
target_link_libraries(mcs_pmsm_nt_cctl PRIVATE Threads::Threads)

target_compile_features(mcs_pmsm_nt_cctl PRIVATE cxx_std_17)
target_compile_definitions(mcs_pmsm_nt_cctl PRIVATE CCTL_FIXED_MATH_USE_AVX=1)
target_include_directories(mcs_pmsm_nt_cctl PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/sdpe_mgr"
    "${CMAKE_CURRENT_SOURCE_DIR}/xplt"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../src"
    "${GMP_PRO_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/hw/generated")

if(MSVC)
    target_compile_options(mcs_pmsm_nt_cctl PRIVATE /utf-8 /bigobj /arch:AVX2 /wd4828)
else()
    target_compile_options(mcs_pmsm_nt_cctl PRIVATE -mavx2)
endif()

enable_testing()
add_test(NAME mcs_pmsm_nt_cctl_closed_loop
         COMMAND mcs_pmsm_nt_cctl --no-pause)
