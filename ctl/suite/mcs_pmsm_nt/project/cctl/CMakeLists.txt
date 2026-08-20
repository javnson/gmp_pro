cmake_minimum_required(VERSION 3.21)
project(mcs_pmsm_nt_cctl LANGUAGES C CXX)

if("$ENV{GMP_PRO_LOCATION}" STREQUAL "")
    message(FATAL_ERROR "GMP_PRO_LOCATION is not defined. Run a GMP installer first.")
endif()
file(TO_CMAKE_PATH "$ENV{GMP_PRO_LOCATION}" GMP_PRO_ROOT)

set(GMP_CMAKE_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/gmp_src_mgr/gmp_config.cmake")
if(NOT EXISTS "${GMP_CMAKE_CONFIG}")
    message(FATAL_ERROR
        "gmp_src_mgr/gmp_config.cmake is missing. Run build_test.bat or the "
        "project source-manager generation flow before configuring.")
endif()

find_package(Threads REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

option(CCTL_BUILD_FIXED_BACKEND
       "Build the optional fixed-matrix main-circuit backend" OFF)

function(add_cctl_pmsm_simulator target_name backend_name)
    add_executable(${target_name}
        cctl_pmsm_drive.cpp
        ../../src/ctl_main.c)

    set(GMP_CMAKE_TARGET ${target_name})
    include("${GMP_CMAKE_CONFIG}")

    target_link_libraries(${target_name} PRIVATE Threads::Threads)
    target_compile_features(${target_name} PRIVATE cxx_std_17)
    target_compile_definitions(${target_name} PRIVATE
        CCTL_SIM_MATRIX_BACKEND_NAME="${backend_name}")
    target_include_directories(${target_name} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/sdpe_mgr"
        "${CMAKE_CURRENT_SOURCE_DIR}/xplt"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../src"
        "${GMP_PRO_ROOT}"
        "${CMAKE_CURRENT_SOURCE_DIR}/hw/generated/${backend_name}")

    if(MSVC)
        target_compile_options(${target_name} PRIVATE /utf-8 /bigobj /arch:AVX2 /wd4828)
        if(backend_name STREQUAL "fixed")
            # Large generated fixed pools are constant-initialized so they do
            # not create multi-megabyte startup temporaries on the thread stack.
            target_compile_definitions(${target_name} PRIVATE
                CCTL_FIXED_MATH_USE_AVX=1)
            target_compile_options(${target_name} PRIVATE /constexpr:steps16777216)
        endif()
    else()
        if(backend_name STREQUAL "fixed")
            target_compile_definitions(${target_name} PRIVATE
                CCTL_FIXED_MATH_USE_AVX=1)
            target_compile_options(${target_name} PRIVATE -mavx2)
        endif()
    endif()
endfunction()

add_cctl_pmsm_simulator(mcs_pmsm_nt_cctl eigen)
target_link_libraries(mcs_pmsm_nt_cctl PRIVATE Eigen3::Eigen)

enable_testing()
add_test(NAME mcs_pmsm_nt_cctl_closed_loop
         COMMAND mcs_pmsm_nt_cctl --no-pause --normal-priority
                 --output mcs_pmsm_nt_cctl.csv)

if(CCTL_BUILD_FIXED_BACKEND)
    add_cctl_pmsm_simulator(mcs_pmsm_nt_cctl_fixed fixed)
    add_test(NAME mcs_pmsm_nt_cctl_fixed_closed_loop
             COMMAND mcs_pmsm_nt_cctl_fixed --no-pause --normal-priority
                     --output mcs_pmsm_nt_cctl_fixed.csv)
    set_tests_properties(
        mcs_pmsm_nt_cctl_closed_loop
        mcs_pmsm_nt_cctl_fixed_closed_loop
        PROPERTIES RUN_SERIAL TRUE)
endif()
