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
find_package(Python3 COMPONENTS Interpreter REQUIRED)

function(add_cctl_pmsm_simulator target_name backend_name)
    add_executable(${target_name}
        cctl_pmsm_drive.cpp
        ../../src/ctl_main.c)

    set(GMP_CMAKE_TARGET ${target_name})
    include("${GMP_CMAKE_CONFIG}")

    target_link_libraries(${target_name} PRIVATE Threads::Threads)
    target_compile_features(${target_name} PRIVATE cxx_std_17)
    target_compile_definitions(${target_name} PRIVATE
        CCTL_FIXED_MATH_USE_AVX=1
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
    else()
        target_compile_options(${target_name} PRIVATE -mavx2)
    endif()
endfunction()

add_cctl_pmsm_simulator(mcs_pmsm_nt_cctl fixed)
add_cctl_pmsm_simulator(mcs_pmsm_nt_cctl_eigen eigen)
target_link_libraries(mcs_pmsm_nt_cctl_eigen PRIVATE Eigen3::Eigen)

enable_testing()
add_test(NAME mcs_pmsm_nt_cctl_fixed_closed_loop
         COMMAND mcs_pmsm_nt_cctl --no-pause --normal-priority
                 --output mcs_pmsm_nt_cctl_fixed.csv)
add_test(NAME mcs_pmsm_nt_cctl_eigen_closed_loop
         COMMAND mcs_pmsm_nt_cctl_eigen --no-pause --normal-priority
                 --output mcs_pmsm_nt_cctl_eigen.csv)
add_test(NAME mcs_pmsm_nt_cctl_backend_equivalence
         COMMAND ${Python3_EXECUTABLE}
                 ${CMAKE_CURRENT_SOURCE_DIR}/compare_backend_outputs.py
                 ${CMAKE_CURRENT_BINARY_DIR}/mcs_pmsm_nt_cctl_fixed.csv
                 ${CMAKE_CURRENT_BINARY_DIR}/mcs_pmsm_nt_cctl_eigen.csv
                 --tolerance 1e-8)
set_tests_properties(
    mcs_pmsm_nt_cctl_fixed_closed_loop
    mcs_pmsm_nt_cctl_eigen_closed_loop
    PROPERTIES RUN_SERIAL TRUE)
set_tests_properties(mcs_pmsm_nt_cctl_backend_equivalence PROPERTIES
    DEPENDS "mcs_pmsm_nt_cctl_fixed_closed_loop;mcs_pmsm_nt_cctl_eigen_closed_loop"
    RUN_SERIAL TRUE)
