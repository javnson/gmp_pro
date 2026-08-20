cmake_minimum_required(VERSION 3.21)
project(gmp_pmsm_drive_testbench LANGUAGES CXX)

option(GMP_MNA_FIXED_USE_AVX "Enable AVX2 kernels in cctl fixed math" OFF)

find_package(Eigen3 CONFIG REQUIRED)

if("$ENV{GMP_PRO_LOCATION}" STREQUAL "")
    message(FATAL_ERROR "GMP_PRO_LOCATION is not defined in the environment. Install or activate GMP first.")
endif()
file(TO_CMAKE_PATH "$ENV{GMP_PRO_LOCATION}" GMP_PRO_ROOT)

configure_file("${CMAKE_CURRENT_LIST_DIR}/../../generated/pmsmcircuit.archive"
               "${CMAKE_CURRENT_BINARY_DIR}/pmsmcircuit.archive" COPYONLY)

add_executable(pmsm_drive_testbench pmsm_drive_testbench.cpp)
target_compile_features(pmsm_drive_testbench PRIVATE cxx_std_17)
target_include_directories(pmsm_drive_testbench PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../generated"
    "${GMP_PRO_ROOT}"
)
target_link_libraries(pmsm_drive_testbench PRIVATE Eigen3::Eigen)
if(GMP_MNA_FIXED_USE_AVX)
    target_compile_definitions(pmsm_drive_testbench PRIVATE CCTL_FIXED_MATH_USE_AVX=1)
    if(MSVC)
        target_compile_options(pmsm_drive_testbench PRIVATE /arch:AVX2)
    else()
        target_compile_options(pmsm_drive_testbench PRIVATE -mavx2)
    endif()
endif()
if(MSVC)
    target_compile_options(pmsm_drive_testbench PRIVATE /bigobj /W4 /permissive- /utf-8)
else()
    target_compile_options(pmsm_drive_testbench PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()
add_test(NAME pmsm_drive_testbench COMMAND pmsm_drive_testbench)
