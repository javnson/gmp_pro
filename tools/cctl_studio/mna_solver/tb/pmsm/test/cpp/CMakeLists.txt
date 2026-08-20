cmake_minimum_required(VERSION 3.21)
project(gmp_pmsm_drive_testbench LANGUAGES CXX)

find_package(Eigen3 CONFIG REQUIRED)

if(NOT GMP_PRO_LOCATION)
    message(FATAL_ERROR "GMP_PRO_LOCATION must point to the GMP repository")
endif()

add_executable(pmsm_drive_testbench pmsm_drive_testbench.cpp)
target_compile_features(pmsm_drive_testbench PRIVATE cxx_std_17)
target_include_directories(pmsm_drive_testbench PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../generated"
    "${GMP_PRO_LOCATION}"
)
target_link_libraries(pmsm_drive_testbench PRIVATE Eigen3::Eigen)
if(MSVC)
    target_compile_options(pmsm_drive_testbench PRIVATE /bigobj /W4 /permissive- /utf-8)
else()
    target_compile_options(pmsm_drive_testbench PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()
add_test(NAME pmsm_drive_testbench COMMAND pmsm_drive_testbench)
