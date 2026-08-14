cmake_minimum_required(VERSION 3.16)
project(GMP_Linux_CSP_Compile_Test LANGUAGES C CXX)

find_package(asio CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(Threads REQUIRED)

get_filename_component(GMP_REPOSITORY_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

add_library(
    gmp_linux_csp_compile_contract
    OBJECT
    ../src/linux_can_driver.cpp
    ../src/linux_peripheral_driver.cpp
    ../src/linux_rt_tracer.cpp
    ../src/linux_simulink_main.cpp
)
target_compile_features(gmp_linux_csp_compile_contract PRIVATE cxx_std_17)
target_include_directories(
    gmp_linux_csp_compile_contract
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/.."
    "${GMP_REPOSITORY_ROOT}"
)
target_link_libraries(
    gmp_linux_csp_compile_contract
    PRIVATE
    asio::asio
    nlohmann_json::nlohmann_json
    Threads::Threads
)

if(MSVC)
    target_compile_options(gmp_linux_csp_compile_contract PRIVATE /W4 /WX /utf-8)
else()
    target_compile_options(gmp_linux_csp_compile_contract PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
