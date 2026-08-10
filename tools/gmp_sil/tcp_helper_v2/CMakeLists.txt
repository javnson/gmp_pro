cmake_minimum_required(VERSION 3.21)

project(gmp_sil_tcp_helper_v2 VERSION 0.1.0 LANGUAGES CXX)

find_package(asio CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_library(gmp_sil_tcp_helper_v2 INTERFACE)
target_compile_features(gmp_sil_tcp_helper_v2 INTERFACE cxx_std_17)
target_include_directories(gmp_sil_tcp_helper_v2 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(gmp_sil_tcp_helper_v2 INTERFACE asio::asio nlohmann_json::nlohmann_json)

option(GMP_TCP_HELPER_BUILD_TESTS "Build TCP helper loopback tests" ON)
if(GMP_TCP_HELPER_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

