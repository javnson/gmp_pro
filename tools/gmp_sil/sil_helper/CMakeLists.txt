cmake_minimum_required(VERSION 3.21)

project(gmp_sil_helper VERSION 0.2.0 LANGUAGES CXX)

# The repository-pinned compiler can be newer than the MSVC redistributable
# installed beside MATLAB.  Keep STL synchronization state inside GMP SIL
# test/tools binaries; no CRT-owned allocation crosses this library boundary.
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

find_package(asio CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_library(gmp_sil_helper INTERFACE)
target_compile_features(gmp_sil_helper INTERFACE cxx_std_17)
get_filename_component(GMP_REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
target_include_directories(gmp_sil_helper INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} ${GMP_REPOSITORY_ROOT})
target_link_libraries(gmp_sil_helper INTERFACE asio::asio nlohmann_json::nlohmann_json)

option(GMP_SIL_HELPER_BUILD_TESTS "Build GMP SIL TCP/UDP tests" ON)
if(GMP_SIL_HELPER_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
