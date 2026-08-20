cmake_minimum_required(VERSION 3.21)
project(gmp_bucknpccircuit_testbench LANGUAGES CXX)

find_package(Eigen3 CONFIG REQUIRED)

# This testbench is maintained by hand. The circuit header and archive are generated.
configure_file("${CMAKE_CURRENT_LIST_DIR}/../../generated/bucknpccircuit.archive"
               "${CMAKE_CURRENT_BINARY_DIR}/bucknpccircuit.archive" COPYONLY)
add_executable(bucknpccircuit_testbench bucknpccircuit_testbench.cpp)
target_compile_features(bucknpccircuit_testbench PRIVATE cxx_std_17)
target_include_directories(bucknpccircuit_testbench PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../../generated")
target_link_libraries(bucknpccircuit_testbench PRIVATE Eigen3::Eigen)
if(MSVC)
    target_compile_options(bucknpccircuit_testbench PRIVATE /bigobj)
endif()

enable_testing()
add_test(NAME bucknpccircuit_testbench COMMAND bucknpccircuit_testbench)
