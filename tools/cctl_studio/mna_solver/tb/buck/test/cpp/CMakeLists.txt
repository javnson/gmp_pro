cmake_minimum_required(VERSION 3.21)
project(gmp_buckcircuit_testbench LANGUAGES CXX)

find_package(Eigen3 CONFIG REQUIRED)

# This testbench is maintained by hand. Only buckcircuit.hpp is generated.
add_executable(buckcircuit_testbench buckcircuit_testbench.cpp)
target_compile_features(buckcircuit_testbench PRIVATE cxx_std_17)
target_include_directories(buckcircuit_testbench PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../../generated")
target_link_libraries(buckcircuit_testbench PRIVATE Eigen3::Eigen)

enable_testing()
add_test(NAME buckcircuit_testbench COMMAND buckcircuit_testbench)
