cmake_minimum_required(VERSION 3.21)
project(gmp_boostcircuit_testbench LANGUAGES CXX)

find_package(Eigen3 CONFIG REQUIRED)

# This testbench is maintained by hand. Only boostcircuit.hpp is generated.
add_executable(boostcircuit_testbench boostcircuit_testbench.cpp)
target_compile_features(boostcircuit_testbench PRIVATE cxx_std_17)
target_include_directories(boostcircuit_testbench PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../../generated")
target_link_libraries(boostcircuit_testbench PRIVATE Eigen3::Eigen)

enable_testing()
add_test(NAME boostcircuit_testbench COMMAND boostcircuit_testbench)
