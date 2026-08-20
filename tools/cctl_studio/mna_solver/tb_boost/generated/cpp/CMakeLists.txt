cmake_minimum_required(VERSION 3.21)
project(gmp_generated_boostcircuit_testbench LANGUAGES CXX)

find_package(Eigen3 CONFIG REQUIRED)

add_executable(boostcircuit_testbench boostcircuit_testbench.cpp)
target_compile_features(boostcircuit_testbench PRIVATE cxx_std_17)
target_link_libraries(boostcircuit_testbench PRIVATE Eigen3::Eigen)

enable_testing()
add_test(NAME boostcircuit_testbench COMMAND boostcircuit_testbench)
