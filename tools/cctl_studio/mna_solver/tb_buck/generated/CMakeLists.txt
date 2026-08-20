cmake_minimum_required(VERSION 3.21)
project(gmp_generated_buckcircuit_testbench LANGUAGES CXX)

find_package(Eigen3 CONFIG REQUIRED)

add_executable(buckcircuit_testbench buckcircuit_testbench.cpp)
target_compile_features(buckcircuit_testbench PRIVATE cxx_std_17)
target_link_libraries(buckcircuit_testbench PRIVATE Eigen3::Eigen)
