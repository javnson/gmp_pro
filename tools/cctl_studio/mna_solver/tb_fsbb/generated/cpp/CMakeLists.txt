cmake_minimum_required(VERSION 3.21)
project(gmp_fsbbcircuit_testbench LANGUAGES CXX)

find_package(Eigen3 CONFIG REQUIRED)

# This testbench is maintained by hand. Only fsbbcircuit.hpp is generated.
add_executable(fsbbcircuit_testbench fsbbcircuit_testbench.cpp)
target_compile_features(fsbbcircuit_testbench PRIVATE cxx_std_17)
target_link_libraries(fsbbcircuit_testbench PRIVATE Eigen3::Eigen)

enable_testing()
add_test(NAME fsbbcircuit_testbench COMMAND fsbbcircuit_testbench)
