cmake_minimum_required(VERSION 3.21)
project(gmp_sinvcircuit_testbench LANGUAGES CXX)

find_package(Eigen3 CONFIG REQUIRED)

# This testbench is maintained by hand. The circuit header and archive are generated.
configure_file("${CMAKE_CURRENT_LIST_DIR}/../../generated/sinvcircuit.archive"
               "${CMAKE_CURRENT_BINARY_DIR}/sinvcircuit.archive" COPYONLY)
add_executable(sinvcircuit_testbench sinvcircuit_testbench.cpp)
target_compile_features(sinvcircuit_testbench PRIVATE cxx_std_17)
target_include_directories(sinvcircuit_testbench PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../../generated")
target_link_libraries(sinvcircuit_testbench PRIVATE Eigen3::Eigen)

enable_testing()
add_test(NAME sinvcircuit_testbench COMMAND sinvcircuit_testbench)
