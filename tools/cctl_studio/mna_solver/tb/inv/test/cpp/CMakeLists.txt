cmake_minimum_required(VERSION 3.21)
project(gmp_invcircuit_testbench LANGUAGES CXX)

find_package(Eigen3 CONFIG REQUIRED)

# This testbench is maintained by hand. The circuit header and archive are generated.
configure_file("${CMAKE_CURRENT_LIST_DIR}/../../generated/invcircuit.archive"
               "${CMAKE_CURRENT_BINARY_DIR}/invcircuit.archive" COPYONLY)
add_executable(invcircuit_testbench invcircuit_testbench.cpp)
target_compile_features(invcircuit_testbench PRIVATE cxx_std_17)
target_include_directories(invcircuit_testbench PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../../generated")
target_link_libraries(invcircuit_testbench PRIVATE Eigen3::Eigen)
if(MSVC)
    target_compile_options(invcircuit_testbench PRIVATE /bigobj)
endif()

enable_testing()
add_test(NAME invcircuit_testbench COMMAND invcircuit_testbench)
