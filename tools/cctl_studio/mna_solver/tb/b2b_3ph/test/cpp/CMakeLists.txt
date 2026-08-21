cmake_minimum_required(VERSION 3.21)
project(gmp_b2binvcircuit_testbench LANGUAGES CXX)

find_package(Eigen3 CONFIG REQUIRED)

configure_file("${CMAKE_CURRENT_LIST_DIR}/../../generated/b2binvcircuit.archive"
               "${CMAKE_CURRENT_BINARY_DIR}/b2binvcircuit.archive" COPYONLY)
add_executable(b2binvcircuit_testbench b2binvcircuit_testbench.cpp)
target_compile_features(b2binvcircuit_testbench PRIVATE cxx_std_17)
target_include_directories(b2binvcircuit_testbench PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../generated")
target_link_libraries(b2binvcircuit_testbench PRIVATE Eigen3::Eigen)
if(MSVC)
    target_compile_options(b2binvcircuit_testbench PRIVATE /bigobj)
endif()

enable_testing()
add_test(NAME b2binvcircuit_testbench COMMAND b2binvcircuit_testbench)
