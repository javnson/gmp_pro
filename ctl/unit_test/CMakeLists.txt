cmake_minimum_required(VERSION 3.16)
project(GMP_CTL_UnitTest LANGUAGES C CXX)

include(CTest)
find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

function(add_gmp_ctl_test target numeric_definition)
    add_executable(
        ${target}
        tests/math_and_pid_tests.cpp
        ../component/intrinsic/continuous/src/ctl_continuous_pid.c
    )
    target_compile_features(${target} PRIVATE c_std_11 cxx_std_17)
    target_compile_definitions(${target} PRIVATE GMP_CTL_PORTABLE ${numeric_definition})
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/config ${CMAKE_CURRENT_SOURCE_DIR}/../..)
    target_link_libraries(${target} PRIVATE GTest::gtest)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX /utf-8)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    gtest_discover_tests(${target} DISCOVERY_TIMEOUT 30)
endfunction()

add_gmp_ctl_test(ctl_float_tests GMP_CTL_TEST_FLOAT)
add_gmp_ctl_test(ctl_double_tests GMP_CTL_TEST_DOUBLE)

# Keep the established fixed-point and numeric contract suite in its canonical
# math directory while exposing it through the unified CTL test entry point.
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../math_block/tests
    ${CMAKE_CURRENT_BINARY_DIR}/math_contracts
)
