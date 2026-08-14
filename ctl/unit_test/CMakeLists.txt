cmake_minimum_required(VERSION 3.16)
project(GMP_CTL_UnitTest LANGUAGES C CXX)

include(CTest)
find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

function(configure_gmp_ctl_gtest target)
    target_compile_features(${target} PRIVATE c_std_11 cxx_std_17)
    target_compile_definitions(${target} PRIVATE GMP_CTL_PORTABLE)
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/config ${CMAKE_CURRENT_SOURCE_DIR}/../..)
    target_link_libraries(${target} PRIVATE GTest::gtest_main)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX /utf-8)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    gtest_discover_tests(${target} DISCOVERY_TIMEOUT 30)
endfunction()

function(add_gmp_ctl_math_gtest target numeric_definition)
    add_executable(${target} tests/math_gtests.cpp)
    configure_gmp_ctl_gtest(${target})
    target_compile_definitions(${target} PRIVATE ${numeric_definition})
endfunction()

add_gmp_ctl_math_gtest(ctl_math_float_tests GMP_CTL_TEST_FLOAT)
add_gmp_ctl_math_gtest(ctl_math_double_tests GMP_CTL_TEST_DOUBLE)

add_executable(
    ctl_intrinsic_unit_tests
    tests/intrinsic_gtests.cpp
    ../component/intrinsic/basic/src/ctl_hysteresis_controller.c
    ../component/intrinsic/basic/src/ctl_saturation.c
    ../component/intrinsic/basic/src/ctl_slope_limiter.c
    ../component/intrinsic/continuous/src/ctl_continuous_pi.c
    ../component/intrinsic/continuous/src/ctl_continuous_pid.c
    ../component/intrinsic/discrete/src/ctl_discrete_filter.c
    ../component/intrinsic/discrete/src/ctl_discrete_sg.c
)
configure_gmp_ctl_gtest(ctl_intrinsic_unit_tests)
target_compile_definitions(ctl_intrinsic_unit_tests PRIVATE GMP_CTL_TEST_FLOAT)

add_executable(
    ctl_interface_unit_tests
    tests/interface_gtests.cpp
    ../component/interface/src/ctl_component_interface.c
    ../component/interface/src/ctl_if_pwm_spwm.c
    ../component/intrinsic/discrete/src/ctl_discrete_biquad_filter.c
)
configure_gmp_ctl_gtest(ctl_interface_unit_tests)
target_compile_definitions(ctl_interface_unit_tests PRIVATE GMP_CTL_TEST_FLOAT)

# Keep fixed-point, architecture, lite-algebra, and QR-controller contracts in
# their canonical math directory while exposing them through this test entry point.
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../math_block/tests
    ${CMAKE_CURRENT_BINARY_DIR}/math_contracts
)
