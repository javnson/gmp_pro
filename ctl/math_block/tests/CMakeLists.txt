cmake_minimum_required(VERSION 3.24)

project(gmp_numeric_contract_tests LANGUAGES C CXX)

enable_testing()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(GMP_REPOSITORY_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(GMP_TEST_INCLUDE_DIRS
    "${GMP_REPOSITORY_ROOT}"
    "${CMAKE_CURRENT_LIST_DIR}"
)

add_library(gmp_arch_compile_contract OBJECT
    "${GMP_REPOSITORY_ROOT}/core/std/tests/arch_compile_smoke.c"
)
target_include_directories(gmp_arch_compile_contract PRIVATE
    "${GMP_REPOSITORY_ROOT}/core/std/tests"
    "${GMP_REPOSITORY_ROOT}"
)

function(add_gmp_numeric_contract target ctrl_backend parameter_backend)
    add_executable(${target} numeric_type_contract_test.c)
    target_include_directories(${target} PRIVATE ${GMP_TEST_INCLUDE_DIRS})
    target_compile_definitions(${target} PRIVATE
        GMP_CTL_PORTABLE
        SPECIFY_CTRL_GT_TYPE=${ctrl_backend}
        SPECIFY_PARAMETER_GT_TYPE=${parameter_backend}
    )
    if(NOT MSVC)
        target_link_libraries(${target} PRIVATE m)
    endif()
    add_test(NAME ${target} COMMAND ${target})
endfunction()

add_gmp_numeric_contract(gmp_numeric_float_float USING_FLOAT_FPU USING_FLOAT_FPU)
add_gmp_numeric_contract(gmp_numeric_float_double USING_FLOAT_FPU USING_DOUBLE_FPU)
add_gmp_numeric_contract(gmp_numeric_double_double USING_DOUBLE_FPU USING_DOUBLE_FPU)

file(GLOB GMP_IQMATH_SOURCES CONFIGURE_DEPENDS
    "${GMP_REPOSITORY_ROOT}/third_party/iqmath/src/*.c"
)
foreach(target IN ITEMS gmp_numeric_iq24_float gmp_numeric_iq24_double)
    if(target STREQUAL "gmp_numeric_iq24_float")
        set(parameter_backend USING_FLOAT_FPU)
    else()
        set(parameter_backend USING_DOUBLE_FPU)
    endif()
    add_executable(${target} numeric_type_contract_test.c ${GMP_IQMATH_SOURCES})
    target_include_directories(${target} PRIVATE
        ${GMP_TEST_INCLUDE_DIRS}
        "${GMP_REPOSITORY_ROOT}/third_party/iqmath"
        "${GMP_REPOSITORY_ROOT}/third_party/iqmath/src"
    )
    target_compile_definitions(${target} PRIVATE
        GMP_CTL_PORTABLE
        SPECIFY_CTRL_GT_TYPE=USING_FIXED_TI_IQ_LIBRARY
        SPECIFY_PARAMETER_GT_TYPE=${parameter_backend}
        GLOBAL_IQ=24
    )
    if(NOT MSVC)
        target_link_libraries(${target} PRIVATE m)
    endif()
    add_test(NAME ${target} COMMAND ${target})
endforeach()

add_executable(gmp_lite_algebra lite_algebra_test.cpp)
target_include_directories(gmp_lite_algebra PRIVATE "${GMP_REPOSITORY_ROOT}")
add_test(NAME gmp_lite_algebra COMMAND gmp_lite_algebra)

add_executable(gmp_sinv_qr_contract
    "${GMP_REPOSITORY_ROOT}/ctl/component/digital_power/sinv/tests/sinv_qr_contract_test.c"
    "${GMP_REPOSITORY_ROOT}/ctl/component/intrinsic/discrete/src/ctl_discrete_pr.c"
)
target_include_directories(gmp_sinv_qr_contract PRIVATE ${GMP_TEST_INCLUDE_DIRS})
target_compile_definitions(gmp_sinv_qr_contract PRIVATE GMP_CTL_PORTABLE)
if(NOT MSVC)
    target_link_libraries(gmp_sinv_qr_contract PRIVATE m)
endif()
add_test(NAME gmp_sinv_qr_contract COMMAND gmp_sinv_qr_contract)
