cmake_minimum_required(VERSION 3.16)
project(cctl_pmsm_average_model_test LANGUAGES CXX)

enable_testing()

add_executable(cctl_pmsm_average_model_test
    pmsm_average_model_test.cpp
)

add_executable(cctl_pmsm_average_model_benchmark
    pmsm_average_model_benchmark.cpp
)

target_compile_features(cctl_pmsm_average_model_test PRIVATE cxx_std_11)
target_compile_features(cctl_pmsm_average_model_benchmark PRIVATE cxx_std_11)
target_include_directories(cctl_pmsm_average_model_test PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../.."
)
target_include_directories(cctl_pmsm_average_model_benchmark PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../.."
)

if(MSVC)
    target_compile_options(cctl_pmsm_average_model_test PRIVATE /W4 /permissive- /utf-8)
    target_compile_options(cctl_pmsm_average_model_benchmark PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(cctl_pmsm_average_model_test PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(cctl_pmsm_average_model_benchmark PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME cctl_pmsm_average_model_test COMMAND cctl_pmsm_average_model_test)
