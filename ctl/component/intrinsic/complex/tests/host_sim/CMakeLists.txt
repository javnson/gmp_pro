cmake_minimum_required(VERSION 3.20)
project(gmp_dq_controller_host_sim C CXX)

if(NOT DEFINED GMP_PRO_LOCATION)
    message(FATAL_ERROR "Set GMP_PRO_LOCATION to the GMP repository root.")
endif()

add_executable(gmp_dq_controller_host_sim
    dq_controller_host_sim.c
    ${GMP_PRO_LOCATION}/ctl/component/intrinsic/complex/src/ctl_dq_pi.c
    ${GMP_PRO_LOCATION}/ctl/component/intrinsic/complex/src/ctl_dq_ladrc1.c
    ${GMP_PRO_LOCATION}/ctl/component/intrinsic/continuous/src/ctl_continuous_pid.c
    ${GMP_PRO_LOCATION}/ctl/component/intrinsic/continuous/src/ctl_internal_ladrc1.c
)

add_executable(gmp_dq_controller_header_cpp_smoke header_cpp_smoke.cpp)

foreach(target gmp_dq_controller_host_sim gmp_dq_controller_header_cpp_smoke)
    target_include_directories(${target} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/config
        ${GMP_PRO_LOCATION}
    )
endforeach()

if(MSVC)
    target_compile_options(gmp_dq_controller_host_sim PRIVATE /W4)
    target_compile_options(gmp_dq_controller_header_cpp_smoke PRIVATE /W4)
else()
    target_compile_options(gmp_dq_controller_host_sim PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(gmp_dq_controller_header_cpp_smoke PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(gmp_dq_controller_host_sim PRIVATE m)
    target_link_libraries(gmp_dq_controller_header_cpp_smoke PRIVATE m)
endif()

enable_testing()
add_test(NAME dq_controller_host_sim COMMAND gmp_dq_controller_host_sim)
add_test(NAME dq_controller_header_cpp_smoke COMMAND gmp_dq_controller_header_cpp_smoke)
