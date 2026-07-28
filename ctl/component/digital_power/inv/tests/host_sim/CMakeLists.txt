cmake_minimum_required(VERSION 3.20)
project(gmp_inv_host_sim C CXX)

if(NOT DEFINED GMP_PRO_LOCATION)
    message(FATAL_ERROR "Set GMP_PRO_LOCATION to the GMP repository root.")
endif()

add_executable(gmp_inv_host_sim
    inv_host_sim.c
    ${GMP_PRO_LOCATION}/ctl/component/digital_power/inv/src/ctl_dp_voltage_ctrl.c
    ${GMP_PRO_LOCATION}/ctl/component/digital_power/inv/src/ctl_dp_zero_ctrl.c
    ${GMP_PRO_LOCATION}/ctl/component/intrinsic/continuous/src/ctl_continuous_pid_aw.c
    ${GMP_PRO_LOCATION}/ctl/component/intrinsic/discrete/src/ctl_discrete_pr.c
    ${GMP_PRO_LOCATION}/ctl/component/intrinsic/discrete/src/ctl_discrete_pr_tuner.c
)

add_executable(gmp_inv_header_cpp_smoke header_cpp_smoke.cpp)

foreach(target gmp_inv_host_sim gmp_inv_header_cpp_smoke)
    target_include_directories(${target} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/config
        ${GMP_PRO_LOCATION}
    )
endforeach()

if(MSVC)
    target_compile_options(gmp_inv_host_sim PRIVATE /W4)
    target_compile_options(gmp_inv_header_cpp_smoke PRIVATE /W4)
else()
    target_compile_options(gmp_inv_host_sim PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(gmp_inv_header_cpp_smoke PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(gmp_inv_host_sim PRIVATE m)
    target_link_libraries(gmp_inv_header_cpp_smoke PRIVATE m)
endif()
