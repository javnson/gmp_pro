cmake_minimum_required(VERSION 3.20)
project(gmp_inv_gfm_host_sim C)

if(NOT DEFINED GMP_PRO_LOCATION)
    message(FATAL_ERROR "Set GMP_PRO_LOCATION to the GMP repository root.")
endif()

add_executable(gmp_inv_gfm_host_sim
    inv_gfm_host_sim.c
    ${GMP_PRO_LOCATION}/ctl/component/digital_power/inv/src/ctl_dp_gfm_droop_ctrl.c
    ${GMP_PRO_LOCATION}/ctl/component/digital_power/inv/src/ctl_dp_gfm_vsm_ctrl.c
    ${GMP_PRO_LOCATION}/ctl/component/digital_power/inv/src/ctl_dp_gfm_virtual_impedance.c
    ${GMP_PRO_LOCATION}/ctl/component/digital_power/inv/src/ctl_dp_gfl_pq_droop_ctrl.c
    ${GMP_PRO_LOCATION}/ctl/component/digital_power/inv/src/ctl_dp_gfm_transition.c
    ${GMP_PRO_LOCATION}/ctl/component/intrinsic/discrete/src/ctl_discrete_filter.c
)

target_include_directories(gmp_inv_gfm_host_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/config
    ${GMP_PRO_LOCATION}
)

if(MSVC)
    target_compile_options(gmp_inv_gfm_host_sim PRIVATE /W4)
else()
    target_compile_options(gmp_inv_gfm_host_sim PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(gmp_inv_gfm_host_sim PRIVATE m)
endif()

enable_testing()
add_test(NAME inv_gfm_host_sim COMMAND gmp_inv_gfm_host_sim)
