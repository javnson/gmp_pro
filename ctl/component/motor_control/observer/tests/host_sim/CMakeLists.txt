cmake_minimum_required(VERSION 3.20)
project(gmp_acim_observer_host_sim C)

if(NOT DEFINED GMP_PRO_LOCATION)
    message(FATAL_ERROR "Set GMP_PRO_LOCATION to the GMP repository root.")
endif()

add_executable(gmp_acim_observer_host_sim
    acim_observer_host_sim.c
    ${GMP_PRO_LOCATION}/ctl/component/motor_control/consultant/src/ctl_mc_acim_consultant.c
    ${GMP_PRO_LOCATION}/ctl/component/motor_control/consultant/src/ctl_mc_pu_consultant.c
    ${GMP_PRO_LOCATION}/ctl/component/motor_control/observer/src/ctl_mc_acim_pos_calc.c
    ${GMP_PRO_LOCATION}/ctl/component/motor_control/observer/src/ctl_mc_acim_fo.c
    ${GMP_PRO_LOCATION}/ctl/component/motor_control/observer/src/ctl_mc_ato_pll.c
    ${GMP_PRO_LOCATION}/ctl/component/motor_control/observer/src/ctl_mc_pmsm_esmo.c
    ${GMP_PRO_LOCATION}/ctl/component/motor_control/observer/src/ctl_mc_pmsm_fo.c
    ${GMP_PRO_LOCATION}/ctl/component/motor_control/observer/src/ctl_mc_pmsm_hfi.c
    ${GMP_PRO_LOCATION}/ctl/component/motor_control/interface/src/ctl_mc_encoder_switcher.c
    ${GMP_PRO_LOCATION}/ctl/component/intrinsic/continuous/src/ctl_continuous_pid.c
    ${GMP_PRO_LOCATION}/ctl/component/intrinsic/discrete/src/ctl_discrete_filter.c
)

target_include_directories(gmp_acim_observer_host_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/config
    ${GMP_PRO_LOCATION}
)

if(MSVC)
    target_compile_options(gmp_acim_observer_host_sim PRIVATE /W4 /utf-8)
else()
    target_compile_options(gmp_acim_observer_host_sim PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(gmp_acim_observer_host_sim PRIVATE m)
endif()

enable_testing()
add_test(NAME acim_observer_host_sim COMMAND gmp_acim_observer_host_sim)
