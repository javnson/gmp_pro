/**
 * @file ctl_dp_single_phase.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation for single-phase digital power controller modules.
 * @version 1.0
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @brief Initialization functions for single-phase digital power components,
 * including the Single-Phase PLL, H-Bridge Modulation, and PFC controllers.
 */

#include <gmp_core.h>
#include <math.h>



//////////////////////////////////////////////////////////////////////////
// Single Phase PFC Control
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/single_phase/spfc.h>

void ctl_init_spfc_ctrl(spfc_t* pfc, parameter_gt voltage_kp, parameter_gt voltage_Ti, parameter_gt voltage_Td,
                        parameter_gt current_kp, parameter_gt current_Ti, parameter_gt current_Td, parameter_gt fs)
{
    // Initialize the series-form PID for the outer voltage control loop.
    ctl_init_pid_Tmode(&pfc->voltage_ctrl, voltage_kp, voltage_Ti, voltage_Td, fs);
    // Initialize the series-form PID for the inner current control loop.
    ctl_init_pid_Tmode(&pfc->current_ctrl, current_kp, current_Ti, current_Td, fs);
}

