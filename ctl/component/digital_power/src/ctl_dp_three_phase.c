/**
 * @file ctl_dp_three_phase.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation for three-phase digital power controller modules.
 * @version 1.05
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @brief Initialization functions for three-phase digital power components,
 * including the Three-Phase PLL and bridge modulation modules.
 */

#include <gmp_core.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Three Phase PLL
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/three_phase/pll.h>

void ctl_init_pll_3ph(three_phase_pll_t* pll, parameter_gt f_base, parameter_gt pid_kp, parameter_gt pid_Ti,
                      parameter_gt pid_Td, parameter_gt f_ctrl)
{
    // Clear all internal states before initialization.
    ctl_clear_pll_3ph(pll);

    // Calculate the frequency scaling factor. This converts the per-unit frequency
    // into a per-unit angle increment for the given sampling time.
    pll->freq_sf = float2ctrl(f_base / f_ctrl);

    // Initialize the parallel-form PI controller for the loop.
    ctl_init_pid_Tmode(&pll->pid_pll, pid_kp, pid_Ti, pid_Td, f_ctrl);
}

//////////////////////////////////////////////////////////////////////////
// Three Phase Modulation
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/three_phase/tp_modulation.h>

void ctl_init_three_phase_bridge_modulation(three_phase_bridge_modulation_t* bridge, pwm_gt pwm_full_scale,
                                            pwm_gt pwm_deadband, ctrl_gt current_deadband)
{
    bridge->pwm_full_scale = pwm_full_scale;
    // Pre-calculate and store half of the deadband for runtime efficiency.
    bridge->pwm_deadband_half = pwm_deadband / 2;
    bridge->current_deadband = current_deadband;

    ctl_clear_three_phase_bridge_modulation(bridge);
}

