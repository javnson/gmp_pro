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

/**
 * @brief Initializes the three-phase PLL controller.
 * @ingroup CTL_PLL_API
 *
 * @param[out] pll Pointer to the `three_phase_pll_t` structure to be initialized.
 * @param[in] f_base The nominal grid frequency (e.g., 50 or 60 Hz), used as the per-unit base.
 * @param[in] pid_kp Proportional gain for the phase-locking PI controller.
 * @param[in] pid_Ti Integral time constant for the phase-locking PI controller (in seconds).
 * @param[in] pid_Td Derivative time constant (typically 0 for a PI controller).
 * @param[in] f_ctrl The controller's execution frequency (sampling frequency) in Hz.
 */
void ctl_init_pll_3ph(three_phase_pll_t* pll, parameter_gt f_base, parameter_gt pid_kp, parameter_gt pid_Ti,
                      parameter_gt pid_Td, parameter_gt f_ctrl)
{
    // It's good practice to clear the controller's state first.
    ctl_clear_pll_3ph(pll);

    // Initialize the PI controller for the phase-locking loop.
    ctl_init_pid(&pll->pid_pll, pid_kp, pid_Ti, pid_Td, f_ctrl);

    // Calculate and set the frequency scaling factor.
    pll->freq_sf = float2ctrl(f_base / f_ctrl);
}

//////////////////////////////////////////////////////////////////////////
// Three Phase Modulation
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/three_phase/tp_modulation.h>

/**
 * @brief Initializes the three-phase bridge modulation module.
 * @ingroup CTL_TP_MODULATION_API
 *
 * @param[out] bridge Pointer to the `three_phase_bridge_modulation_t` structure.
 * @param[in] pwm_full_scale The maximum value of the PWM counter.
 * @param[in] pwm_deadband The total dead-time value in PWM timer counts.
 * @param[in] current_deadband The current threshold to enable dead-time compensation.
 */
void ctl_init_three_phase_bridge_modulation(three_phase_bridge_modulation_t* bridge, pwm_gt pwm_full_scale,
                                            pwm_gt pwm_deadband, ctrl_gt current_deadband)
{
    bridge->pwm_full_scale = pwm_full_scale;
    // Pre-calculate and store half of the deadband for runtime efficiency.
    bridge->pwm_deadband_half = pwm_deadband / 2;
    bridge->current_deadband = current_deadband;

    ctl_clear_three_phase_bridge_modulation(bridge);
}

/**
 * @}
 */
