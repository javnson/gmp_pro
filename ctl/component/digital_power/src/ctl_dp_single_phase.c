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
// Single Phase PLL
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/single_phase/spll.h>

/**
 * @brief Initializes a Single-Phase Phase-Locked Loop (SPLL) controller.
 * @ingroup spll_api
 * @details This function configures the components of the SPLL, including the
 * Second-Order Generalized Integrator (SOGI) for quadrature signal
 * generation, a low-pass filter for the q-axis component, and a PI
 * controller for the phase-locking loop.
 *
 * @param[out] spll Pointer to the `ctl_single_phase_pll` structure to be initialized.
 * @param[in] gain Proportional gain of the phase-locking PI controller.
 * @param[in] Ti Integral time constant of the phase-locking PI controller (in seconds).
 * @param[in] fc Cutoff frequency of the low-pass filter for the q-axis voltage (in Hz).
 * @param[in] fg Nominal grid frequency to be tracked (in Hz).
 * @param[in] fs Controller's sampling and execution frequency (in Hz).
 */
void ctl_init_single_phase_pll(ctl_single_phase_pll* spll, parameter_gt gain, parameter_gt Ti, parameter_gt fc,
                               parameter_gt fg, parameter_gt fs)
{
    // Clear the SPLL structure to ensure a clean state.
    ctl_clear_single_phase_pll(spll);

    // Initialize a discrete SOGI for generating alpha-beta orthogonal signals.
    ctl_init_discrete_sogi(&spll->sogi, 0.5, fg, fs);

    // Initialize a low-pass filter for the q-axis component (Uq) of the SOGI output.
    ctl_init_lp_filter(&spll->filter_uq, fs, fc);

    // Initialize the PI controller for the phase-locking loop.
    ctl_init_pid_ser(&spll->spll_ctrl, gain, Ti, 0, fs);

    // Pre-calculate the normalized grid frequency as a feed-forward term for the VCO.
    spll->frequency_sf = float2ctrl(fg / fs);
}

//////////////////////////////////////////////////////////////////////////
// Single Phase Modulation
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/single_phase/sp_modulation.h>

/**
 * @brief Initializes a single-phase H-bridge modulation module.
 * @ingroup sp_modulation_api
 * @details This function sets up the parameters for generating PWM signals for a
 * single-phase H-bridge, including dead-time compensation parameters.
 *
 * @param[out] bridge Pointer to the `single_phase_H_modulation_t` structure.
 * @param[in] pwm_full_scale The maximum value of the PWM counter, representing 100% duty cycle.
 * @param[in] pwm_deadband The dead-time value in PWM timer counts.
 * @param[in] current_deadband The current threshold (in ctrl_gt format) below which
 * dead-time compensation is inactive to prevent distortion.
 */
void ctl_init_single_phase_H_modulation(single_phase_H_modulation_t* bridge, pwm_gt pwm_full_scale, pwm_gt pwm_deadband,
                                        ctrl_gt current_deadband)
{
    bridge->pwm_full_scale = pwm_full_scale;
    // The deadband value is typically applied symmetrically, so we store half of it.
    bridge->pwm_deadband = pwm_deadband / 2;
    bridge->current_deadband = current_deadband;

    ctl_clear_single_phase_H_modulation(bridge);
}

//////////////////////////////////////////////////////////////////////////
// Single Phase PFC Control
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/single_phase/spfc.h>

/**
 * @brief Initializes a Single-Phase Power Factor Correction (SPFC) controller.
 * @ingroup spfc_api
 * @details This function configures the cascaded control structure for a single-phase PFC,
 * which consists of an outer voltage loop and an inner current loop.
 * Both controllers are implemented as series-form PID controllers.
 *
 * @param[out] pfc Pointer to the @ref spfc_t structure to be initialized.
 * @param[in] voltage_kp Proportional gain for the outer voltage loop.
 * @param[in] voltage_Ti Integral time constant for the outer voltage loop (in seconds).
 * @param[in] voltage_Td Derivative time constant for the outer voltage loop (in seconds).
 * @param[in] current_kp Proportional gain for the inner current loop.
 * @param[in] current_Ti Integral time constant for the inner current loop (in seconds).
 * @param[in] current_Td Derivative time constant for the inner current loop (in seconds).
 * @param[in] fs Controller's sampling and execution frequency (in Hz).
 */
void ctl_init_spfc_ctrl(spfc_t* pfc, parameter_gt voltage_kp, parameter_gt voltage_Ti, parameter_gt voltage_Td,
                        parameter_gt current_kp, parameter_gt current_Ti, parameter_gt current_Td, parameter_gt fs)
{
    // Initialize the series-form PID for the outer voltage control loop.
    ctl_init_pid_ser(&pfc->voltage_ctrl, voltage_kp, voltage_Ti, voltage_Td, fs);
    // Initialize the series-form PID for the inner current control loop.
    ctl_init_pid_ser(&pfc->current_ctrl, current_kp, current_Ti, current_Td, fs);
}

/**
 * @}
 */
