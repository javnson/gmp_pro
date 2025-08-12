/**
 * @file spll.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Implements a Single-Phase Phase-Locked Loop (SPLL) using a Second-Order Generalized Integrator (SOGI).
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef _FILE_SINGLE_PHASE_PLL_H_
#define _FILE_SINGLE_PHASE_PLL_H_

#include <ctl/component/interface/interface_base.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_sogi.h>
#include <ctl/math_block/coordinate/coord_trans.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup spll_api Single Phase PLL API
 * @brief A Phase-Locked Loop for single-phase grid synchronization.
 * @details This file provides an implementation of a SOGI-based PLL, which is a robust method
 * for tracking the phase and frequency of a single-phase AC signal. It is commonly used
 * in grid-tied applications like PFC rectifiers and inverters.
 *
 * The PLL operates as follows:
 * 1.  A Second-Order Generalized Integrator (SOGI) takes the single-phase input and
 * generates a pair of orthogonal @f((\alpha -\beta)@f) signals.
 * 2.  A Park transform converts these @f((\alpha -\beta)@f) signals into a rotating d-q frame using the
 * PLL's estimated phase angle.
 * 3.  A PI controller acts as a loop filter, driving the q-component of the transformed
 * signal to zero.
 * 4.  A Voltage-Controlled Oscillator (VCO) integrates the output of the loop filter to
 * generate the estimated frequency and phase, which closes the loop.
 * @{
 * @ingroup CTL_DP_LIB
 */

/**
 * @brief Data structure for the Single-Phase PLL controller.
 */
typedef struct _tag_single_phase_pll
{
    /*-- Outputs --*/
    ctrl_gt frequency; /**< Estimated grid frequency in per-unit (1.0 = nominal frequency). */
    ctrl_gt theta;     /**< Estimated grid phase angle in per-unit (0 to 1.0 represents 0 to 2*pi). */
    vector2_gt phasor; /**< Output phasor containing sin(theta) and cos(theta). */

    /*-- Internal Variables --*/
    vector2_gt udq;     /**< The input voltage transformed into the d-q rotating frame. */
    ctrl_gt freq_error; /**< The frequency error term from the loop filter (PI controller output). */

    /*-- Parameters --*/
    ctrl_gt frequency_sf; /**< Scaling factor to convert per-unit frequency to per-step phase increment. */

    /*-- Submodules --*/
    discrete_sogi_t sogi;            /**< The SOGI-based orthogonal signal generator. */
    ctl_pid_t spll_ctrl;             /**< The PI controller that acts as the loop filter. */
    ctl_low_pass_filter_t filter_uq; /**< Low-pass filter for the q-axis component to reduce harmonics. */

} ctl_single_phase_pll;

/**
 * @brief Initializes the Single-Phase PLL module.
 * @param[out] spll Pointer to the Single-Phase PLL instance.
 * @param[in] gain Proportional gain (Kp) for the PLL's PI loop filter.
 * @param[in] Ti Integral time constant (s) for the PLL's PI loop filter.
 * @param[in] fc Cutoff frequency (Hz) for the q-axis low-pass filter.
 * @param[in] fg Nominal grid frequency (e.g., 50 or 60 Hz).
 * @param[in] fs Controller execution frequency (Hz).
 */
void ctl_init_single_phase_pll(ctl_single_phase_pll* spll, parameter_gt gain, parameter_gt Ti, parameter_gt fc,
                               parameter_gt fg, parameter_gt fs);

/**
 * @brief Clears the internal states of the PLL.
 * @details Resets the SOGI, filters, and phase angle to a known initial state.
 * @param[in,out] spll Pointer to the Single-Phase PLL instance.
 */
GMP_STATIC_INLINE void ctl_clear_single_phase_pll(ctl_single_phase_pll* spll)
{
    ctl_clear_lowpass_filter(&spll->filter_uq);
    ctl_clear_discrete_sogi(&spll->sogi);
    ctl_clear_pid(&spll->spll_ctrl);

    spll->theta = float2ctrl(0.0f);
    ctl_set_phasor_via_angle(spll->theta, &spll->phasor);
}

/**
 * @brief Executes one step of the Single-Phase PLL algorithm.
 * @param[in,out] spll Pointer to the Single-Phase PLL instance.
 * @param[in] ac_input The instantaneous value of the single-phase AC input signal.
 */
GMP_STATIC_INLINE void ctl_step_single_phase_pll(ctl_single_phase_pll* spll, ctrl_gt ac_input)
{
    // 1. Orthogonal Signal Generation using SOGI
    ctl_step_discrete_sogi(&spll->sogi, ac_input);

    // 2. Park Transform from stationary ¦Á-¦Â to rotating d-q frame
    vector2_gt uab;
    // The SOGI outputs are assigned to alpha and beta components.
    // The negative sign is a convention to align the final locked phase.
    uab.dat[phase_alpha] = -ctl_get_discrete_sogi_ds(&spll->sogi);
    uab.dat[phase_beta] = ctl_get_discrete_sogi_qs(&spll->sogi);
    ctl_ct_park2(&uab, &spll->phasor, &spll->udq);

    // 3. Loop Filter: PI controller drives the q-axis component (phase error) to zero.
    ctl_step_lowpass_filter(&spll->filter_uq, spll->udq.dat[phase_q]);
    spll->freq_error = ctl_step_pid_ser(&spll->spll_ctrl, ctl_get_lowpass_filter_result(&spll->filter_uq));

    // 4. Voltage-Controlled Oscillator (VCO)
    // The nominal frequency (1.0 p.u.) is adjusted by the error from the loop filter.
    spll->frequency = 1 + spll->freq_error;
    // Integrate the frequency to get the new phase angle.
    spll->theta = ctrl_mod_1(spll->theta + ctl_mul(spll->frequency, spll->frequency_sf));
    // Update the phasor (sin/cos) for the next iteration's Park transform.
    ctl_set_phasor_via_angle(spll->theta, &spll->phasor);
}

/**
 * @brief Gets the filtered q-axis component, which represents the phase error.
 * @param[in] spll Pointer to the Single-Phase PLL instance.
 * @return The filtered phase error signal (udq.q).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_spll_error_fbk(ctl_single_phase_pll* spll)
{
    return ctl_get_lowpass_filter_result(&spll->filter_uq);
}

/** @} */ // end of spll_api group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SINGLE_PHASE_PLL_H_
