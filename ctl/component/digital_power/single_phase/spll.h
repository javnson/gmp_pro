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
    ctrl_gt v_mag;     /**< Estimated Grid Voltage Amplitude (Peak Value). */

    /*-- Internal Variables --*/
    vector2_gt uab;     /**< Alpha-Beta voltage (SOGI output) */
    vector2_gt udq;     /**< The input voltage transformed into the d-q rotating frame. */
    ctrl_gt freq_error; /**< The frequency error term from the loop filter (PI controller output). */

    /*-- Parameters --*/
    ctrl_gt freq_sf; /**< Scaling factor to convert per-unit frequency to per-step phase increment. */

    /*-- Submodules --*/
    discrete_sogi_t sogi;            /**< The SOGI-based orthogonal signal generator. */
    ctl_pid_t spll_ctrl;             /**< The PI controller that acts as the loop filter. */
    ctl_low_pass_filter_t filter_uq; /**< Low-pass filter for the q-axis component to reduce harmonics. */

} ctl_single_phase_pll;

/**
 * @brief Initializes the Single-Phase PLL module.
 * @param[out] spll Pointer to the Single-Phase PLL instance.
 * @param[in] gain Proportional gain (Kp) for the PLL's PI loop filter.
 * @param[in] ki Integral time constant (s) for the PLL's PI loop filter.
 * @param[in] fc Cutoff frequency (Hz) for the q-axis low-pass filter.
 * @param[in] fg Nominal grid frequency (e.g., 50 or 60 Hz).
 * @param[in] fs Controller execution frequency (Hz).
 */
void ctl_init_single_phase_pll(ctl_single_phase_pll* spll, parameter_gt gain, parameter_gt ki, parameter_gt fc,
                               parameter_gt fg, parameter_gt fs);

/**
 * @brief Initializes the Single-Phase PLL module.
 * @param[out] spll Pointer to the Single-Phase PLL instance.
 * @param[in] gain Proportional gain (Kp) for the PLL's PI loop filter.
 * @param[in] Ti Integral time constant (s) for the PLL's PI loop filter.
 * @param[in] fc Cutoff frequency (Hz) for the q-axis low-pass filter.
 * @param[in] fg Nominal grid frequency (e.g., 50 or 60 Hz).
 * @param[in] fs Controller execution frequency (Hz).
 */
void ctl_init_single_phase_pll_T(ctl_single_phase_pll* spll, parameter_gt gain, parameter_gt Ti, parameter_gt fc,
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
    // SOGI generates v_alpha (in-phase) and v_beta (quadrature, 90 deg lag)
    ctl_step_discrete_sogi(&spll->sogi, ac_input);

    // 2. Park Transform from stationary \alpha - \beta to rotating d-q frame
    // The SOGI outputs are assigned to alpha and beta components.
    // The negative sign is a convention to align the final locked phase.
    spll->uab.dat[phase_alpha] = -ctl_get_discrete_sogi_ds(&spll->sogi);
    spll->uab.dat[phase_beta] = ctl_get_discrete_sogi_qs(&spll->sogi);

    // 3. Update the phasor (sin/cos) for the next iteration's Park transform.
    ctl_set_phasor_via_angle(spll->theta, &spll->phasor);

    // 4. Park Transform (AlphaBeta -> DQ)
    // We want to lock to the Alpha axis (Input Voltage Vector).
    // In a locked system:
    // Vd = Valpha*cos + Vbeta*sin  --> Magnitude
    // Vq = -Valpha*sin + Vbeta*cos --> Error (to be driven to 0)
    ctl_ct_park2(&spll->uab, &spll->phasor, &spll->udq);

    // When Vq is regulated to 0, Vd represents the full vector magnitude.
    spll->v_mag = spll->udq.dat[phase_d];

    // 5. Loop Filter: PI controller drives the q-axis component (phase error) to zero.
    ctl_step_lowpass_filter(&spll->filter_uq, spll->udq.dat[phase_q]);
    spll->freq_error = ctl_step_pid_ser(&spll->spll_ctrl, ctl_get_lowpass_filter_result(&spll->filter_uq));

    // 6. Voltage-Controlled Oscillator (VCO)
    // The nominal frequency (1.0 p.u.) is adjusted by the error from the loop filter.
    spll->frequency = float2ctrl(1) + spll->freq_error;

    // Integrate the frequency to get the new phase angle, and warp.
    spll->theta = ctrl_mod_1(spll->theta + ctl_mul(spll->frequency, spll->freq_sf));
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

/**
 * @defgroup spll_dc_api Single Phase SOGI-DC PLL API
 * @brief A Phase-Locked Loop with DC Offset Rejection for single-phase grid synchronization.
 * @details This implementation uses a SOGI-DC block to generate orthogonal signals.
 * The DC rejection loop inside the SOGI-DC block estimates and subtracts the input DC offset,
 * preventing low-frequency oscillations (50/60Hz) in the d-q frame outputs.
 * @{
 */

/**
 * @brief Data structure for the Single-Phase SOGI-DC PLL controller.
 */
typedef struct _tag_single_phase_dc_pll
{
    /*-- Outputs --*/
    ctrl_gt frequency;    /**< Estimated grid frequency in per-unit (1.0 = nominal frequency). */
    ctrl_gt theta;        /**< Estimated grid phase angle in per-unit (0 to 1.0 represents 0 to 2*pi). */
    ctl_vector2_t phasor; /**< Output phasor containing sin(theta) and cos(theta). */
    ctrl_gt v_mag;        /**< Estimated Grid Voltage Amplitude (Peak Value). */
    ctrl_gt v_dc_est;     /**< [NEW] Estimated Grid Voltage DC Offset. */

    /*-- Internal Variables --*/
    ctl_vector2_t uab;  /**< Alpha-Beta voltage (SOGI output). */
    ctl_vector2_t udq;  /**< The input voltage transformed into the d-q rotating frame. */
    ctrl_gt freq_error; /**< The frequency error term from the loop filter (PI controller output). */

    /*-- Parameters --*/
    ctrl_gt freq_sf; /**< Scaling factor to convert per-unit frequency to per-step phase increment. */

    /*-- Submodules --*/
    discrete_sogi_dc_t sogi_dc;      /**< The SOGI-DC based orthogonal signal generator. */
    ctl_pid_t spll_ctrl;             /**< The PI controller that acts as the loop filter. */
    ctl_low_pass_filter_t filter_uq; /**< Low-pass filter for the q-axis component to reduce harmonics. */

} ctl_single_phase_dc_pll;

/**
 * @brief Initializes the Single-Phase SOGI-DC PLL module.
 * @param[out] spll Pointer to the Single-Phase PLL instance.
 * @param[in] loop_kp Proportional gain (Kp) for the PLL's PI loop filter.
 * @param[in] loop_ki Integral time constant (s) for the PLL's PI loop filter.
 * @param[in] k_sogi SOGI damping gain (typically 1.414).
 * @param[in] k_dc DC rejection gain (typically 0.5 - 1.0).
 * @param[in] fc_uq Cutoff frequency (Hz) for the q-axis low-pass filter.
 * @param[in] fg Nominal grid frequency (e.g., 50 or 60 Hz).
 * @param[in] fs Controller execution frequency (Hz).
 */
void ctl_init_single_phase_dc_pll(ctl_single_phase_dc_pll* spll, parameter_gt loop_kp, parameter_gt loop_ki,
                                  parameter_gt k_sogi, parameter_gt k_dc, parameter_gt fc_uq, parameter_gt fg,
                                  parameter_gt fs);

/**
 * @brief Clears the internal states of the PLL.
 */
GMP_STATIC_INLINE void ctl_clear_single_phase_dc_pll(ctl_single_phase_dc_pll* spll)
{
    ctl_clear_lowpass_filter(&spll->filter_uq);
    ctl_clear_discrete_sogi_dc(&spll->sogi_dc);
    ctl_clear_pid(&spll->spll_ctrl);

    spll->theta = float2ctrl(0.0f);
    ctl_set_phasor_via_angle(spll->theta, &spll->phasor);

    spll->frequency = float2ctrl(1.0f);
    spll->v_mag = 0;
    spll->v_dc_est = 0;

    ctl_vector2_clear(&spll->uab);
    ctl_vector2_clear(&spll->udq);
}

/**
 * @brief Executes one step of the Single-Phase SOGI-DC PLL algorithm.
 * @param[in,out] spll Pointer to the Single-Phase PLL instance.
 * @param[in] ac_input The instantaneous value of the single-phase AC input signal.
 */
GMP_STATIC_INLINE void ctl_step_single_phase_dc_pll(ctl_single_phase_dc_pll* spll, ctrl_gt ac_input)
{
    // 1. Orthogonal Signal Generation using SOGI-DC
    // This step removes DC from input and generates v_alpha, v_beta
    ctl_step_discrete_sogi_dc(&spll->sogi_dc, ac_input);

    // Retrieve SOGI outputs
    // Note: SOGI-DC Alpha is aligned with input (BandPass), Beta is lagging 90 (LowPass)
    // Applying the same convention as standard PLL: Alpha -> -Alpha to align frames if necessary,
    // or keep standard.
    // Convention check: Park transform expects Vd aligned with V_alpha if theta=0.
    // If we use standard: Vd = Valpha*cos + Vbeta*sin.
    spll->uab.dat[phase_alpha] = -ctl_get_discrete_sogi_dc_alpha(&spll->sogi_dc);
    spll->uab.dat[phase_beta] = ctl_get_discrete_sogi_dc_beta(&spll->sogi_dc);

    // Also retrieve the estimated DC offset for diagnostics
    spll->v_dc_est = ctl_get_discrete_sogi_dc_offset(&spll->sogi_dc);

    // 2. Update Phasor (based on previous theta)
    ctl_set_phasor_via_angle(spll->theta, &spll->phasor);

    // 3. Park Transform (AlphaBeta -> DQ)
    // Vd = Valpha*cos + Vbeta*sin  --> Magnitude
    // Vq = -Valpha*sin + Vbeta*cos --> Error
    ctl_ct_park2(&spll->uab, &spll->phasor, &spll->udq);

    // 4. Extract Output: Magnitude
    // When locked, Vd is the amplitude
    spll->v_mag = spll->udq.dat[phase_d];

    // 5. Loop Filter chain
    // LPF on Q-axis error (Optional, reduces harmonics)
    ctl_step_lowpass_filter(&spll->filter_uq, spll->udq.dat[phase_q]);

    // PI Controller drives filtered error to zero
    spll->freq_error = ctl_step_pid_ser(&spll->spll_ctrl, ctl_get_lowpass_filter_result(&spll->filter_uq));

    // 6. VCO (Frequency & Angle Update)
    spll->frequency = float2ctrl(1.0f) + spll->freq_error;

    // Integrate: theta += freq * (2pi * Ts_normalized)
    spll->theta += ctl_mul(spll->frequency, spll->freq_sf);

    // Wrap
    spll->theta = ctrl_mod_1(spll->theta);
}


/** @} */ // end of spll_api group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SINGLE_PHASE_PLL_H_
