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
