/**
 * @file pmsm.hfi.h
 * @author Javnson (javnson@zju.edu.cn); MY_Lin(lammanyee@zju.edu.cn)
 * @brief Implements the Pulsating Sinusoidal High-Frequency Injection (HFI) Observer.
 * @details This module enables sensorless position estimation at zero and low speeds 
 * for salient PMSMs (IPM). It injects a high-frequency voltage into the estimated 
 * d-axis and demodulates the resulting q-axis high-frequency current to extract 
 * the rotor position error.
 *
 * @version 0.2
 * @date 2025-08-06
 *
 * @copyright Copyright GMP(c) 2025
 *
 */

#ifndef _FILE_PMSM_HFI_H_
#define _FILE_PMSM_HFI_H_

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>
#include <ctl/component/motor_control/consultant/pu_consultant.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/component/motor_control/observer/ato_pll.h>
#include <ctl/math_block/coordinate/coord_trans.h>
#include <ctl/math_block/vector_lite/vector2.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* PMSM High-Frequency Injection (HFI) Position Estimator                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup PMSM_HFI PMSM HFI Estimator
 * @brief A module for sensorless position and speed estimation using HFI.
 * @details Implements the signal injection, demodulation, filtering, and PLL
 * required to track the rotor angle of a salient PMSM at low speeds.
 * 
 * This module implements an HFI-based position and speed estimator. It works by
 * injecting a high-frequency voltage signal into the estimated d-axis and then
 * demodulating the resulting q-axis current response. The demodulated signal,
 * which is proportional to the rotor position error, is fed into a Phase-Locked
 * Loop (PLL) to track the rotor's angle and speed. This method is particularly
 * effective for low and zero-speed sensorless control.
 * The core principle relies on the motor's saliency @f( (Ld != Lq) @f). A high-frequency
 * voltage, @f( v_{dqh} = [V_h \cos(\omega_h t), 0]^T @f), is injected. The resulting
 * high-frequency current response contains information about the rotor position.
 * The demodulated q-axis current is proportional to the position error:
 * @f[ i_{q\_demod} \propto \sin(2(\theta_e - \hat{\theta}_e)) @f]
 * @{
 */

/**
 * @brief Raw initialization structure for the HFI Observer.
 */
typedef struct _tag_pmsm_hfi_init_t
{
    // --- Injection Parameters ---
    parameter_gt v_inj_pu; //!< Injection voltage amplitude (PU).
    parameter_gt f_inj_hz; //!< Injection frequency (Hz). Typically 500Hz ~ 2000Hz.

    // --- Execution & Tuning Parameters ---
    parameter_gt fs;             //!< Controller execution frequency (Hz).
    parameter_gt f_lpf_iq_hz;    //!< Cutoff freq for extracting fundamental Iq (Hz). Used for HPF equivalence.
    parameter_gt f_lpf_demod_hz; //!< Cutoff freq for the demodulated error signal (Hz).
    parameter_gt ato_bw_hz;      //!< Bandwidth for the ATO/PLL (Hz).

    // --- Compensation & Scaling ---
    parameter_gt delay_comp_rad; //!< Phase delay compensation for the demodulation carrier (Radians).
    parameter_gt err_gain_sf;    //!< Scale factor to normalize the demodulated error to roughly 1.0 PU at max error.

} ctl_pmsm_hfi_init_t;

/**
 * @brief Main state structure for the PMSM HFI controller.
 */
typedef struct _tag_pmsm_hfi_t
{
    // --- Standard Outputs ---
    rotation_ift pos_out;
    velocity_ift spd_out;
    ctrl_gt v_d_inj_out; //!< Output: The HF voltage to be ADDED to the d-axis voltage command (PU).

    // --- Core Sub-modules ---
    ctl_ato_pll_t ato_pll;
    ctl_filter_IIR1_t lpf_iq;    //!< LPF to isolate fundamental Iq (used to create HPF).
    ctl_filter_IIR1_t lpf_demod; //!< LPF to smooth the demodulated angle error.

    // --- Carrier Generation States ---
    ctrl_gt carrier_angle_pu;     //!< High-frequency carrier angle (PU).
    ctl_vector2_t carrier_phasor; //!< Carrier phasor [cos(wh*t), sin(wh*t)].
    ctl_vector2_t demod_phasor;   //!< Phase-shifted carrier for demodulation [cos(wh*t + phi), sin(wh*t + phi)].

    // --- Scale Factors & Constants ---
    ctrl_gt sf_carrier_step;  //!< Step size per tick for the injection carrier (PU).
    ctrl_gt v_inj_amp;        //!< Amplitude of the injected voltage (PU).
    ctrl_gt sf_delay_comp_pu; //!< Phase delay compensation angle (PU).
    ctrl_gt sf_err_gain;      //!< Gain to normalize the extracted error signal.

    // --- Flags ---
    fast_gt flag_enable;

} ctl_pmsm_hfi_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_init_pmsm_hfi(ctl_pmsm_hfi_t* hfi, const ctl_pmsm_hfi_init_t* init);

/**
 * @brief Advanced initialization function utilizing the Consultant models.
 * @details Automatically calculates the optimal error normalization gain based on 
 * the motor's physical saliency (Lq - Ld).
 */
void ctl_init_pmsm_hfi_consultant(ctl_pmsm_hfi_t* hfi, const ctl_consultant_pmsm_t* motor,
                                  const ctl_consultant_pu_pmsm_t* pu, parameter_gt fs, parameter_gt f_inj_hz,
                                  parameter_gt v_inj_v, parameter_gt ato_bw_hz);

GMP_STATIC_INLINE void ctl_clear_pmsm_hfi(ctl_pmsm_hfi_t* hfi)
{
    hfi->carrier_angle_pu = float2ctrl(0.0f);
    hfi->v_d_inj_out = float2ctrl(0.0f);

    ctl_clear_filter_iir1(&hfi->lpf_iq);
    ctl_clear_filter_iir1(&hfi->lpf_demod);
    ctl_clear_ato_pll(&hfi->ato_pll);

    hfi->pos_out.elec_position = float2ctrl(0.0f);
    hfi->spd_out.speed = float2ctrl(0.0f);
}

GMP_STATIC_INLINE void ctl_enable_pmsm_hfi(ctl_pmsm_hfi_t* hfi)
{
    hfi->flag_enable = 1;
}
GMP_STATIC_INLINE void ctl_disable_pmsm_hfi(ctl_pmsm_hfi_t* hfi)
{
    hfi->flag_enable = 0;
    hfi->v_d_inj_out = float2ctrl(0.0f);
}

/**
 * @brief Executes one high-frequency step of the Pulsating HFI Observer.
 * @details Generates the d-axis injection voltage and demodulates the q-axis current.
 * @param[in,out] hfi      Pointer to the HFI instance.
 * @param[in]     i_alpha  Measured alpha-axis stator current (PU).
 * @param[in]     i_beta   Measured beta-axis stator current (PU).
 */
GMP_STATIC_INLINE void ctl_step_pmsm_hfi(ctl_pmsm_hfi_t* hfi, ctrl_gt i_alpha, ctrl_gt i_beta)
{
    if (!hfi->flag_enable)
    {
        hfi->v_d_inj_out = float2ctrl(0.0f);
        return;
    }

    // ========================================================================
    // 1. High-Frequency Carrier Generation & Voltage Injection
    // ========================================================================
    hfi->carrier_angle_pu += hfi->sf_carrier_step;
    hfi->carrier_angle_pu = ctrl_mod_1(hfi->carrier_angle_pu);

    // Generate injection phasor and output voltage: Vd_inj = V_inj * cos(w_h * t)
    ctl_set_phasor_via_angle(hfi->carrier_angle_pu, &hfi->carrier_phasor);
    hfi->v_d_inj_out = ctl_mul(hfi->v_inj_amp, hfi->carrier_phasor.dat[0]); // cos is dat[0]

    // ========================================================================
    // 2. High-Frequency Current Extraction (Equivalent HPF)
    // ========================================================================
    // Park Transform to get i_q in the estimated synchronous frame
    ctl_vector2_t rotor_phasor;
    ctl_set_phasor_via_angle(hfi->ato_pll.elec_angle_pu, &rotor_phasor);
    ctrl_gt i_q_est = -ctl_mul(i_alpha, rotor_phasor.dat[1]) + ctl_mul(i_beta, rotor_phasor.dat[0]);

    // Fast HPF implementation: HPF(x) = x - LPF(x). Isolates the HF current from the torque current.
    ctrl_gt i_q_lf = ctl_step_filter_iir1(&hfi->lpf_iq, i_q_est);
    ctrl_gt i_q_hf = i_q_est - i_q_lf;

    // ========================================================================
    // 3. Demodulation
    // ========================================================================
    // To align with the current response, the demodulation carrier must be phase-shifted.
    ctrl_gt demod_angle_pu = ctrl_mod_1(hfi->carrier_angle_pu + hfi->sf_delay_comp_pu);
    ctl_set_phasor_via_angle(demod_angle_pu, &hfi->demod_phasor);

    // Multiply HF current by sin(w_h * t + phi_delay).
    // Theory: i_q_hf ~ sin(2*delta_theta) * sin(w_h * t). Demodulating with sin gives DC error.
    ctrl_gt err_raw = ctl_mul(i_q_hf, hfi->demod_phasor.dat[1]); // sin is dat[1]

    // Normalize error magnitude based on motor saliency (improves PLL robustness)
    err_raw = ctl_mul(err_raw, hfi->sf_err_gain);

    // Extract the DC envelope (which is proportional to sin(2 * delta_theta))
    ctrl_gt err_dc = ctl_step_filter_iir1(&hfi->lpf_demod, err_raw);

    // ========================================================================
    // 4. ATO/PLL Tracking
    // ========================================================================
    // Note: HFI error curve is sin(2*delta_theta), meaning it has two zero-crossings per electrical cycle.
    // N/S pole ambiguity must be resolved by initial polarity checks, but tracking works for either once locked.
    ctl_step_ato_pll(&hfi->ato_pll, err_dc);

    // ========================================================================
    // 5. Output to Top-Level Interfaces
    // ========================================================================
    hfi->pos_out.elec_position = hfi->ato_pll.elec_angle_pu;
    hfi->spd_out.speed = hfi->ato_pll.elec_speed_pu;
}
/** @} */ // end of PMSM_HFI group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_HFI_H_
