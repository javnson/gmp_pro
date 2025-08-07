/**
 * @file pmsm.hfi.h
 * @author Javnson (javnson@zju.edu.cn); MY_Lin(lammanyee@zju.edu.cn)
 * @brief Provides a sensorless position estimator for PMSM using High-Frequency Injection (HFI).
 * @details This module implements an HFI-based position and speed estimator. It works by
 * injecting a high-frequency voltage signal into the estimated d-axis and then
 * demodulating the resulting q-axis current response. The demodulated signal,
 * which is proportional to the rotor position error, is fed into a Phase-Locked
 * Loop (PLL) to track the rotor's angle and speed. This method is particularly
 * effective for low and zero-speed sensorless control.
 *
 * @version 0.2
 * @date 2025-08-06
 *
 * @copyright Copyright GMP(c) 2025
 *
 * //tex:
 * // The core principle relies on the motor's saliency (Ld != Lq). A high-frequency
 * // voltage, v_{dqh} = [V_h \cos(\omega_h t), 0]^T, is injected. The resulting
 * // high-frequency current response contains information about the rotor position.
 * // The demodulated q-axis current is proportional to the position error:
 * // i_{q\_demod} \propto \sin(2(\theta_e - \hat{\theta}_e))
 *
 */

#ifndef _FILE_PMSM_HFI_H_
#define _FILE_PMSM_HFI_H_

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/intrinsic/discrete/signal_generator.h>
#include <ctl/component/motor_control/basic/encoder.h>
#include <ctl/math_block/gmp_math.h>

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
 * @{
 */

//================================================================================
// Type Defines
//================================================================================

/**
 * @brief Initialization parameters for the HFI estimator module.
 */
typedef struct _tag_ctl_hfi_init
{
    // --- Motor & System Parameters ---
    uint16_t pole_pairs;         ///< Number of motor pole pairs.
    parameter_gt speed_base_rpm; ///< Base speed for per-unit conversion (RPM).
    parameter_gt f_ctrl;         ///< Controller ISR frequency (Hz).

    // --- HFI Controller Parameters ---
    parameter_gt f_hfi;      ///< HFI injection frequency (Hz).
    parameter_gt u_amp_hfi;  ///< HFI injection voltage amplitude (p.u.).
    parameter_gt iq_lp_fc;   ///< Cutoff frequency for the q-axis current LPF (Hz).
    parameter_gt iq_lp_damp; ///< Damping ratio for the q-axis current LPF.

    // --- PLL Controller Parameters ---
    ctrl_gt pid_kp;        ///< Proportional gain for the PLL's PI controller.
    ctrl_gt pid_Ti;        ///< Integral time constant for the PLL's PI controller.
    ctrl_gt pid_Td;        ///< Derivative time constant for the PLL's PI controller.
    ctrl_gt spd_max_limit; ///< Maximum speed output from the PLL (p.u.).
    ctrl_gt spd_min_limit; ///< Minimum speed output from the PLL (p.u.).

} ctl_hfi_init_t;

/**
 * @brief Holds the state and parameters for the HFI estimator.
 */
typedef struct _tag_pmsm_hfi
{
    // --- Outputs ---
    rotation_ift encif; ///< Standard encoder interface providing estimated position.
    velocity_ift spdif; ///< Standard velocity interface providing estimated speed.
    ctrl_gt ud_inj;     ///< The calculated high-frequency injection voltage for the d-axis.

    // --- Internal State Variables ---
    ctrl_gt iq_demodulate; ///< The demodulated q-axis current signal.
    ctrl_gt theta_error;   ///< The filtered position error signal fed to the PLL.
    ctrl_gt wr_est;        ///< Estimated mechanical speed from the PLL (rad/tick).
    ctrl_gt theta_r_est;   ///< Estimated mechanical rotor angle (p.u., 0 to 1.0).

    // --- Controller Entities ---
    ctl_sine_generator_t hfi_sincos_gen; ///< Generator for the injection sine/cosine signals.
    ctl_filter_IIR2_t iq_lp_filter;      ///< 2nd-order IIR LPF for extracting the error signal.
    ctl_pid_t pid_pll;                   ///< PI controller for the Phase-Locked Loop.

    // --- Pre-calculated Parameters ---
    ctrl_gt hfi_inj_amp; ///< HFI injection amplitude (p.u.).
    ctrl_gt spd_sf;      ///< Scale factor to convert speed from rad/tick to p.u.
    ctrl_gt pole_pairs;  ///< Number of pole pairs as a ctrl_gt type.

} pmsm_hfi_t;

//================================================================================
// Function Prototypes
//================================================================================

/**
 * @brief Initializes the HFI estimator module.
 * @param[out] hfi  Pointer to the HFI structure to initialize.
 * @param[in]  init Pointer to the structure containing initialization parameters.
 */
void ctl_init_pmsm_hfi(pmsm_hfi_t* hfi, const ctl_hfi_init_t* init);

/**
 * @brief Clears the internal states of the HFI estimator.
 * @param[out] hfi Pointer to the HFI structure.
 */
void ctl_clear_pmsm_hfi(pmsm_hfi_t* hfi);

/**
 * @brief Executes one step of the HFI position and speed estimation.
 * @details This function performs the core HFI algorithm: it generates the injection
 * signal, demodulates the feedback current, filters the result to get a
 * position error, and uses a PLL to track the angle and speed.
 * @param[out] hfi Pointer to the HFI structure.
 * @param[in]  iq  The measured q-axis current from the main FOC loop.
 * @return The calculated high-frequency injection voltage (ud_inj) to be added
 * to the d-axis voltage command in the FOC controller.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pmsm_hfi(pmsm_hfi_t* hfi, ctrl_gt iq)
{
    // 1. Generate the high-frequency carrier signal and the injection voltage.
    hfi->ud_inj = ctl_mul(ctl_get_sine_generator_cos(&hfi->hfi_sincos_gen), hfi->hfi_inj_amp);

    // 2. Demodulate the q-axis current using the carrier signal.
    hfi->iq_demodulate = ctl_mul(iq, ctl_get_sine_generator_sin(&hfi->hfi_sincos_gen));

    // 3. Advance the high-frequency signal generator for the next cycle.
    ctl_step_sine_generator(&hfi->hfi_sincos_gen);

    // 4. Low-pass filter the demodulated signal to extract the position error.
    hfi->theta_error = ctl_step_filter_iir2(&hfi->iq_lp_filter, hfi->iq_demodulate);

    // 5. Process the error signal through the PLL to track speed and position.
    // 5a. PI controller generates the estimated speed in rad/tick.
    hfi->wr_est = ctl_step_pid_par(&hfi->pid_pll, hfi->theta_error);

    // 5b. Integrate the mechanical speed to get the mechanical angle.
    hfi->theta_r_est += ctl_mul(hfi->wr_est, CTL_CTRL_CONST_1_OVER_2PI);
    hfi->theta_r_est = ctrl_mod_1(hfi->theta_r_est); // Wrap angle to [0, 1.0)

    // 6. Update the output interfaces.
    // 6a. Convert estimated speed to p.u. for the velocity interface.
    hfi->spdif.speed = ctl_mul(hfi->wr_est, hfi->spd_sf);

    // 6b. Provide the estimated mechanical and electrical angles.
    hfi->encif.position = hfi->theta_r_est;
    hfi->encif.elec_position = ctrl_mod_1(ctl_mul(hfi->theta_r_est, hfi->pole_pairs));

    // 7. Return the injection voltage to be used by the main controller.
    return hfi->ud_inj;
}

/** @} */ // end of PMSM_HFI group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_HFI_H_
