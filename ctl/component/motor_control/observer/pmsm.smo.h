/**
 * @file pmsm.smo.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a sensorless position estimator for PMSM using a Sliding Mode Observer (SMO).
 *
 * @version 0.2
 * @date 2025-08-06
 *
 * @copyright Copyright GMP(c) 2025
 *
 */

#ifndef _FILE_PMSM_SMO_H_
#define _FILE_PMSM_SMO_H_

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/basic/encoder.h>
#include <ctl/math_block/gmp_math.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* PMSM Sliding Mode Observer (SMO) Position Estimator                       */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup PMSM_SMO PMSM SMO Estimator
 * @brief A module for sensorless position and speed estimation using SMO.
 * @details Implements the observer model, sliding control law, filtering, and PLL
 * required to track the rotor angle of a PMSM.
 * 
 * This module implements an SMO to estimate the rotor position and speed of a
 * Permanent Magnet Synchronous Motor. It uses the motor's voltage and current
 * measurements to drive an observer model. A sliding control law generates a
 * correction term (z) that, when filtered, represents the motor's back-EMF.
 * A Phase-Locked Loop (PLL) then tracks the back-EMF signal to extract the
 * rotor angle and speed.
 *
 * The SMO is based on the PMSM voltage model in the stationary @f(\alpha-\beta@f) frame.
 * The observer estimates the stator currents:
 * @f[ \frac{d\hat{i}_{\alpha\beta}}{dt} = \frac{1}{L_s}(v_{\alpha\beta} - R_s\hat{i}_{\alpha\beta} - z_{\alpha\beta}) @f]
 * The sliding control law, z, forces the estimated current to track the measured current:
 * @f[ z_{\alpha\beta} = k_{slide} \cdot \text{sign}(\hat{i}_{\alpha\beta} - i_{\alpha\beta}) @f]
 * When in the sliding mode, the filtered value of z is equal to the back-EMF: 
 * @f[ E_{\alpha\beta} \approx \bar{z}_{\alpha\beta} @f]
 * @{
 */

//================================================================================
// Type Defines
//================================================================================

/**
 * @brief Initialization parameters for the SMO estimator module.
 */
typedef struct _tag_ctl_smo_init
{
    // --- Motor Model Parameters ---
    parameter_gt Rs;     ///< Motor Stator Resistance (Ohm).
    parameter_gt Ld;     ///< Motor d-axis Inductance (H).
    parameter_gt Lq;     ///< Motor q-axis Inductance (H).
    uint16_t pole_pairs; ///< Number of motor pole pairs.

    // --- System & Per-Unit Parameters ---
    parameter_gt speed_base_rpm; ///< Base speed for per-unit conversion (RPM).
    parameter_gt f_ctrl;         ///< Controller ISR frequency (Hz).
    parameter_gt u_base;         ///< Base voltage for per-unit conversion (V).
    parameter_gt i_base;         ///< Base current for per-unit conversion (A).

    // --- SMO & PLL Controller Parameters ---
    parameter_gt fc_e;     ///< Cutoff frequency for the back-EMF LPF (Hz).
    parameter_gt fc_omega; ///< Cutoff frequency for the speed LPF (Hz).
    ctrl_gt pid_kp;        ///< Proportional gain for the PLL's PI controller.
    ctrl_gt pid_Ti;        ///< Integral time for the PLL's PI controller.
    ctrl_gt pid_Td;        ///< Derivative time for the PLL's PI controller.
    ctrl_gt spd_max_limit; ///< Maximum speed output from the PLL (p.u.).
    ctrl_gt spd_min_limit; ///< Minimum speed output from the PLL (p.u.).
    ctrl_gt k_slide;       ///< The sliding gain for the observer.

} ctl_smo_init_t;

/**
 * @brief Holds the state and parameters for the SMO estimator.
 */
typedef struct _tag_pmsm_smo
{
    // --- Outputs ---
    rotation_ift encif; ///< Standard encoder interface providing estimated position.
    velocity_ift spdif; ///< Standard velocity interface providing estimated speed.

    // --- Internal State Variables ---
    ctl_vector2_t i_est;  ///< Estimated stator current vector [i_alpha_est, i_beta_est]^T.
    ctl_vector2_t e_est;  ///< Estimated back-EMF vector [e_alpha_est, e_beta_est]^T.
    ctl_vector2_t z;      ///< Sliding control law output vector [z_alpha, z_beta]^T.
    ctrl_gt wr_est;       ///< Estimated electrical speed from the PLL (rad/tick).
    ctrl_gt theta_est;    ///< Estimated electrical angle (p.u., 0 to 1.0).
    ctl_vector2_t phasor; ///< Estimated phasor (sin, cos).

    // --- Controller Entities ---
    ctl_low_pass_filter_t filter_e[2]; ///< LPFs for back-EMF components (alpha, beta).
    ctl_low_pass_filter_t filter_spd;  ///< LPF for the estimated speed.
    ctl_pid_t pid_pll;                 ///< PI controller for the Phase-Locked Loop.

    // --- Pre-calculated Model Parameters ---
    ctrl_gt k1;               ///< Model parameter: Ts / Ld.
    ctrl_gt k2;               ///< Model parameter: Ts * Rs / Ld.
    ctrl_gt k3;               ///< Model parameter: (Ld - Lq) * Ts / Ld (coupling term).
    ctrl_gt k_slide;          ///< Sliding gain.
    ctrl_gt spd_sf;           ///< Scale factor to convert speed from rad/tick to p.u.
    ctrl_gt theta_compensate; ///< Pre-calculated constant for phase lag compensation.

} pmsm_smo_t;

//================================================================================
// Function Prototypes
//================================================================================

/**
 * @brief Initializes the SMO estimator module.
 * @param[out] smo  Pointer to the SMO structure to initialize.
 * @param[in]  init Pointer to the structure containing initialization parameters.
 */
void ctl_init_pmsm_smo(pmsm_smo_t* smo, const ctl_smo_init_t* init);

/**
 * @brief Clears the internal states of the SMO estimator.
 * @param[out] smo Pointer to the SMO structure.
 */
void ctl_clear_pmsm_smo(pmsm_smo_t* smo);

/**
 * @brief Executes one step of the SMO position and speed estimation.
 * @param[out] smo      Pointer to the SMO structure.
 * @param[in]  u_alpha  The measured/commanded alpha-axis voltage.
 * @param[in]  u_beta   The measured/commanded beta-axis voltage.
 * @param[in]  i_alpha  The measured alpha-axis current.
 * @param[in]  i_beta   The measured beta-axis current.
 * @return The final estimated electrical angle with phase compensation (p.u.).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pmsm_smo(pmsm_smo_t* smo, ctrl_gt u_alpha, ctrl_gt u_beta, ctrl_gt i_alpha,
                                            ctrl_gt i_beta)
{
    // 1. Update the PMSM observer model to get the next estimated current.
    // This is a discrete-time model of the motor's electrical dynamics.
    ctrl_gt delta_i_alpha = ctl_mul(smo->k1, u_alpha - smo->z.dat[0]) - ctl_mul(smo->k2, smo->i_est.dat[0]) -
                            ctl_mul(ctl_mul(smo->wr_est, smo->k3), smo->i_est.dat[1]);

    ctrl_gt delta_i_beta = ctl_mul(smo->k1, u_beta - smo->z.dat[1]) - ctl_mul(smo->k2, smo->i_est.dat[1]) +
                           ctl_mul(ctl_mul(smo->wr_est, smo->k3), smo->i_est.dat[0]);

    smo->i_est.dat[0] += delta_i_alpha;
    smo->i_est.dat[1] += delta_i_beta;

    // 2. Apply the sliding control law.
    // The output 'z' is proportional to the sign of the current error.
    // A saturation function is used as a practical replacement for the signum function.
    smo->z.dat[0] = ctl_mul(ctl_sat(smo->i_est.dat[0] - i_alpha, float2ctrl(0.1), -float2ctrl(0.1)), smo->k_slide);
    smo->z.dat[1] = ctl_mul(ctl_sat(smo->i_est.dat[1] - i_beta, float2ctrl(0.1), -float2ctrl(0.1)), smo->k_slide);

    // 3. Low-pass filter the sliding term 'z' to extract the estimated back-EMF.
    smo->e_est.dat[0] = ctl_step_lowpass_filter(&smo->filter_e[0], smo->z.dat[0]);
    smo->e_est.dat[1] = ctl_step_lowpass_filter(&smo->filter_e[1], smo->z.dat[1]);

    // 4. Process the back-EMF through the PLL to track angle and speed.
    // 4a. Generate the PLL error signal (estimated q-axis component of back-EMF).
    ctl_set_phasor_via_angle(smo->theta_est, &smo->phasor);
    ctrl_gt e_error = -ctl_mul(smo->e_est.dat[0], smo->phasor.dat[1]) + ctl_mul(smo->e_est.dat[1], smo->phasor.dat[0]);

    // 4b. PI controller generates the estimated speed command.
    ctrl_gt pid_out = ctl_step_pid_par(&smo->pid_pll, e_error);

    // 4c. Low-pass filter the speed for a smoother estimate (in rad/tick).
    smo->wr_est = ctl_step_lowpass_filter(&smo->filter_spd, pid_out);

    // 4d. Integrate the speed to get the next estimated angle.
    smo->theta_est += ctl_mul(smo->wr_est, CTL_CTRL_CONST_1_OVER_2PI);
    smo->theta_est = ctrl_mod_1(smo->theta_est); // Wrap angle to [0, 1.0)

    // 5. Update the output interfaces.
    // 5a. Convert speed to p.u. and apply to the velocity interface.
    smo->spdif.speed = ctl_mul(smo->wr_est, smo->spd_sf);

    // 5b. Apply phase compensation to the angle for the encoder interface.
    // This corrects for the phase lag introduced by the back-EMF low-pass filters.
    ctrl_gt phase_lag_comp = ctl_atan2(ctl_mul(smo->wr_est, smo->theta_compensate), CTL_CTRL_CONST_1);
    smo->encif.elec_position = smo->theta_est + ctl_mul(phase_lag_comp, CTL_CTRL_CONST_1_OVER_2PI);
    smo->encif.elec_position = ctrl_mod_1(smo->encif.elec_position);

    return smo->encif.elec_position;
}

/** @} */ // end of PMSM_SMO group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_SMO_H_
