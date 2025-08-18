/**
 * @file acm_smo.h
 * @brief Implements a sensorless speed and flux observer for ACM using a Sliding Mode Observer (SMO).
 *
 * @version 1.0
 * @date 2025-08-07
 *
 */

#ifndef _FILE_ACM_SMO_H_
#define _FILE_ACM_SMO_H_

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* AC Induction Motor (ACM) Sliding Mode Observer                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup ACM_SMO ACM Sliding Mode Observer
 * @brief A sensorless module for estimating speed and flux angle of an ACM.
 * @details This module provides a sensorless estimator for AC Induction Motors. It uses a
 * full-order SMO to estimate both the stator currents and the rotor flux vector
 * in the stationary alpha-beta frame. A sliding control law, based on the stator
 * current error, generates a correction term. This correction term drives the
 * observer model, and the estimated rotor flux vector is then processed by a
 * Phase-Locked Loop (PLL) to extract the rotor flux angle, electrical speed,
 * and mechanical speed.
 * 
 * The observer is based on the full-order discrete-time model of the ACM:
 * @f[ \frac{d\mathbf{x}}{dt} = \mathbf{A}(\omega_r)\mathbf{x} + \mathbf{B}\mathbf{u} @f]
 * Where the state vector is @f( \mathbf{x} = [\mathbf{i}_s, \mathbf{\psi}_r]^T @f).
 * A sliding control law Z is applied to the current dynamics based on current error:
 * @f( \frac{d\hat{\mathbf{i}}_s}{dt} = ... - \mathbf{Z} @f)
 * The PLL then tracks the angle of the estimated rotor flux @f( \hat{\mathbf{\psi}}_r @f).
 * @{
 */

//================================================================================
// Type Defines
//================================================================================

/**
 * @brief Initialization parameters for the ACM SMO module.
 */
typedef struct
{
    // --- Motor Parameters (SI units) ---
    parameter_gt Rs;     ///< Stator Resistance (Ohm).
    parameter_gt Rr;     ///< Rotor Resistance (Ohm).
    parameter_gt Ls;     ///< Stator Inductance (H).
    parameter_gt Lr;     ///< Rotor Inductance (H).
    parameter_gt Lm;     ///< Mutual Inductance (H).
    uint16_t pole_pairs; ///< Number of motor pole pairs.

    // --- System & Per-Unit Parameters ---
    parameter_gt f_ctrl;         ///< Controller ISR frequency (Hz).
    parameter_gt speed_base_rpm; ///< Base speed for per-unit conversion (RPM).

    // --- SMO & PLL Controller Parameters ---
    parameter_gt pll_kp;       ///< Proportional gain for the PLL's PI controller.
    parameter_gt pll_ki;       ///< Integral gain for the PLL's PI controller.
    parameter_gt speed_lpf_fc; ///< Cutoff frequency for the speed LPF (Hz).
    ctrl_gt k_slide;           ///< The sliding gain for the observer.

} ctl_acm_smo_init_t;

/**
 * @brief Main structure for the ACM SMO estimator.
 */
typedef struct
{
    // --- Outputs ---
    rotation_ift encif; ///< Standard encoder interface providing estimated flux position.
    velocity_ift spdif; ///< Standard velocity interface providing estimated speed.

    // --- Internal State Variables ---
    ctl_vector2_t i_s_est;   ///< Estimated stator current vector [i_alpha_est, i_beta_est]^T.
    ctl_vector2_t psi_r_est; ///< Estimated rotor flux vector [psi_r_alpha, psi_r_beta]^T.
    ctl_vector2_t z;         ///< Sliding control law output vector [z_alpha, z_beta]^T.
    ctrl_gt omega_e_est;     ///< Estimated electrical speed from the PLL (rad/s).
    ctrl_gt theta_est;       ///< Estimated electrical angle (p.u., 0 to 1.0).

    // --- Controller Entities ---
    ctl_pid_t pid_pll;                ///< PI controller for the Phase-Locked Loop.
    ctl_low_pass_filter_t filter_spd; ///< LPF for the estimated speed.

    // --- Pre-calculated Model Parameters ---
    ctrl_gt c1, c2, c3, c4, c5; ///< Coefficients for the discrete-time model.
    ctrl_gt k_slide;            ///< Sliding gain.
    ctrl_gt pole_pairs;         ///< Number of pole pairs.
    ctrl_gt speed_pu_sf;        ///< Scale factor to convert speed from rad/s to p.u.
    ctrl_gt Ts;                 ///< Controller sampling time (s).

} ctl_acm_smo_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the ACM SMO estimator module.
 * @param[out] smo  Pointer to the SMO structure to initialize.
 * @param[in]  init Pointer to the structure containing initialization parameters.
 */
void ctl_init_acm_smo(ctl_acm_smo_t* smo, const ctl_acm_smo_init_t* init);

/**
 * @brief Executes one step of the ACM SMO estimation.
 * @param[out] smo      Pointer to the SMO structure.
 * @param[in]  u_alpha  The commanded alpha-axis voltage.
 * @param[in]  u_beta   The commanded beta-axis voltage.
 * @param[in]  i_alpha  The measured alpha-axis current.
 * @param[in]  i_beta   The measured beta-axis current.
 */
GMP_STATIC_INLINE void ctl_step_acm_smo(ctl_acm_smo_t* smo, ctrl_gt u_alpha, ctrl_gt u_beta, ctrl_gt i_alpha,
                                        ctrl_gt i_beta)
{
    // 1. Apply the sliding control law based on current error
    smo->z.dat[0] = ctl_mul(ctl_sat(smo->i_s_est.dat[0] - i_alpha, 1.0f, -1.0f), smo->k_slide);
    smo->z.dat[1] = ctl_mul(ctl_sat(smo->i_s_est.dat[1] - i_beta, 1.0f, -1.0f), smo->k_slide);

    // 2. Update the observer model using the discrete-time equations
    // Rotor flux estimation
    ctrl_gt d_psi_r_alpha = smo->c5 * smo->i_s_est.dat[0] - (1.0f / smo->c5 * smo->c3) * smo->psi_r_est.dat[0] -
                            smo->omega_e_est * smo->psi_r_est.dat[1];
    ctrl_gt d_psi_r_beta = smo->c5 * smo->i_s_est.dat[1] - (1.0f / smo->c5 * smo->c3) * smo->psi_r_est.dat[1] +
                           smo->omega_e_est * smo->psi_r_est.dat[0];
    smo->psi_r_est.dat[0] += d_psi_r_alpha * smo->Ts;
    smo->psi_r_est.dat[1] += d_psi_r_beta * smo->Ts;

    // Stator current estimation
    ctrl_gt d_i_s_alpha = smo->c1 * u_alpha - smo->c2 * smo->i_s_est.dat[0] + smo->c3 * smo->psi_r_est.dat[0] +
                          smo->c4 * smo->omega_e_est * smo->psi_r_est.dat[1] - smo->z.dat[0];
    ctrl_gt d_i_s_beta = smo->c1 * u_beta - smo->c2 * smo->i_s_est.dat[1] + smo->c3 * smo->psi_r_est.dat[1] -
                         smo->c4 * smo->omega_e_est * smo->psi_r_est.dat[0] - smo->z.dat[1];
    smo->i_s_est.dat[0] += d_i_s_alpha * smo->Ts;
    smo->i_s_est.dat[1] += d_i_s_beta * smo->Ts;

    // 3. Process the estimated rotor flux through the PLL
    // 3a. Calculate the PLL error signal. This is the cross product of the estimated
    //     flux vector and a unit vector aligned with the estimated angle.
    ctrl_gt pll_error = -smo->psi_r_est.dat[0] * sinf(smo->theta_est * CTL_CTRL_CONST_2_PI) +
                        smo->psi_r_est.dat[1] * cosf(smo->theta_est * CTL_CTRL_CONST_2_PI);

    // 3b. PI controller generates the estimated electrical speed (rad/s).
    smo->omega_e_est = ctl_step_pid_ser(&smo->pid_pll, pll_error);

    // 3c. Integrate the speed to get the next estimated angle.
    smo->theta_est += smo->omega_e_est * smo->Ts / CTL_CTRL_CONST_2_PI;
    smo->theta_est = ctrl_mod_1(smo->theta_est); // Wrap angle to [0, 1.0)

    // 4. Update the output interfaces
    // 4a. Calculate mechanical speed, filter it, and convert to p.u.
    ctrl_gt omega_m_est =
        (smo->omega_e_est - (smo->c5 * smo->i_s_est.dat[1] / smo->psi_r_est.dat[0])) / smo->pole_pairs;
    ctrl_gt speed_filtered = ctl_step_lowpass_filter(&smo->filter_spd, omega_m_est);
    smo->spdif.speed = speed_filtered * smo->speed_pu_sf;

    // 4b. Provide the estimated electrical and mechanical angles.
    smo->encif.elec_position = smo->theta_est;
    smo->encif.position = ctrl_mod_1(smo->theta_est / smo->pole_pairs);
}

/** @} */ // end of ACM_SMO group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ACM_SMO_H_
