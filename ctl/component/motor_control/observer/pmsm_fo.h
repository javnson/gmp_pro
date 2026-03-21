/**
 * @file pmsm_fo.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements the State-Feedback Flux Observer (FO) for PMSM.
 * @details This module utilizes standard PID controllers acting as Extended 
 * State Observers (ESO) to directly estimate the Back-EMF. By employing a 
 * linear state-feedback tracking law, it eliminates the need for low-pass 
 * filtering on the EMF, achieving naturally zero-phase-lag estimation. It 
 * strictly adheres to the Per-Unit (PU) normalization and scale factor (sf_) 
 * naming conventions.
 * @version 1.2
 * @date 2024-10-02
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_PMSM_FO_H_
#define _FILE_PMSM_FO_H_

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

/**
 * @defgroup PMSM_FLUX_OBSERVER PMSM Flux and Torque Observer
 * @brief A module for estimating PMSM stator flux and electromagnetic torque.
 *
 * @details This module calculates the stator flux and electromagnetic torque based on the motor's
 * currents and rotor angle. It uses a sensored model, meaning it requires rotor position feedback.
 * The calculations are performed in the stationary ¦Á-¦Â reference frame using per-unit values.

 * This observer uses the following per-unit equations:
 *
 * **Flux Equations:**
 * @f[ \psi_\alpha^* = \omega_b L_s^* I_\alpha^* + \psi_{PM}^* \cos\theta @f]
 * @f[ \psi_\alpha^* = \omega_b L_s^* I_\alpha^* + \psi_{PM}^* \cos\theta @f]
 * @f[ \psi_\beta^* = \omega_b L_s^* I_\beta^* + \psi_{PM}^* \sin\theta @f]
 * @f[ \psi_\beta^* = \omega_b L_s^* I_\beta^* + \psi_{PM}^* \sin\theta @f]
 * @f[ \psi^* = (\psi_\alpha^{*\,2} + \psi_\beta^{*\,2})^{\frac{1}{2}} @f]
 * @f[ \psi^* = \sqrt{(\psi_\alpha^*)^2 + (\psi_\beta^*)^2} @f]
 *
 * **Torque Equation:**
 * @f[ T^* = \frac{1}{\psi_{PM}^*} (\psi_\alpha^* i_\beta^* - \psi_\beta^* i_\alpha^* ) @f]
 * @f[ T^* = \frac{1}{\psi_{PM}^*} (\psi_\alpha^* i_\beta^* - \psi_\beta^* i_\alpha^*) @f]
 *
 */

/*---------------------------------------------------------------------------*/
/* PMSM State-Feedback Flux Observer (FO)                                    */
/*---------------------------------------------------------------------------*/

/**
 * @brief Raw initialization structure for the State-Feedback Flux Observer.
 * @details Used when supplying bare physical parameters directly rather than 
 * using the high-level Consultant objects.
 */
typedef struct _tag_pmsm_fo_init_t
{
    // --- Motor Physical Parameters ---
    parameter_gt Rs; //!< Stator phase resistance (Ohm).
    parameter_gt Ld; //!< D-axis synchronous inductance (H).
    parameter_gt Lq; //!< Q-axis synchronous inductance (H).

    // --- Per-Unit Base Values ---
    parameter_gt V_base; //!< Base Phase Voltage Peak (V).
    parameter_gt I_base; //!< Base Phase Current Peak (A).
    parameter_gt W_base; //!< Base Electrical Angular Velocity (rad/s).

    // --- Execution & Tuning Parameters ---
    parameter_gt fs;            //!< Controller execution frequency (Hz).
    parameter_gt ato_bw_hz;     //!< Tracking bandwidth for the ATO/PLL (Hz).
    parameter_gt fault_time_ms; //!< Divergence confirmation debounce time (ms).

    // --- Observer PI Gains (PU Space) ---
    parameter_gt kp_fo_pu; //!< Proportional gain for EMF state feedback (PU).
    parameter_gt ki_fo_pu; //!< Integral gain for EMF state feedback (PU, absorbs Ts).

    // --- Margins & Limits ---
    parameter_gt current_err_limit_pu; //!< Max absolute current tracking error (PU) before triggering divergence.
    parameter_gt e_max_limit_pu;       //!< Anti-windup saturation limit for estimated Back-EMF (PU).

} ctl_pmsm_fo_init_t;

/**
 * @brief Main state structure for the PMSM FO controller.
 * @details Encapsulates all state variables, PID trackers, pre-calculated constants, 
 * scale factors, and safety mechanisms required for the observer.
 */
typedef struct _tag_pmsm_fo_t
{
    // --- Standard Outputs (User Interfaces) ---
    rotation_ift pos_out; //!< Standard position interface providing estimated electrical angle.
    velocity_ift spd_out; //!< Standard velocity interface providing estimated electrical speed.

    // --- Core Sub-modules ---
    ctl_ato_pll_t ato_pll; //!< Angle Tracking Observer (Software PLL) for zero-lag tracking.
    ctl_pid_t pi_emf[2];   //!< Standard PIDs acting as Extended State Observers for Back-EMF (alpha, beta).

    // --- State Variables ---
    ctl_vector2_t i_est;  //!< Estimated stator current vector [alpha, beta] (PU).
    ctl_vector2_t e_est;  //!< Directly estimated Back-EMF vector [alpha, beta] (PU).
    ctl_vector2_t phasor; //!< Estimated rotor phasor [cos(theta), sin(theta)].

    // --- Pre-calculated Physical Constants (Plant Model) ---
    ctrl_gt k1; //!< Voltage integration coefficient: (Ts * V_base) / (Ld * I_base).
    ctrl_gt k2; //!< Current decay coefficient: (Rs * Ts) / Ld.
    ctrl_gt k3; //!< Saliency cross-coupling coefficient: (Ld - Lq) / Ld.

    // --- Scale Factors ---
    ctrl_gt sf_w_to_rad_tick; //!< Scale factor: converts PU speed to rad/tick for cross-coupling.

    // --- Safety Mechanism ---
    ctrl_gt current_err_limit; //!< Maximum allowed absolute current tracking error (PU).
    uint32_t diverge_cnt;      //!< Divergence debounce counter with anti-overflow saturation.
    uint32_t diverge_limit;    //!< Divergence debounce limit (ticks) to confirm Loss-of-Lock.

    // --- Flags ---
    fast_gt flag_enable;          //!< Master enable flag for the observer execution.
    fast_gt flag_observer_locked; //!< Status flag: 1 if tracking is stable, 0 if observer has diverged.

} ctl_pmsm_fo_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

/**
 * @brief Core initialization function using the bare physical parameters.
 * @param[out] fo   Pointer to the FO instance to be initialized.
 * @param[in]  init Pointer to the raw initialization structure.
 */
void ctl_init_pmsm_fo(ctl_pmsm_fo_t* fo, const ctl_pmsm_fo_init_t* init);

/**
 * @brief Advanced initialization function utilizing the Consultant models.
 * @details Automatically calculates the optimal state-feedback PI gains based on 
 * the target observer bandwidth using Luenberger pole placement, eliminating 
 * complex manual tuning.
 * @param[out] fo            Pointer to the FO instance.
 * @param[in]  motor         Pointer to the PMSM physical model consultant.
 * @param[in]  pu            Pointer to the Per-Unit base model consultant.
 * @param[in]  fs            Controller execution frequency (Hz).
 * @param[in]  obs_bw_hz     Target tracking bandwidth for the State-Feedback observer (Hz).
 * @param[in]  ato_bw_hz     Target tracking bandwidth for the ATO/PLL (Hz).
 * @param[in]  fault_time_ms Divergence confirmation debounce time (ms).
 */
void ctl_init_pmsm_fo_consultant(ctl_pmsm_fo_t* fo, const ctl_consultant_pmsm_t* motor,
                                 const ctl_consultant_pu_pmsm_t* pu, parameter_gt fs, parameter_gt obs_bw_hz,
                                 parameter_gt ato_bw_hz, parameter_gt fault_time_ms);

/**
 * @brief Safely clears all history states, integrators, and health flags.
 * @details Does not alter the enable/disable state. Ensures a bumpless restart 
 * when switching from closed-loop sensor mode to sensorless mode.
 * @param[in,out] fo Pointer to the FO instance.
 */
GMP_STATIC_INLINE void ctl_clear_pmsm_fo(ctl_pmsm_fo_t* fo)
{
    ctl_vector2_clear(&fo->i_est);
    ctl_vector2_clear(&fo->e_est);

    ctl_clear_pid(&fo->pi_emf[0]);
    ctl_clear_pid(&fo->pi_emf[1]);
    ctl_clear_ato_pll(&fo->ato_pll);

    fo->diverge_cnt = 0;
    fo->flag_observer_locked = 0;
    fo->pos_out.elec_position = float2ctrl(0.0f);
    fo->spd_out.speed = float2ctrl(0.0f);
}

/**
 * @brief Enables the FO execution.
 * @param[in,out] fo Pointer to the FO instance.
 */
GMP_STATIC_INLINE void ctl_enable_pmsm_fo(ctl_pmsm_fo_t* fo)
{
    fo->flag_enable = 1;
}

/**
 * @brief Disables the FO execution.
 * @param[in,out] fo Pointer to the FO instance.
 */
GMP_STATIC_INLINE void ctl_disable_pmsm_fo(ctl_pmsm_fo_t* fo)
{
    fo->flag_enable = 0;
}

/**
 * @brief Executes one high-frequency step of the State-Feedback Flux Observer.
 * @details Solves the physics-accurate difference equations, applies the PI 
 * state-feedback tracking law to estimate Back-EMF, assesses tracking health, 
 * and updates the Angle Tracking Observer (ATO).
 * @param[in,out] fo      Pointer to the FO instance.
 * @param[in]     v_alpha Applied alpha-axis stator voltage (PU).
 * @param[in]     v_beta  Applied beta-axis stator voltage (PU).
 * @param[in]     i_alpha Measured alpha-axis stator current (PU).
 * @param[in]     i_beta  Measured beta-axis stator current (PU).
 */
GMP_STATIC_INLINE void ctl_step_pmsm_fo(ctl_pmsm_fo_t* fo, ctrl_gt v_alpha, ctrl_gt v_beta, ctrl_gt i_alpha,
                                        ctrl_gt i_beta)
{
    if (!fo->flag_enable)
        return;

    // ========================================================================
    // 1. Current Estimation Plant (Difference Equations)
    // ========================================================================
    ctrl_gt wr_tick = ctl_mul(fo->ato_pll.elec_speed_pu, fo->sf_w_to_rad_tick);
    ctrl_gt cross_term = ctl_mul(wr_tick, fo->k3);

    // E_est (calculated in previous tick) acts directly as the disturbance voltage
    ctrl_gt delta_i_alpha = ctl_mul(fo->k1, v_alpha - fo->e_est.dat[0]) - ctl_mul(fo->k2, fo->i_est.dat[0]) -
                            ctl_mul(cross_term, fo->i_est.dat[1]);

    ctrl_gt delta_i_beta = ctl_mul(fo->k1, v_beta - fo->e_est.dat[1]) - ctl_mul(fo->k2, fo->i_est.dat[1]) +
                           ctl_mul(cross_term, fo->i_est.dat[0]);

    fo->i_est.dat[0] += delta_i_alpha;
    fo->i_est.dat[1] += delta_i_beta;

    // ========================================================================
    // 2. State-Feedback Tracking Law (PI tracking of Back-EMF)
    // ========================================================================
    ctrl_gt err_alpha = fo->i_est.dat[0] - i_alpha;
    ctrl_gt err_beta = fo->i_est.dat[1] - i_beta;

    // The PID outputs are precisely the estimated Back-EMF components!
    // Saturation and anti-windup are elegantly handled inside the generic PID module.
    fo->e_est.dat[0] = ctl_step_pid_par(&fo->pi_emf[0], err_alpha);
    fo->e_est.dat[1] = ctl_step_pid_par(&fo->pi_emf[1], err_beta);

    // ========================================================================
    // 3. Observer Health Assessment (Loss-of-Lock Protection)
    // ========================================================================
    ctrl_gt abs_err_alpha = (err_alpha > float2ctrl(0.0f)) ? err_alpha : -err_alpha;
    ctrl_gt abs_err_beta = (err_beta > float2ctrl(0.0f)) ? err_beta : -err_beta;

    // Fast O(1) Absolute Threshold Check for divergence
    if ((abs_err_alpha > fo->current_err_limit) || (abs_err_beta > fo->current_err_limit))
    {
        if (fo->diverge_cnt < fo->diverge_limit)
        {
            fo->diverge_cnt++;
        }
        if (fo->diverge_cnt >= fo->diverge_limit)
        {
            fo->flag_observer_locked = 0;
        }
    }
    else
    {
        if (fo->diverge_cnt > 0)
        {
            fo->diverge_cnt--;
        }
        if (fo->diverge_cnt == 0)
        {
            fo->flag_observer_locked = 1;
        }
    }

    // ========================================================================
    // 4. Phase Error Generation & PLL Tracking
    // ========================================================================
    ctl_set_phasor_via_angle(fo->ato_pll.elec_angle_pu, &fo->phasor);

    // Raw voltage tracking error. Gain scheduling is handled externally.
    ctrl_gt e_err_voltage =
        -ctl_mul(fo->e_est.dat[0], fo->phasor.dat[1]) + ctl_mul(fo->e_est.dat[1], fo->phasor.dat[0]);

    ctl_step_ato_pll(&fo->ato_pll, e_err_voltage);

    // NOTE: Zero phase compensation needed! The PI state-feedback naturally
    // aligns the estimated EMF with the physical reality without LPF lag.

    // ========================================================================
    // 5. Output to Top-Level Interfaces
    // ========================================================================
    fo->pos_out.elec_position = fo->ato_pll.elec_angle_pu; // Already modulo'd inside ATO
    fo->spd_out.speed = fo->ato_pll.elec_speed_pu;
}
/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_FO_H_
