/**
 * @file im.fo.h
 * @author Javnson (javnson@zju.edu.cn)
* @brief Implements the State-Feedback Flux & Speed Observer (FO) for Induction Motors.
 * @details This module utilizes a Gopinath-style closed-loop flux observer architecture.
 * It seamlessly combines a robust Current Model (accurate at zero/low speeds) with a 
 * Voltage Model (accurate at medium/high speeds) via PI compensators. It tightly 
 * integrates the Angle Tracking Observer (ATO) to extract synchronous speed and 
 * accurately calculates slip to estimate the mechanical rotor speed.
 * * **Core Mathematical Architecture (Per-Unitized):**
 * - **Current Model (Rotor Flux):** @f$ \Psi_{rd}[k] = \frac{\tau_r}{\tau_r + T_s} \Psi_{rd}[k-1] + \frac{L_m \cdot T_s}{\tau_r + T_s} i_{sd}[k] @f$
 * - **Current Model (Stator Flux):** @f$ \vec{\Psi}_{s\_ref} = \frac{L_m}{L_r} \vec{\Psi}_{r\_cm} + \sigma L_s \vec{I}_s @f$
 * - **Voltage Model (Back-EMF):** @f$ \vec{E} = \vec{V}_s - R_s \vec{I}_s - \vec{U}_{comp} @f$
 * - **Voltage Model (Stator Flux):** @f$ \vec{\Psi}_s[k] = \vec{\Psi}_s[k-1] + \vec{E}[k] \cdot (\Omega_{base} T_s) @f$
 * - **Slip Calculation:** @f$ \omega_{slip} = \frac{L_m}{\tau_r \cdot \Omega_{base}} \frac{\vec{\Psi}_r \times \vec{I}_s}{|\vec{\Psi}_r|^2} @f$
 *
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/consultant/acim_consultant.h>
#include <ctl/component/motor_control/consultant/pu_consultant.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/component/motor_control/observer/ato_pll.h>
#include <ctl/math_block/coordinate/coord_trans.h>
#include <ctl/math_block/vector_lite/vector2.h>

#ifndef _FILE_IM_FO_H_
#define _FILE_IM_FO_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup IM_FLUX_OBSERVER Induction Motor Flux and Torque Observer
 * @brief A module for estimating IM rotor flux and electromagnetic torque.
 *
 * This observer is based on the voltage model in the stationary (¦Á-¦Â) reference frame.
 * @details This module estimates the rotor flux and electromagnetic torque of an induction
 * motor using a voltage model. It integrates the stator back-EMF to find the stator flux,
 * then calculates the rotor flux and torque. A low-pass filter is used instead of a pure
 * integrator to mitigate DC drift issues at low speeds.
 * 
 * **Stator Flux Estimation (using a Low-Pass Filter):**
 * @f[ \psi_{s(\alpha\beta)} = \frac{1}{s + \omega_c}(V_{s(\alpha\beta)} - R_s I_{s(\alpha\beta)}) @f]
 * @f[ \vec{\psi}_s = \frac{1}{s + \omega_c}(\vec{V}_s - R_s \vec{I}_s) @f]
 *
 * **Rotor Flux Calculation:**
 * @f[ \psi_{r(\alpha\beta)} = \frac{L_r}{L_m}(\psi_{s(\alpha\beta)} - \sigma L_s I_{s(\alpha\beta)}) @f]
 * @f[ \vec{\psi}_r = \frac{L_r}{L_m}(\vec{\psi}_s - \sigma L_s \vec{I}_s) @f]
 * where @f[ \sigma = 1 - \frac{L_m^2}{L_s L_r} @f]
 *
 * **Torque Equation:**
 * @f[ T_e = \frac{3}{2}P \frac{L_m}{L_r} (\psi_{r\alpha} i_{s\beta} - \psi_{r\beta} i_{s\alpha}) @f]
 * @f[ T_e = \frac{3}{2}P \frac{L_m}{L_r} (\psi_{r\alpha} i_{s\beta} - \psi_{r\beta} i_{s\alpha}) @f]
 *
 */

/*---------------------------------------------------------------------------*/
/* Induction Motor State-Feedback Flux Observer (IM FO)                      */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup IM_FO Induction Motor State-Feedback Flux Observer
 * @brief Zero-lag flux and speed estimator using Gopinath composite architecture.
 * @{
 */

/**
 * @brief Raw initialization structure for the IM Flux Observer.
 * @details Contains explicitly derived scale factors (`sf_`) mapping physical equations to PU.
 */
typedef struct _tag_im_fo_init_t
{
    // --- Execution & Tuning Parameters ---
    parameter_gt fs;            //!< Controller execution frequency (@f$ f_s @f$) in Hz.
    parameter_gt ato_bw_hz;     //!< Tracking bandwidth for the ATO/PLL (Hz).
    parameter_gt fault_time_ms; //!< Divergence confirmation debounce time (ms).

    // --- Observer PI Compensator Gains (PU Space) ---
    parameter_gt
        kp_comp_pu; //!< Proportional gain for flux compensation (@f$ K_p \approx \frac{\omega_{comp}}{\Omega_{base}} @f$).
    parameter_gt
        ki_comp_pu; //!< Integral gain for flux compensation (@f$ K_i \approx \frac{\omega_{comp}^2}{\Omega_{base}} T_s @f$).
    parameter_gt u_comp_limit_pu; //!< Anti-windup limit for compensation voltage (PU).

    /** * @name Core Scale Factors (Derived from physical PU bases)
     * @brief Multipliers used in the fast discrete O(1) solver. */
    ///@{
    parameter_gt sf_cm_k1;      //!< Current model decay factor: @f$ \frac{\tau_r}{\tau_r + T_s} @f$
    parameter_gt sf_cm_k2;      //!< Current model integration factor: @f$ \frac{L_{m(pu)} \cdot T_s}{\tau_r + T_s} @f$
    parameter_gt sf_rs;         //!< Stator resistance PU: @f$ \frac{R_s}{Z_{s\_base}} @f$
    parameter_gt sf_lm_over_lr; //!< Inductance ratio: @f$ \frac{L_m}{L_r} @f$
    parameter_gt sf_sigma_ls;   //!< Transient Inductance PU: @f$ \frac{\sigma L_s}{L_{s\_base}} @f$
    parameter_gt sf_lr_over_lm; //!< Inverse inductance ratio: @f$ \frac{L_r}{L_m} @f$
    parameter_gt sf_v_int;      //!< Voltage integration factor mapping to PU flux: @f$ \Omega_{base} \cdot T_s @f$
    parameter_gt sf_slip_const; //!< Slip equation constant: @f$ \frac{L_{m(pu)}}{\tau_r \cdot \Omega_{base}} @f$
    parameter_gt sf_torque_const; //!< Torque constant magnitude: @f$ \frac{L_m}{L_r} @f$
    ///@}

} ctl_im_fo_init_t;

/**
 * @brief Main state structure for the IM FO controller.
 */
typedef struct _tag_im_fo_t
{
    // --- Standard Outputs (User Interfaces) ---
    rotation_ift pos_out; //!< Synchronous Flux Angle (Provides Rotor Field Orientation).
    velocity_ift spd_out; //!< Estimated Mechanical Rotor Speed.
    ctrl_gt torque_est;   //!< Estimated Electromagnetic Torque (PU).
    ctrl_gt psi_r_mag;    //!< Rotor Flux Magnitude (PU). Used for Field Weakening and slip division.

    // --- Core Sub-modules ---
    ctl_ato_pll_t ato_pll; //!< ATO tracks the Rotor Flux vector to extract synchronous speed and angle.
    ctl_pid_t pi_comp[2];  //!< PI Compensators bridging Current and Voltage models.

    // --- State Variables (PU) ---
    ctrl_gt psi_rd_cm;       //!< Rotor flux magnitude from Current Model (D-axis sync frame).
    ctl_vector2_t psi_s_ref; //!< Reference stator flux derived from the robust Current Model.
    ctl_vector2_t psi_s_est; //!< Estimated stator flux from Voltage Model integration.
    ctl_vector2_t psi_r_est; //!< Estimated rotor flux projected from Voltage Model.
    ctl_vector2_t phasor;    //!< Synchronous frame phasor [cos, sin].

    // --- Pre-calculated Scale Factors (Fixed-Point Mapping) ---
    ctrl_gt sf_cm_k1;
    ctrl_gt sf_cm_k2;
    ctrl_gt sf_rs;
    ctrl_gt sf_lm_over_lr;
    ctrl_gt sf_sigma_ls;
    ctrl_gt sf_lr_over_lm;
    ctrl_gt sf_v_int;
    ctrl_gt sf_slip_const;
    ctrl_gt sf_torque_const;

    // --- Safety Mechanism ---
    ctrl_gt flux_min_limit; //!< Minimum flux limit to validate PLL and slip calculation. Prevents div-by-zero.
    uint32_t diverge_cnt;   //!< Divergence debounce counter with anti-overflow saturation.
    uint32_t diverge_limit; //!< Divergence debounce limit (ticks).

    // --- Flags ---
    fast_gt flag_enable;          //!< Master enable flag for the observer execution.
    fast_gt flag_observer_locked; //!< Status flag: 1 if tracking is stable, 0 if observer has diverged.

} ctl_im_fo_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

/**
 * @brief Core initialization function using the explicit scale factors.
 * @param[out] fo   Pointer to the FO instance to be initialized.
 * @param[in]  init Pointer to the raw initialization structure.
 */
void ctl_init_im_fo(ctl_im_fo_t* fo, const ctl_im_fo_init_t* init);

/**
 * @brief Advanced initialization function utilizing the IM Consultant models.
 * @details Automatically calculates all scale factors and tunes the Gopinath 
 * crossover PI compensator based on the target observer bandwidth.
 * @param[out] fo            Pointer to the FO instance.
 * @param[in]  motor         Pointer to the IM physical model consultant.
 * @param[in]  pu            Pointer to the Per-Unit base model consultant.
 * @param[in]  fs            Controller execution frequency (Hz).
 * @param[in]  comp_bw_hz    Crossover frequency between Current/Voltage models (Hz).
 * @param[in]  ato_bw_hz     Target tracking bandwidth for the ATO/PLL (Hz).
 * @param[in]  fault_time_ms Divergence confirmation debounce time (ms).
 */
void ctl_init_im_fo_consultant(ctl_im_fo_t* fo, const ctl_consultant_im_t* motor, const ctl_consultant_pu_im_t* pu,
                               parameter_gt fs, parameter_gt comp_bw_hz, parameter_gt ato_bw_hz,
                               parameter_gt fault_time_ms);

/**
 * @brief Safely clears all history states, integrators, and health flags.
 * @details Does not alter the enable/disable state. Ensures a bumpless restart.
 * @param[in,out] fo Pointer to the FO instance.
 */
GMP_STATIC_INLINE void ctl_clear_im_fo(ctl_im_fo_t* fo)
{
    fo->psi_rd_cm = float2ctrl(0.0f);
    ctl_vector2_clear(&fo->psi_s_ref);
    ctl_vector2_clear(&fo->psi_s_est);
    ctl_vector2_clear(&fo->psi_r_est);

    ctl_clear_pid(&fo->pi_comp[0]);
    ctl_clear_pid(&fo->pi_comp[1]);
    ctl_clear_ato_pll(&fo->ato_pll);

    fo->diverge_cnt = 0;
    fo->flag_observer_locked = 0;
    fo->pos_out.elec_position = float2ctrl(0.0f);
    fo->spd_out.speed = float2ctrl(0.0f);
    fo->torque_est = float2ctrl(0.0f);
    fo->psi_r_mag = float2ctrl(0.0f);
}

/**
 * @brief Enables the IM FO execution.
 */
GMP_STATIC_INLINE void ctl_enable_im_fo(ctl_im_fo_t* fo)
{
    fo->flag_enable = 1;
}

/**
 * @brief Disables the IM FO execution.
 */
GMP_STATIC_INLINE void ctl_disable_im_fo(ctl_im_fo_t* fo)
{
    fo->flag_enable = 0;
}

/**
 * @brief Executes one high-frequency step of the IM Flux Observer.
 * @details Runs the Gopinath Composite Flux Estimator, computes the Slip frequency, 
 * and extracts mechanical speed via the PLL.
 * @param[in,out] fo      Pointer to the FO instance.
 * @param[in]     v_alpha Applied alpha-axis stator voltage (PU).
 * @param[in]     v_beta  Applied beta-axis stator voltage (PU).
 * @param[in]     i_alpha Measured alpha-axis stator current (PU).
 * @param[in]     i_beta  Measured beta-axis stator current (PU).
 */
GMP_STATIC_INLINE void ctl_step_im_fo(ctl_im_fo_t* fo, ctrl_gt v_alpha, ctrl_gt v_beta, ctrl_gt i_alpha, ctrl_gt i_beta)
{
    if (!fo->flag_enable)
        return;

    // Retrieve previous synchronous angle for coordinate transforms
    ctrl_gt theta_sync = fo->ato_pll.elec_angle_pu;
    ctl_set_phasor_via_angle(theta_sync, &fo->phasor);
    ctrl_gt cos_t = fo->phasor.dat[0];
    ctrl_gt sin_t = fo->phasor.dat[1];

    // ========================================================================
    // 1. Current Model (Low-Speed Authority)
    // ========================================================================
    // Extract magnetizing current (D-axis) in synchronous frame
    ctrl_gt i_sd = ctl_mul(i_alpha, cos_t) + ctl_mul(i_beta, sin_t);

    // 1st order LPF to build rotor flux: Psi_rd = sf_cm_k1 * Psi_rd + sf_cm_k2 * i_sd
    fo->psi_rd_cm = ctl_mul(fo->sf_cm_k1, fo->psi_rd_cm) + ctl_mul(fo->sf_cm_k2, i_sd);

    // Project Current Model Rotor Flux back to alpha-beta frame
    ctrl_gt psi_r_alpha_cm = ctl_mul(fo->psi_rd_cm, cos_t);
    ctrl_gt psi_r_beta_cm = ctl_mul(fo->psi_rd_cm, sin_t);

    // Calculate Reference Stator Flux from Current Model
    // Psi_s_ref = (Lm/Lr)*Psi_r_cm + (sigma*Ls)*Is
    fo->psi_s_ref.dat[0] = ctl_mul(fo->sf_lm_over_lr, psi_r_alpha_cm) + ctl_mul(fo->sf_sigma_ls, i_alpha);
    fo->psi_s_ref.dat[1] = ctl_mul(fo->sf_lm_over_lr, psi_r_beta_cm) + ctl_mul(fo->sf_sigma_ls, i_beta);

    // ========================================================================
    // 2. PI Compensator (Bridging Current & Voltage Models)
    // ========================================================================
    // Error = Voltage Model Flux - Current Model Flux
    ctrl_gt err_psi_alpha = fo->psi_s_est.dat[0] - fo->psi_s_ref.dat[0];
    ctrl_gt err_psi_beta = fo->psi_s_est.dat[1] - fo->psi_s_ref.dat[1];

    ctrl_gt u_comp_alpha = ctl_step_pid_par(&fo->pi_comp[0], err_psi_alpha);
    ctrl_gt u_comp_beta = ctl_step_pid_par(&fo->pi_comp[1], err_psi_beta);

    // ========================================================================
    // 3. Voltage Model (High-Speed Authority)
    // ========================================================================
    // Back-EMF = Vs - Rs*Is - U_comp
    ctrl_gt bemf_alpha = v_alpha - ctl_mul(fo->sf_rs, i_alpha) - u_comp_alpha;
    ctrl_gt bemf_beta = v_beta - ctl_mul(fo->sf_rs, i_beta) - u_comp_beta;

    // Integrate Back-EMF to get Stator Flux: Psi_s[k] = Psi_s[k-1] + BEMF * (W_base * Ts)
    fo->psi_s_est.dat[0] += ctl_mul(bemf_alpha, fo->sf_v_int);
    fo->psi_s_est.dat[1] += ctl_mul(bemf_beta, fo->sf_v_int);

    // ========================================================================
    // 4. Rotor Flux Estimation
    // ========================================================================
    // Psi_r = (Lr/Lm) * (Psi_s_est - sigma*Ls*Is)
    ctrl_gt diff_alpha = fo->psi_s_est.dat[0] - ctl_mul(fo->sf_sigma_ls, i_alpha);
    ctrl_gt diff_beta = fo->psi_s_est.dat[1] - ctl_mul(fo->sf_sigma_ls, i_beta);

    fo->psi_r_est.dat[0] = ctl_mul(fo->sf_lr_over_lm, diff_alpha);
    fo->psi_r_est.dat[1] = ctl_mul(fo->sf_lr_over_lm, diff_beta);

    // Magnitude Calculation: |Psi_r|^2
    ctrl_gt psi_r_sq =
        ctl_mul(fo->psi_r_est.dat[0], fo->psi_r_est.dat[0]) + ctl_mul(fo->psi_r_est.dat[1], fo->psi_r_est.dat[1]);
    fo->psi_r_mag = ctl_sqrt(psi_r_sq);

    // ========================================================================
    // 5. Health Assessment & Protection
    // ========================================================================
    // The observer diverges if the rotor flux collapses or explodes pathologically.
    if ((fo->psi_r_mag < fo->flux_min_limit) || (fo->psi_r_mag > float2ctrl(1.5f)))
    {
        if (fo->diverge_cnt < fo->diverge_limit)
            fo->diverge_cnt++;
        if (fo->diverge_cnt >= fo->diverge_limit)
            fo->flag_observer_locked = 0;
    }
    else
    {
        if (fo->diverge_cnt > 0)
            fo->diverge_cnt--;
        if (fo->diverge_cnt == 0)
            fo->flag_observer_locked = 1;
    }

    // ========================================================================
    // 6. ATO/PLL Tracking (Synchronous Speed & Angle)
    // ========================================================================
    // Cross product error for PLL: -Psi_r_alpha * sin + Psi_r_beta * cos
    ctrl_gt pll_err = -ctl_mul(fo->psi_r_est.dat[0], sin_t) + ctl_mul(fo->psi_r_est.dat[1], cos_t);

    // Normalize error by flux magnitude for consistent PLL bandwidth
    if (fo->psi_r_mag > float2ctrl(0.01f))
    {
        pll_err = ctl_div(pll_err, fo->psi_r_mag);
    }
    else
    {
        pll_err = float2ctrl(0.0f);
    }

    pll_err = ctl_mul(pll_err, CTL_CTRL_CONST_1_OVER_2PI);

    ctl_step_ato_pll(&fo->ato_pll, pll_err);

    // ========================================================================
    // 7. Slip & Mechanical Speed & Torque Calculation
    // ========================================================================
    // Cross product: (Psi_r_alpha * I_beta - Psi_r_beta * I_alpha)
    ctrl_gt cross_flux_i = ctl_mul(fo->psi_r_est.dat[0], i_beta) - ctl_mul(fo->psi_r_est.dat[1], i_alpha);

    // Slip = sf_slip_const * (Psi_r x I_s) / |Psi_r|^2
    ctrl_gt w_slip_pu = float2ctrl(0.0f);
    if (psi_r_sq > float2ctrl(0.001f))
    {
        w_slip_pu = ctl_mul(fo->sf_slip_const, ctl_div(cross_flux_i, psi_r_sq));
    }

    // Mechanical Speed = Synchronous Speed - Slip Speed
    ctrl_gt w_mech_pu = fo->ato_pll.elec_speed_pu - w_slip_pu;

    // Torque = sf_torque_const * (Psi_r x I_s)
    fo->torque_est = ctl_mul(fo->sf_torque_const, cross_flux_i);

    // ========================================================================
    // 8. Output to Top-Level Interfaces
    // ========================================================================
    fo->pos_out.elec_position = fo->ato_pll.elec_angle_pu;
    fo->spd_out.speed = w_mech_pu; // Output mechanical speed for outer velocity loop
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_IM_FO_H_
