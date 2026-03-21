/**
 * @file acm_smo.h
 * @brief Implements the Sliding Mode Flux & Speed Observer (SMO) for Induction Motors.
 * @details This module utilizes a Gopinath-style closed-loop architecture, but upgrades 
 * the traditional PI crossover compensator to a robust Sliding Mode controller. 
 * By applying a saturation function and a low-pass filter to the flux estimation error, 
 * it forces the high-speed Voltage Model to rigorously track the low-speed Current Model, 
 * providing extreme robustness against parameter variations (especially stator resistance).
 * * * **Core Mathematical Architecture (Per-Unitized):**
 * - **Current Model (Rotor Flux):** @f$ \Psi_{rd}[k] = \frac{\tau_r}{\tau_r + T_s} \Psi_{rd}[k-1] + \frac{L_m \cdot T_s}{\tau_r + T_s} i_{sd}[k] @f$
 * - **Current Model (Stator Flux):** @f$ \vec{\Psi}_{s\_ref} = \frac{L_m}{L_r} \vec{\Psi}_{r\_cm} + \sigma L_s \vec{I}_s @f$
 * - **Sliding Mode Compensator:** @f$ \vec{Z}_{comp} = K_{slide} \cdot \text{sat}(\vec{\Psi}_{s\_est} - \vec{\Psi}_{s\_ref}) @f$
 * - **Voltage Compensation:** @f$ \vec{U}_{comp} = LPF(\vec{Z}_{comp}) @f$
 * - **Voltage Model (Back-EMF):** @f$ \vec{E} = \vec{V}_s - R_s \vec{I}_s - \vec{U}_{comp} @f$
 * - **Voltage Model (Stator Flux):** @f$ \vec{\Psi}_s[k] = \vec{\Psi}_s[k-1] + \vec{E}[k] \cdot (\Omega_{base} T_s) @f$
 * 
 * @version 1.0
 * @date 2025-08-07
 *
 */

#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/consultant/im_consultant.h>
#include <ctl/component/motor_control/consultant/pu_consultant.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/component/motor_control/observer/ato_pll.h>
#include <ctl/math_block/coordinate/coord_trans.h>
#include <ctl/math_block/vector_lite/vector2.h>

#ifndef _FILE_ACM_SMO_H_
#define _FILE_ACM_SMO_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Induction Motor Sliding Mode Flux Observer (IM SMO)                       */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup IM_SMO Induction Motor Sliding Mode Flux Observer
 * @brief Robust composite flux estimator replacing PI with SMO + LPF.
 * @{
 */

/**
 * @brief Raw initialization structure for the IM SMO.
 * @details Contains explicit scale factors and sliding margins mapping physics to PU.
 */
typedef struct _tag_im_smo_init_t
{
    // --- Execution & Tuning Parameters ---
    parameter_gt fs;            //!< Controller execution frequency (@f$ f_s @f$) in Hz.
    parameter_gt ato_bw_hz;     //!< Tracking bandwidth for the ATO/PLL (Hz).
    parameter_gt fault_time_ms; //!< Divergence confirmation debounce time (ms).

    // --- Sliding Mode Compensator Gains ---
    parameter_gt k_slide_pu;  //!< Sliding gain for flux error compensation (PU).
    parameter_gt z_margin_pu; //!< Boundary layer margin for the continuous saturation function.
    parameter_gt fc_comp_hz;  //!< Cutoff frequency for the LPF smoothing the sliding control effort (Hz).

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

} ctl_im_smo_init_t;

/**
 * @brief Main state structure for the IM SMO controller.
 */
typedef struct _tag_im_smo_t
{
    // --- Standard Outputs (User Interfaces) ---
    rotation_ift pos_out; //!< Synchronous Flux Angle.
    velocity_ift spd_out; //!< Estimated Mechanical Rotor Speed.
    ctrl_gt torque_est;   //!< Estimated Electromagnetic Torque (PU).
    ctrl_gt psi_r_mag;    //!< Rotor Flux Magnitude (PU).

    // --- Core Sub-modules ---
    ctl_ato_pll_t ato_pll;            //!< ATO tracks the Rotor Flux vector.
    ctl_filter_IIR1_t filter_comp[2]; //!< LPFs acting as the integral part of the SMO compensator.

    // --- State Variables (PU) ---
    ctrl_gt psi_rd_cm;       //!< Rotor flux magnitude from Current Model.
    ctl_vector2_t psi_s_ref; //!< Reference stator flux derived from the Current Model.
    ctl_vector2_t psi_s_est; //!< Estimated stator flux from Voltage Model integration.
    ctl_vector2_t psi_r_est; //!< Estimated rotor flux projected from Voltage Model.
    ctl_vector2_t z_comp;    //!< Raw sliding mode control effort (switch function output).
    ctl_vector2_t u_comp;    //!< Filtered compensation voltage bridging the models.
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

    // --- Sliding Mode Parameters ---
    ctrl_gt k_slide;         //!< Sliding gain.
    ctrl_gt z_margin;        //!< Saturation margin.
    ctrl_gt sf_z_margin_inv; //!< Pre-calculated inverse of the margin (1 / z_margin).

    // --- Safety Mechanism ---
    ctrl_gt flux_min_limit; //!< Minimum flux limit to validate PLL.
    uint32_t diverge_cnt;
    uint32_t diverge_limit;

    // --- Flags ---
    fast_gt flag_enable;
    fast_gt flag_observer_locked;

} ctl_im_smo_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

/**
 * @brief Core initialization function using the explicit scale factors.
 */
void ctl_init_im_smo(ctl_im_smo_t* smo, const ctl_im_smo_init_t* init);

/**
 * @brief Advanced initialization function utilizing the IM Consultant models.
 * @details Automatically calculates scale factors and configures the SMO compensator 
 * to bridge the current and voltage models.
 */
void ctl_init_im_smo_consultant(ctl_im_smo_t* smo, const ctl_consultant_im_t* motor, const ctl_consultant_pu_im_t* pu,
                                parameter_gt fs, parameter_gt comp_bw_hz, parameter_gt ato_bw_hz,
                                parameter_gt fault_time_ms);

/**
 * @brief Safely clears all history states, integrators, and health flags.
 */
GMP_STATIC_INLINE void ctl_clear_im_smo(ctl_im_smo_t* smo)
{
    smo->psi_rd_cm = float2ctrl(0.0f);
    ctl_vector2_clear(&smo->psi_s_ref);
    ctl_vector2_clear(&smo->psi_s_est);
    ctl_vector2_clear(&smo->psi_r_est);
    ctl_vector2_clear(&smo->z_comp);
    ctl_vector2_clear(&smo->u_comp);

    ctl_clear_filter_iir1(&smo->filter_comp[0]);
    ctl_clear_filter_iir1(&smo->filter_comp[1]);
    ctl_clear_ato_pll(&smo->ato_pll);

    smo->diverge_cnt = 0;
    smo->flag_observer_locked = 0;
    smo->pos_out.elec_position = float2ctrl(0.0f);
    smo->spd_out.velocity = float2ctrl(0.0f);
    smo->torque_est = float2ctrl(0.0f);
    smo->psi_r_mag = float2ctrl(0.0f);
}

GMP_STATIC_INLINE void ctl_enable_im_smo(ctl_im_smo_t* smo)
{
    smo->flag_enable = 1;
}
GMP_STATIC_INLINE void ctl_disable_im_smo(ctl_im_smo_t* smo)
{
    smo->flag_enable = 0;
}

/**
 * @brief Executes one high-frequency step of the IM Sliding Mode Observer.
 * @details Seamlessly blends the low-speed current model with the high-speed voltage 
 * model using a robust sliding mode compensator, then extracts speed via the PLL.
 */
GMP_STATIC_INLINE void ctl_step_im_smo(ctl_im_smo_t* smo, ctrl_gt v_alpha, ctrl_gt v_beta, ctrl_gt i_alpha,
                                       ctrl_gt i_beta)
{
    if (!smo->flag_enable)
        return;

    ctrl_gt theta_sync = smo->ato_pll.elec_angle_pu;
    ctl_set_phasor_via_angle(theta_sync, &smo->phasor);
    ctrl_gt cos_t = smo->phasor.dat[0];
    ctrl_gt sin_t = smo->phasor.dat[1];

    // ========================================================================
    // 1. Current Model (Low-Speed Authority)
    // ========================================================================
    ctrl_gt i_sd = ctl_mul(i_alpha, cos_t) + ctl_mul(i_beta, sin_t);

    smo->psi_rd_cm = ctl_mul(smo->sf_cm_k1, smo->psi_rd_cm) + ctl_mul(smo->sf_cm_k2, i_sd);

    ctrl_gt psi_r_alpha_cm = ctl_mul(smo->psi_rd_cm, cos_t);
    ctrl_gt psi_r_beta_cm = ctl_mul(smo->psi_rd_cm, sin_t);

    smo->psi_s_ref.dat[0] = ctl_mul(smo->sf_lm_over_lr, psi_r_alpha_cm) + ctl_mul(smo->sf_sigma_ls, i_alpha);
    smo->psi_s_ref.dat[1] = ctl_mul(smo->sf_lm_over_lr, psi_r_beta_cm) + ctl_mul(smo->sf_sigma_ls, i_beta);

    // ========================================================================
    // 2. Sliding Mode Compensator (Replaces the PI from FO)
    // ========================================================================
    ctrl_gt err_psi_alpha = smo->psi_s_est.dat[0] - smo->psi_s_ref.dat[0];
    ctrl_gt err_psi_beta = smo->psi_s_est.dat[1] - smo->psi_s_ref.dat[1];

    // Continuous Saturation Function to eliminate chattering
    ctrl_gt sat_alpha = ctl_sat(err_psi_alpha, smo->z_margin, -smo->z_margin);
    ctrl_gt sat_beta = ctl_sat(err_psi_beta, smo->z_margin, -smo->z_margin);

    // Sliding Control Effort (Raw Switch Function)
    smo->z_comp.dat[0] = ctl_mul(smo->k_slide, ctl_mul(sat_alpha, smo->sf_z_margin_inv));
    smo->z_comp.dat[1] = ctl_mul(smo->k_slide, ctl_mul(sat_beta, smo->sf_z_margin_inv));

    // Low-Pass Filter extracts the equivalent control (Compensation Voltage)
    smo->u_comp.dat[0] = ctl_step_filter_iir1(&smo->filter_comp[0], smo->z_comp.dat[0]);
    smo->u_comp.dat[1] = ctl_step_filter_iir1(&smo->filter_comp[1], smo->z_comp.dat[1]);

    // ========================================================================
    // 3. Voltage Model (High-Speed Authority)
    // ========================================================================
    ctrl_gt bemf_alpha = v_alpha - ctl_mul(smo->sf_rs, i_alpha) - smo->u_comp.dat[0];
    ctrl_gt bemf_beta = v_beta - ctl_mul(smo->sf_rs, i_beta) - smo->u_comp.dat[1];

    // Integrate Back-EMF to get Estimated Stator Flux
    smo->psi_s_est.dat[0] += ctl_mul(bemf_alpha, smo->sf_v_int);
    smo->psi_s_est.dat[1] += ctl_mul(bemf_beta, smo->sf_v_int);

    // ========================================================================
    // 4. Rotor Flux Estimation
    // ========================================================================
    ctrl_gt diff_alpha = smo->psi_s_est.dat[0] - ctl_mul(smo->sf_sigma_ls, i_alpha);
    ctrl_gt diff_beta = smo->psi_s_est.dat[1] - ctl_mul(smo->sf_sigma_ls, i_beta);

    smo->psi_r_est.dat[0] = ctl_mul(smo->sf_lr_over_lm, diff_alpha);
    smo->psi_r_est.dat[1] = ctl_mul(smo->sf_lr_over_lm, diff_beta);

    ctrl_gt psi_r_sq =
        ctl_mul(smo->psi_r_est.dat[0], smo->psi_r_est.dat[0]) + ctl_mul(smo->psi_r_est.dat[1], smo->psi_r_est.dat[1]);
    smo->psi_r_mag = ctl_sqrt(psi_r_sq);

    // ========================================================================
    // 5. Health Assessment & Protection
    // ========================================================================
    if ((smo->psi_r_mag < smo->flux_min_limit) || (smo->psi_r_mag > float2ctrl(1.5f)))
    {
        if (smo->diverge_cnt < smo->diverge_limit)
            smo->diverge_cnt++;
        if (smo->diverge_cnt >= smo->diverge_limit)
            smo->flag_observer_locked = 0;
    }
    else
    {
        if (smo->diverge_cnt > 0)
            smo->diverge_cnt--;
        if (smo->diverge_cnt == 0)
            smo->flag_observer_locked = 1;
    }

    // ========================================================================
    // 6. ATO/PLL Tracking (Synchronous Speed & Angle)
    // ========================================================================
    ctrl_gt pll_err = -ctl_mul(smo->psi_r_est.dat[0], sin_t) + ctl_mul(smo->psi_r_est.dat[1], cos_t);

    if (smo->psi_r_mag > float2ctrl(0.01f))
    {
        pll_err = ctl_div(pll_err, smo->psi_r_mag);
    }
    else
    {
        pll_err = float2ctrl(0.0f);
    }

    pll_err = ctl_mul(pll_err, CTL_CTRL_CONST_1_OVER_2PI);

    ctl_step_ato_pll(&smo->ato_pll, pll_err);

    // ========================================================================
    // 7. Slip & Mechanical Speed & Torque Calculation
    // ========================================================================
    ctrl_gt cross_flux_i = ctl_mul(smo->psi_r_est.dat[0], i_beta) - ctl_mul(smo->psi_r_est.dat[1], i_alpha);

    ctrl_gt w_slip_pu = float2ctrl(0.0f);
    if (psi_r_sq > float2ctrl(0.001f))
    {
        w_slip_pu = ctl_mul(smo->sf_slip_const, ctl_div(cross_flux_i, psi_r_sq));
    }

    ctrl_gt w_mech_pu = smo->ato_pll.elec_speed_pu - w_slip_pu;
    smo->torque_est = ctl_mul(smo->sf_torque_const, cross_flux_i);

    // ========================================================================
    // 8. Output Interfaces
    // ========================================================================
    smo->pos_out.elec_position = smo->ato_pll.elec_angle_pu;
    smo->spd_out.velocity = w_mech_pu;
}

/** @} */ // end of ACM_SMO group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ACM_SMO_H_
