/**
 * @file dtc_svm_core.h
 * @brief Implements an advanced DTC-SVM (Direct Torque Control - Space Vector Modulation) core.
 *
 * @version 1.0
 * @date 2024-10-26
 *
 */

#ifndef _FILE_DTC_SVM_CORE_H_
#define _FILE_DTC_SVM_CORE_H_

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/math_block/coordinate/coord_trans.h>
#include <ctl/math_block/vector_lite/vector2.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* DTC-SVM Core Controller                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup DTC_SVM_CONTROLLER DTC-SVM Controller
 * @brief Direct Torque Control with fixed switching frequency using SVM.
 * @details Directly controls stator flux magnitude and electromagnetic torque.
 * Includes a robust voltage-model flux observer with DC-drift compensation.
 * Operates entirely in Per-Unit (PU) space, completely eliminating trigonometric 
 * functions (atan2/sin/cos) from the real-time execution path.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Initialization parameters for the DTC-SVM Controller.
 */
typedef struct _tag_dtc_svm_init
{
    parameter_gt fs;            //!< Controller execution frequency (Hz).
    parameter_gt v_bus;         //!< Nominal DC Bus voltage (V).
    parameter_gt v_phase_limit; //!< Phase voltage limitation (Vrms).

    // --- PU Base Values ---
    parameter_gt v_base;     //!< Base voltage (V).
    parameter_gt i_base;     //!< Base current (A).
    parameter_gt spd_base;   //!< Base speed (rpm).
    parameter_gt pole_pairs; //!< Number of pole pairs.

    // --- Motor Physical Parameters ---
    parameter_gt mtr_Rs;   //!< Stator resistance (Ohm).
    parameter_gt mtr_Flux; //!< Nominal Flux Linkage (Wb) - Used to seed the observer.

    // --- Tuning Targets ---
    parameter_gt flux_drift_wc; //!< Flux observer drift compensation cutoff freq (rad/s) (e.g., 2~10 rad/s).
    parameter_gt bw_flux;       //!< Flux loop bandwidth (Hz).
    parameter_gt bw_torque;     //!< Torque loop bandwidth (Hz).

} dtc_svm_init_t;

/**
 * @brief Main structure for the DTC-SVM controller.
 */
typedef struct _tag_dtc_svm_ctrl
{
    // --- Interfaces ---
    ctl_vector2_t* iab_meas; //!< Measured alpha-beta stator currents (PU).

    // --- Setpoints ---
    ctrl_gt torque_ref_pu; //!< Target electromagnetic torque (PU).
    ctrl_gt flux_ref_pu;   //!< Target stator flux magnitude (PU).

    // --- Flux Observer States (The core of DTC) ---
    ctl_vector2_t flux_ab_pu;  //!< Estimated stator flux in alpha-beta frame (PU).
    ctrl_gt flux_mag_pu;       //!< Estimated stator flux magnitude |Psi_s| (PU).
    ctrl_gt torque_est_pu;     //!< Estimated electromagnetic torque (PU).
    ctl_vector2_t flux_phasor; //!< [cos(theta), sin(theta)] of the flux vector. Extracted without atan2!

    // --- Pre-calculated Math Constants (PU Space) ---
    ctrl_gt coef_ts_wbase;   //!< Integration constant: Ts * Omega_base_elec
    ctrl_gt coef_rs_pu;      //!< Stator resistance PU: (Rs * I_base) / V_base
    ctrl_gt coef_drift_comp; //!< Drift compensation multiplier: (1.0 - wc * Ts)

    // --- Controllers ---
    ctl_pid_t flux_ctrl;   //!< PI Controller for flux magnitude (Outputs d-axis voltage).
    ctl_pid_t torque_ctrl; //!< PI Controller for torque (Outputs q-axis voltage).

    // --- Voltage States ---
    ctrl_gt v_max_pu;       //!< Voltage saturation limit (PU).
    ctl_vector2_t vxy_ctrl; //!< Commanded voltage in flux synchronous frame (x-y).
    ctl_vector2_t vab_out;  //!< Output voltage in stationary frame (alpha-beta) to feed SVPWM.

    // --- Flags ---
    fast_gt flag_enable; //!< Enable switch.

} dtc_svm_ctrl_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_init_dtc_svm(dtc_svm_ctrl_t* mc, const dtc_svm_init_t* init);

GMP_STATIC_INLINE void ctl_enable_dtc_svm(dtc_svm_ctrl_t* mc)
{
    mc->flag_enable = 1;
}
GMP_STATIC_INLINE void ctl_disable_dtc_svm(dtc_svm_ctrl_t* mc)
{
    mc->flag_enable = 0;
    ctl_vector2_clear(&mc->vab_out);
    ctl_vector2_clear(&mc->vxy_ctrl);
}

/**
 * @brief Sets the torque and flux references.
 */
GMP_STATIC_INLINE void ctl_set_dtc_svm_ref(dtc_svm_ctrl_t* mc, ctrl_gt torque_ref, ctrl_gt flux_ref)
{
    mc->torque_ref_pu = torque_ref;
    mc->flux_ref_pu = flux_ref;
}

/**
 * @brief Executes one step of the DTC-SVM Controller.
 * @details Executed entirely in PU space. Uses the previous voltage command 
 * for the flux observer (Delay compensation).
 */
GMP_STATIC_INLINE void ctl_step_dtc_svm(dtc_svm_ctrl_t* mc)
{
    if (!mc->flag_enable)
        return;

    // Local copy of measured currents
    ctrl_gt i_alpha = mc->iab_meas->dat[0];
    ctrl_gt i_beta = mc->iab_meas->dat[1];

    // ========================================================================
    // 1. Stator Flux Observer (Voltage Model with Drift Compensation)
    // ========================================================================
    // Physics Eq: dPsi/dt = V - Rs*I
    // Discrete PU Eq: Psi(k+1) = Psi(k) * decay + Ts*W_base * (V(k) - Rs_pu*I(k))
    // Note: We use the voltage output from the *previous* control cycle (mc->vab_out).

    ctrl_gt back_emf_alpha = mc->vab_out.dat[0] - ctl_mul(mc->coef_rs_pu, i_alpha);
    ctrl_gt back_emf_beta = mc->vab_out.dat[1] - ctl_mul(mc->coef_rs_pu, i_beta);

    mc->flux_ab_pu.dat[0] =
        ctl_mul(mc->coef_drift_comp, mc->flux_ab_pu.dat[0]) + ctl_mul(mc->coef_ts_wbase, back_emf_alpha);
    mc->flux_ab_pu.dat[1] =
        ctl_mul(mc->coef_drift_comp, mc->flux_ab_pu.dat[1]) + ctl_mul(mc->coef_ts_wbase, back_emf_beta);

    // ========================================================================
    // 2. Flux Magnitude & Torque Estimation (The Magic of PU)
    // ========================================================================
    ctrl_gt flux_sq =
        ctl_mul(mc->flux_ab_pu.dat[0], mc->flux_ab_pu.dat[0]) + ctl_mul(mc->flux_ab_pu.dat[1], mc->flux_ab_pu.dat[1]);

    mc->flux_mag_pu = ctl_sqrt(flux_sq);

    // PU Torque Eq: Te_pu = Psi_alpha * I_beta - Psi_beta * I_alpha (Notice: NO 3/2*p constant!)
    mc->torque_est_pu = ctl_mul(mc->flux_ab_pu.dat[0], i_beta) - ctl_mul(mc->flux_ab_pu.dat[1], i_alpha);

    // ========================================================================
    // 3. Fast Phasor Extraction (ZERO Trigonometric Functions)
    // ========================================================================
    // We extract the cos and sin directly from the flux vector to align the x-axis with the stator flux.
    if (mc->flux_mag_pu > float2ctrl(0.01f)) // Protect against div-by-zero
    {
        mc->flux_phasor.dat[0] = ctl_div(mc->flux_ab_pu.dat[0], mc->flux_mag_pu); // cos(theta)
        mc->flux_phasor.dat[1] = ctl_div(mc->flux_ab_pu.dat[1], mc->flux_mag_pu); // sin(theta)
    }

    // ========================================================================
    // 4. Flux and Torque PI Controllers (x-y synchronous frame)
    // ========================================================================
    ctrl_gt flux_err = mc->flux_ref_pu - mc->flux_mag_pu;
    ctrl_gt torque_err = mc->torque_ref_pu - mc->torque_est_pu;

    // Vx controls Flux magnitude (radial)
    mc->vxy_ctrl.dat[0] = ctl_step_pid_ser(&mc->flux_ctrl, flux_err);
    // Vy controls Torque (tangential)
    mc->vxy_ctrl.dat[1] = ctl_step_pid_ser(&mc->torque_ctrl, torque_err);

    // Circular Saturation
    ctrl_gt v_sq =
        ctl_mul(mc->vxy_ctrl.dat[0], mc->vxy_ctrl.dat[0]) + ctl_mul(mc->vxy_ctrl.dat[1], mc->vxy_ctrl.dat[1]);
    ctrl_gt v_max_sq = ctl_mul(mc->v_max_pu, mc->v_max_pu);

    if (v_sq > v_max_sq)
    {
        ctrl_gt v_mag = ctl_sqrt(v_sq);
        ctrl_gt scale = ctl_div(mc->v_max_pu, v_mag);
        mc->vxy_ctrl.dat[0] = ctl_mul(mc->vxy_ctrl.dat[0], scale);
        mc->vxy_ctrl.dat[1] = ctl_mul(mc->vxy_ctrl.dat[1], scale);

        // Feed real saturated output back to PID for Anti-Windup
        ctl_pid_clamping_correction_using_real_output(&mc->flux_ctrl, mc->vxy_ctrl.dat[0]);
        ctl_pid_clamping_correction_using_real_output(&mc->torque_ctrl, mc->vxy_ctrl.dat[1]);
    }

    // ========================================================================
    // 5. Inverse Transformation (x-y back to alpha-beta)
    // ========================================================================
    // V_alpha = Vx * cos - Vy * sin
    // V_beta  = Vx * sin + Vy * cos
    mc->vab_out.dat[0] =
        ctl_mul(mc->vxy_ctrl.dat[0], mc->flux_phasor.dat[0]) - ctl_mul(mc->vxy_ctrl.dat[1], mc->flux_phasor.dat[1]);
    mc->vab_out.dat[1] =
        ctl_mul(mc->vxy_ctrl.dat[0], mc->flux_phasor.dat[1]) + ctl_mul(mc->vxy_ctrl.dat[1], mc->flux_phasor.dat[0]);

    // Note: mc->vab_out is now ready to be fed into the standard SVPWM module.
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_DTC_SVM_CORE_H_
