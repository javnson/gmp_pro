/**
 * @file three_phase_additional.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for a preset three-phase DC/AC inverter additional controller.
 * @version 1.0
 * @date 2026-01-26
 *
 * @copyright Copyright GMP(c) 2025
 */

/** 
 * @defgroup CTL_TOPOLOGY_GFL_INV_H_API Three-Phase GFL Inverter Topology API (Header)
 * @{
 * @ingroup CTL_DP_LIB
 * @brief Defines the data structures, control flags, and function interfaces for a
 * comprehensive three-phase inverter, including harmonic compensation, droop control,
 * and multiple operating modes.
 */

#ifndef _FILE_THREE_PHASE_GFL_
#define _FILE_THREE_PHASE_GFL_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <ctl/math_block/coordinate/coord_trans.h>

#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

#include <ctl/component/intrinsic/discrete/biquad_filter.h>
#include <ctl/component/intrinsic/discrete/lead_lag.h>
#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

// 三相常用的附加控制器

// TODO harmonic controller

// TODO negative controller

/**
 * @brief Main data structure for the three-phase negative sequence controller.
 */
typedef struct _tag_neg_inv_ctrl_type
{
    //
    // --- Input Ports (Pointers to Main Controller Data) ---
    //
    ctl_vector2_t* iab;    //!< RO: Pointer to Clarke transformed currents {alpha, beta} from main controller.
    ctl_vector2_t* vab;    //!< RO: Pointer to Clarke transformed voltages {alpha, beta} (measured).
    ctl_vector2_t* phasor; //!< RO: Pointer to Positive Sequence Phasor {sin, cos} from PLL.

    //
    // --- Output Ports ---
    //
    ctl_vector2_t vab_out; //!< RO: Calculated negative sequence voltage injection {alpha, beta}.

    //
    // --- Setpoints & Intermediate Variables ---
    //
    ctl_vector2_t idqn_set; //!< R/W: Target negative sequence current (usually 0 for suppression).
    ctl_vector2_t vdqn_set; //!< R/W: Target negative sequence voltage (usually 0 for compensation).

    //
    // --- Measurement & Internal State Variables ---
    //
    ctl_vector2_t idqn_raw; //!< RO: Unfiltered Park transformed currents in negative frame.
    ctl_vector2_t idqn;     //!< RO: Filtered negative sequence currents (DC).

    ctl_vector2_t vdqn_raw; //!< RO: Unfiltered Park transformed voltages in negative frame.
    ctl_vector2_t vdqn;     //!< RO: Filtered negative sequence voltages (DC).

    ctl_vector2_t idqn_ref_int; //!< RO: Internal current reference (from voltage loop or external).
    ctl_vector2_t vdqn_out;     //!< RO: Output of current loop in negative dq frame.

    //
    // --- Controller Objects ---
    //

    // Filters to remove 2*omega (100Hz) ripple caused by positive sequence
    // Using 2nd Order Biquad Filter for better attenuation
    ctl_filter_IIR2_t filter_idqn[2];
    ctl_filter_IIR2_t filter_vdqn[2];

    // Controllers
    ctl_pid_t pid_idqn[2]; //!< Inner Loop: Current Control
    ctl_pid_t pid_vdqn[2]; //!< Outer Loop: Voltage Control

    //
    // --- Control Flags ---
    //
    fast_gt flag_enable_negative_current_ctrl; //!< Enable Inner Loop (Current Suppression)
    fast_gt flag_enable_negative_voltage_ctrl; //!< Enable Outer Loop (Voltage Support/Compensation)

} inv_neg_ctrl_t;

/**
 * @brief Clears all internal states of the negative sequence controller.
 * @param[out] neg Pointer to the negative controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_neg_inv(inv_neg_ctrl_t* neg)
{
    // Clear Outputs
    ctl_vector2_clear(&neg->vab_out);
    ctl_vector2_clear(&neg->vdqn_out);

    // Clear Internal States
    ctl_vector2_clear(&neg->idqn_raw);
    ctl_vector2_clear(&neg->idqn);
    ctl_vector2_clear(&neg->vdqn_raw);
    ctl_vector2_clear(&neg->vdqn);
    ctl_vector2_clear(&neg->idqn_ref_int);

    // Clear Filters
    ctl_clear_biquad_filter(&neg->filter_idqn[phase_d]);
    ctl_clear_biquad_filter(&neg->filter_idqn[phase_q]);
    ctl_clear_biquad_filter(&neg->filter_vdqn[phase_d]);
    ctl_clear_biquad_filter(&neg->filter_vdqn[phase_q]);

    // Clear PIDs
    ctl_clear_pid(&neg->pid_idqn[phase_d]);
    ctl_clear_pid(&neg->pid_idqn[phase_q]);
    ctl_clear_pid(&neg->pid_vdqn[phase_d]);
    ctl_clear_pid(&neg->pid_vdqn[phase_q]);
}

/**
 * @brief Initialization structure for auto-tuning the negative sequence controller.
 */
typedef struct _tag_neg_inv_ctrl_init
{
    parameter_gt fs;        //!< Sampling frequency (Hz).
    parameter_gt freq_base; //!< Grid frequency (Hz).

    // --- Filter settings (Crucial for separating Neg/Pos sequence) ---
    parameter_gt seq_filter_fc; //!< Cut-off frequency for Idqn/Vdqn filters (e.g., 10-20Hz).
    parameter_gt seq_filter_q;  //!< Quality factor for the Biquad LPF (e.g., 0.707 for Butterworth).

    // --- Current Loop (Inner) ---
    parameter_gt kp_current;        //!< Proportional gain for negative current loop.
    parameter_gt ki_current;        //!< Integral gain for negative current loop.
    parameter_gt limit_current_out; //!< Voltage output limit (p.u.).

    // --- Voltage Loop (Outer) - Optional ---
    parameter_gt kp_voltage;        //!< Proportional gain for negative voltage loop.
    parameter_gt ki_voltage;        //!< Integral gain for negative voltage loop.
    parameter_gt limit_voltage_out; //!< Current reference limit (p.u.).

} inv_neg_ctrl_init_t;

/**
 * @brief Auto-tuning Negative Sequence Controller parameters based on GFL system parameters.
 * @details Calculates reasonable bandwidths and filter settings derived from the main grid filter L/C and base values.
 * @param[out] neg_init Pointer to the negative controller init structure to be filled.
 * @param[in] gfl_init Pointer to the source GFL system init structure (provides L, C, bases, etc.).
 */
void ctl_auto_tuning_neg_inv(inv_neg_ctrl_init_t* neg_init, const gfl_inv_ctrl_init_t* gfl_init);

/**
 * @brief Update Negative Sequence Controller coefficients.
 * @details Calculates Kp, Ki and initializes filters/PIDs based on the init structure.
 * @param[out] neg Pointer to the negative controller object.
 * @param[in] neg_init Pointer to the negative controller init structure.
 * @param[in] gfl_init Pointer to the GFL init structure (needed for L, C, Base values during calculation).
 */
void ctl_update_neg_inv_coeff(inv_neg_ctrl_t* neg, const inv_neg_ctrl_init_t* neg_init,
                              const gfl_inv_ctrl_init_t* gfl_init);

/**
 * @brief Initialize Negative Sequence Controller.
 * @param[out] neg Pointer to the negative controller object.
 * @param[in] neg_init Pointer to the negative controller init structure.
 * @param[in] gfl_init Pointer to the GFL init structure.
 */
void ctl_init_neg_inv(inv_neg_ctrl_t* neg, const inv_neg_ctrl_init_t* neg_init, const gfl_inv_ctrl_init_t* gfl_init);

/**
 * @brief Executes one step of the Negative Sequence Controller.
 * @details Must be called after measurement sampling and PLL, but before PWM generation.
 * @param[in,out] neg Pointer to the negative controller instance.
 */
GMP_STATIC_INLINE void ctl_step_neg_inv_ctrl(inv_neg_ctrl_t* neg)
{
    // Safety checks
    if (!neg->flag_enable_negative_current_ctrl)
    {
        ctl_vector2_clear(&neg->vab_out);
        // Optional: Reset internal states if desired when disabled
        return;
    }

    // Check pointers are valid
    gmp_base_assert(neg->iab);
    gmp_base_assert(neg->phasor);

    // --- 1. Coordinate Transformation (AlphaBeta -> Neg DQ) ---
    ctl_ct_park2_neg(neg->iab, neg->phasor, &neg->idqn_raw);

    if (neg->vab != NULL)
    {
        ctl_ct_park2_neg(neg->vab, neg->phasor, &neg->vdqn_raw);
    }

    // --- 2. Filtering (Critical: 2nd Order Biquad) ---
    // Remove 2*omega ripple to extract DC negative sequence component
    neg->idqn.dat[phase_d] = ctl_step_biquad_filter(&neg->filter_idqn[phase_d], neg->idqn_raw.dat[phase_d]);
    neg->idqn.dat[phase_q] = ctl_step_biquad_filter(&neg->filter_idqn[phase_q], neg->idqn_raw.dat[phase_q]);

    if (neg->flag_enable_negative_voltage_ctrl && neg->vab != NULL)
    {
        neg->vdqn.dat[phase_d] = ctl_step_biquad_filter(&neg->filter_vdqn[phase_d], neg->vdqn_raw.dat[phase_d]);
        neg->vdqn.dat[phase_q] = ctl_step_biquad_filter(&neg->filter_vdqn[phase_q], neg->vdqn_raw.dat[phase_q]);
    }

    // --- 3. Outer Loop: Voltage Control (Optional) ---
    if (neg->flag_enable_negative_voltage_ctrl)
    {
        // Calculate Error: V_set - V_meas
        ctrl_gt err_vd = neg->vdqn_set.dat[phase_d] - neg->vdqn.dat[phase_d];
        ctrl_gt err_vq = neg->vdqn_set.dat[phase_q] - neg->vdqn.dat[phase_q];

        neg->idqn_ref_int.dat[phase_d] = ctl_step_pid_ser(&neg->pid_vdqn[phase_d], err_vd);
        neg->idqn_ref_int.dat[phase_q] = ctl_step_pid_ser(&neg->pid_vdqn[phase_q], err_vq);
    }
    else
    {
        // Direct Current Suppression (Ref usually 0)
        neg->idqn_ref_int.dat[phase_d] = neg->idqn_set.dat[phase_d];
        neg->idqn_ref_int.dat[phase_q] = neg->idqn_set.dat[phase_q];
    }

    // --- 4. Inner Loop: Current Control ---
    // Error: I_ref - I_meas
    ctrl_gt err_id = neg->idqn_ref_int.dat[phase_d] - neg->idqn.dat[phase_d];
    ctrl_gt err_iq = neg->idqn_ref_int.dat[phase_q] - neg->idqn.dat[phase_q];

    neg->vdqn_out.dat[phase_d] = ctl_step_pid_ser(&neg->pid_idqn[phase_d], err_id);
    neg->vdqn_out.dat[phase_q] = ctl_step_pid_ser(&neg->pid_idqn[phase_q], err_iq);

    // --- 5. Coordinate Transformation (Neg DQ -> AlphaBeta) ---
    // Uses the provided optimized function with Positive Phasor
    ctl_ct_ipark2_neg(&neg->vdqn_out, neg->phasor, &neg->vab_out);
}

/**
 * @brief Attach the negative controller to a Grid-Following Inverter controller.
 * @details Automatically maps the Alpha/Beta currents, voltages, and PLL phasor.
 * @param[in,out] neg Pointer to the negative sequence controller.
 * @param[in] gfl Pointer to the main GFL inverter controller.
 */
GMP_STATIC_INLINE void ctl_attach_neg_inv_to_gfl(inv_neg_ctrl_t* neg, gfl_inv_ctrl_t* gfl)
{
    gmp_base_assert(neg);
    gmp_base_assert(gfl);

    // Casting ctl_vector3_t* to ctl_vector2_t* assumes the memory layout
    // of the first two elements (alpha, beta) matches.
    neg->iab = (ctl_vector2_t*)&gfl->iab0;
    neg->vab = (ctl_vector2_t*)&gfl->vab0;

    // Phasor is already vector2
    neg->phasor = &gfl->phasor;
}

/**
 * @brief Attach external pointers to the negative controller.
 */
GMP_STATIC_INLINE void ctl_attach_neg_inv(inv_neg_ctrl_t* neg, ctl_vector2_t* iab, ctl_vector2_t* vab,
                                          ctl_vector2_t* phasor)
{
    neg->iab = iab;
    neg->vab = vab;
    neg->phasor = phasor;
}



#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_THREE_PHASE_DC_AC_H_

/**
 * @}
 */
