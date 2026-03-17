
/**
 * @file consultant_pu.h
 * @brief Implements the Per-Unit (PU) Base Model Consultant for PMSM and IM.
 *
 * @version 1.0
 * @date 2024-10-27
 */

#include <ctl/component/motor_control/consultant/acim_consultant.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>

#ifndef _FILE_CONSULTANT_PU_H_
#define _FILE_CONSULTANT_PU_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Consultant: Per-Unit (PU) Base Models                                     */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CONSULTANT_PU Per-Unit Model Consultant
 * @brief Derives all mathematical base values for normalization.
 * @details Acts as the "Ruler" of the system. It takes fundamental bases (V, I, W)
 * and derives impedance, inductance, flux, and torque bases. Includes helper APIs 
 * to combine PU models with Motor models to output normalized parameters.
 * @{
 */

//================================================================================
// 1. PMSM Per-Unit Base Model
//================================================================================

/**
 * @brief Standardized structure for PMSM PU bases.
 */
typedef struct _tag_consultant_pu_pmsm
{
    // --- Fundamental Bases ---
    parameter_gt V_base; //!< Base Phase Voltage Peak (V).
    parameter_gt I_base; //!< Base Phase Current Peak (A).
    parameter_gt W_base; //!< Base Electrical Angular Velocity (rad/s).

    // --- Derived Electrical Bases ---
    parameter_gt Z_base;    //!< Base Impedance (Ohm): V_base / I_base
    parameter_gt L_base;    //!< Base Inductance (H): Z_base / W_base
    parameter_gt Flux_base; //!< Base Flux Linkage (Wb): V_base / W_base

    // --- Derived Mechanical Bases ---
    parameter_gt P_base; //!< Base Power (W): 1.5 * V_base * I_base
    parameter_gt T_base; //!< Base Torque (Nm): P_base / (W_base / pole_pairs)

} ctl_consultant_pu_pmsm_t;

//================================================================================
// 2. IM Per-Unit Base Model (With Secondary/Rotor Bases)
//================================================================================

/**
 * @brief Standardized structure for IM PU bases.
 */
typedef struct _tag_consultant_pu_im
{
    // --- Fundamental Stator (Primary) Bases ---
    parameter_gt V_s_base; //!< Stator Base Phase Voltage Peak (V).
    parameter_gt I_s_base; //!< Stator Base Phase Current Peak (A).
    parameter_gt W_base;   //!< Base Electrical Angular Velocity (rad/s).

    // --- Derived Stator Bases ---
    parameter_gt Z_s_base;    //!< Stator Base Impedance (Ohm).
    parameter_gt L_s_base;    //!< Stator Base Inductance (H).
    parameter_gt Flux_s_base; //!< Stator Base Flux Linkage (Wb).

    // --- Rotor (Secondary) Turns Ratio ---
    parameter_gt
        turns_ratio; //!< Stator-to-Rotor turns ratio (Ns / Nr). Use 1.0 if parameters are already referred to stator.

    // --- Derived Rotor (Secondary) Bases ---
    parameter_gt V_r_base;    //!< Rotor Base Voltage: V_s_base / turns_ratio
    parameter_gt I_r_base;    //!< Rotor Base Current: I_s_base * turns_ratio
    parameter_gt Z_r_base;    //!< Rotor Base Impedance: Z_s_base / (turns_ratio^2)
    parameter_gt L_r_base;    //!< Rotor Base Inductance: L_s_base / (turns_ratio^2)
    parameter_gt Flux_r_base; //!< Rotor Base Flux: Flux_s_base / turns_ratio

    // --- Derived Mechanical Bases ---
    parameter_gt P_base; //!< Base Power (W): 1.5 * V_s_base * I_s_base
    parameter_gt T_base; //!< Base Torque (Nm).

} ctl_consultant_pu_im_t;

//================================================================================
// Function Prototypes & Initialization
//================================================================================

/**
 * @brief Initializes the PMSM PU model and derives all bases.
 */
void ctl_consultant_pu_pmsm_init(ctl_consultant_pu_pmsm_t* pu, parameter_gt v_base, parameter_gt i_base,
                                 parameter_gt w_base, uint32_t pole_pairs);

/**
 * @brief Initializes the IM PU model, deriving both stator and rotor bases.
 * @param turns_ratio Effective turns ratio (Ns/Nr). If the motor datasheet gives 
 * parameters already referred to the stator, set this strictly to 1.0f.
 */
void ctl_consultant_pu_im_init(ctl_consultant_pu_im_t* pu, parameter_gt v_base, parameter_gt i_base,
                               parameter_gt w_base, uint32_t pole_pairs, parameter_gt turns_ratio);

//================================================================================
// PU + Physics Bridge APIs (The "Translation" Layer)
//================================================================================

// ---------------- PMSM Normalization ----------------

GMP_STATIC_INLINE ctrl_gt ctl_pu_pmsm_get_Rs_pu(const ctl_consultant_pu_pmsm_t* pu, const ctl_consultant_pmsm_t* motor)
{
    return float2ctrl(motor->Rs / pu->Z_base);
}

GMP_STATIC_INLINE ctrl_gt ctl_pu_pmsm_get_Ld_pu(const ctl_consultant_pu_pmsm_t* pu, const ctl_consultant_pmsm_t* motor)
{
    return float2ctrl(motor->Ld / pu->L_base);
}

GMP_STATIC_INLINE ctrl_gt ctl_pu_pmsm_get_Lq_pu(const ctl_consultant_pu_pmsm_t* pu, const ctl_consultant_pmsm_t* motor)
{
    return float2ctrl(motor->Lq / pu->L_base);
}

GMP_STATIC_INLINE ctrl_gt ctl_pu_pmsm_get_Flux_pu(const ctl_consultant_pu_pmsm_t* pu,
                                                  const ctl_consultant_pmsm_t* motor)
{
    return float2ctrl(motor->flux_linkage / pu->Flux_base);
}

// ---------------- IM Normalization ----------------

GMP_STATIC_INLINE ctrl_gt ctl_pu_im_get_Rs_pu(const ctl_consultant_pu_im_t* pu, const ctl_consultant_im_t* motor)
{
    return float2ctrl(motor->Rs / pu->Z_s_base);
}

GMP_STATIC_INLINE ctrl_gt ctl_pu_im_get_Rr_pu(const ctl_consultant_pu_im_t* pu, const ctl_consultant_im_t* motor)
{
    // Rotor resistance is normalized against the Rotor Base Impedance!
    return float2ctrl(motor->Rr / pu->Z_r_base);
}

GMP_STATIC_INLINE ctrl_gt ctl_pu_im_get_Ls_pu(const ctl_consultant_pu_im_t* pu, const ctl_consultant_im_t* motor)
{
    return float2ctrl(motor->Ls / pu->L_s_base);
}

GMP_STATIC_INLINE ctrl_gt ctl_pu_im_get_Lr_pu(const ctl_consultant_pu_im_t* pu, const ctl_consultant_im_t* motor)
{
    // Rotor inductance is normalized against the Rotor Base Inductance!
    return float2ctrl(motor->Lr / pu->L_r_base);
}

GMP_STATIC_INLINE ctrl_gt ctl_pu_im_get_Lm_pu(const ctl_consultant_pu_im_t* pu, const ctl_consultant_im_t* motor)
{
    // Mutual inductance is technically a primary-side equivalent in most models,
    // but strictly speaking, it relates primary and secondary. Assuming referred to stator:
    return float2ctrl(motor->Lm / pu->L_s_base);
}

GMP_STATIC_INLINE ctrl_gt ctl_pu_im_get_sigma_Ls_pu(const ctl_consultant_pu_im_t* pu, const ctl_consultant_im_t* motor)
{
    return float2ctrl(motor->sigma_Ls / pu->L_s_base);
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CONSULTANT_PU_H_
