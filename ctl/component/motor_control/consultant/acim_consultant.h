/**
 * @file consultant_im.h
 * @brief Implements the AC Induction Motor (IM) physical model.
 *
 * @version 1.0
 * @date 2024-10-27
 */

#ifndef _FILE_CONSULTANT_IM_H_
#define _FILE_CONSULTANT_IM_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Consultant: IM Physical Model                                             */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CONSULTANT_IM IM Model Consultant
 * @brief Manages the electrical and magnetic physical parameters of an Induction Motor.
 * @details Acts as a standardized parameter provider for IM inner loop controllers 
 * (IFOC, MPC, DTC). Automatically derives critical operational constants such as 
 * transient inductance, rotor time constant, and leakage factors.
 * @{
 */

/**
 * @brief Standardized structure for IM physical parameters.
 */
typedef struct _tag_consultant_im
{
    // --- Nameplate & Basic Electrical Parameters ---
    uint32_t pole_pairs; //!< Number of pole pairs (p).
    parameter_gt Rs;     //!< Stator phase resistance (Ohm).
    parameter_gt Rr;     //!< Rotor phase resistance referred to stator (Ohm).
    parameter_gt Ls;     //!< Stator self-inductance (H).
    parameter_gt Lr;     //!< Rotor self-inductance referred to stator (H).
    parameter_gt Lm;     //!< Magnetizing (Mutual) inductance (H).

    // --- Derived Intrinsic Properties (Calculated automatically) ---
    parameter_gt tau_r;         //!< Rotor time constant (Lr / Rr) [seconds].
    parameter_gt sigma;         //!< Total leakage factor (1 - Lm^2 / (Ls * Lr)).
    parameter_gt sigma_Ls;      //!< Stator transient inductance [H]. Plant for current loop PI.
    parameter_gt R_eq;          //!< Equivalent stator resistance [Ohm]. Plant for current loop PI.
    parameter_gt Lm_sq_over_Lr; //!< Lm^2 / Lr. Used frequently in torque and EMF equations.

} ctl_consultant_im_t;

//================================================================================
// Function Prototypes & Inline APIs
//================================================================================

/**
 * @brief Initializes the IM model and derives intrinsic physics parameters.
 * @param[out] motor Pointer to the IM consultant instance.
 * @param[in]  pp    Pole pairs.
 * @param[in]  rs    Stator resistance (Ohm).
 * @param[in]  rr    Rotor resistance (Ohm).
 * @param[in]  ls    Stator self-inductance (H).
 * @param[in]  lr    Rotor self-inductance (H).
 * @param[in]  lm    Mutual inductance (H).
 */
void ctl_consultant_im_init(ctl_consultant_im_t* motor, uint32_t pp, parameter_gt rs, parameter_gt rr, parameter_gt ls,
                            parameter_gt lr, parameter_gt lm);

/**
 * @brief Calculates steady-state slip frequency based on current commands.
 * @details omega_slip = (Rr / Lr) * (Iq / Id) = (1 / tau_r) * (Iq / Id)
 * @param[in] motor Pointer to the IM consultant instance.
 * @param[in] id    D-axis magnetizing current (A).
 * @param[in] iq    Q-axis torque current (A).
 * @return parameter_gt Slip frequency in mechanical rad/s (electrical rad/s / p).
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_im_calc_slip_rads(const ctl_consultant_im_t* motor, parameter_gt id,
                                                                parameter_gt iq)
{
    // Prevent division by zero if Id is not established
    parameter_gt id_safe = (id > 0.01f) ? id : 0.01f;
    return (1.0f / motor->tau_r) * (iq / id_safe);
}

/**
 * @brief Calculates steady-state electromagnetic torque from currents.
 * @details Assuming perfect rotor flux orientation (Psi_r = Lm * Id).
 * Te = 1.5 * p * (Lm^2 / Lr) * Id * Iq
 * @param[in] motor Pointer to the IM consultant instance.
 * @param[in] id    D-axis magnetizing current (A).
 * @param[in] iq    Q-axis torque current (A).
 * @return parameter_gt Electromagnetic torque (Nm).
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_im_calc_torque_steady(const ctl_consultant_im_t* motor, parameter_gt id,
                                                                    parameter_gt iq)
{
    return 1.5f * (parameter_gt)motor->pole_pairs * motor->Lm_sq_over_Lr * id * iq;
}

/**
 * @brief Calculates dynamic electromagnetic torque using estimated rotor flux.
 * @details More accurate during transients than the steady-state equation.
 * Te = 1.5 * p * (Lm / Lr) * Psi_r * Iq
 * @param[in] motor Pointer to the IM consultant instance.
 * @param[in] psi_r Estimated rotor flux magnitude (Weber).
 * @param[in] iq    Q-axis torque current (A).
 * @return parameter_gt Electromagnetic torque (Nm).
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_im_calc_torque_dynamic(const ctl_consultant_im_t* motor,
                                                                     parameter_gt psi_r, parameter_gt iq)
{
    return 1.5f * (parameter_gt)motor->pole_pairs * (motor->Lm / motor->Lr) * psi_r * iq;
}

/**
 * @brief Utility: Convert T-equivalent circuit inductances to Standard Ls, Lr, Lm.
 * @details Often datasheets provide L_ls (stator leakage) and L_lr (rotor leakage) instead of Ls and Lr.
 * Ls = L_ls + Lm; Lr = L_lr + Lm
 */
GMP_STATIC_INLINE void ctl_consultant_im_leakage_to_self_ind(parameter_gt l_ls, parameter_gt l_lr, parameter_gt lm,
                                                             parameter_gt* ls_out, parameter_gt* lr_out)
{
    *ls_out = l_ls + lm;
    *lr_out = l_lr + lm;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CONSULTANT_IM_H_
