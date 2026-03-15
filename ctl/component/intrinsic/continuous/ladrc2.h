/**
 * @file ladrc2.h
 * @brief Implements a highly optimized, Fixed-Point friendly 2nd-Order LADRC.
 *
 * @version 1.0
 * @date 2024-10-26
 *
 */

#ifndef _FILE_CTL_LADRC2_H_
#define _FILE_CTL_LADRC2_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup LADRC2 2nd-Order LADRC
 * @brief A generic 2nd-order Linear Active Disturbance Rejection Controller.
 * @details Designed for 2nd-order plant models (e.g., d^2y/dt^2 = b0*u + f). 
 * Contains a 3rd-order Linear Extended State Observer (LESO) and a Proportional-Derivative 
 * State Error Feedback (LSEF) control law with acceleration feedforward.
 * Highly optimized for fixed-point execution by absorbing sample time (h) and gain (b0).
 * @{
 */

/*---------------------------------------------------------------------------*/
/* 2nd-Order LADRC Controller                                                */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the 2nd-Order LADRC execution.
 */
typedef struct _tag_ladrc2_t
{
    // --- Pre-calculated Absorbed Parameters (PU / Scaled) ---
    ctrl_gt h;          //!< Sample time (h).
    ctrl_gt h_b0;       //!< h * b0
    ctrl_gt h_beta1;    //!< h * 3 * wo
    ctrl_gt h_beta2;    //!< h * 3 * wo^2
    ctrl_gt h_beta3_b0; //!< h * (wo^3) / b0

    ctrl_gt kpp_b0; //!< wc^2 / b0
    ctrl_gt kvp_b0; //!< 2 * wc / b0
    ctrl_gt b0_inv; //!< 1 / b0 (Used for acceleration feedforward)

    // --- Limits ---
    ctrl_gt out_max; //!< Maximum output limit.
    ctrl_gt out_min; //!< Minimum output limit.

    // --- State Variables ---
    ctrl_gt z1;     //!< Estimated system position (filtered feedback).
    ctrl_gt z2;     //!< Estimated system velocity.
    ctrl_gt z3_u;   //!< Estimated total disturbance EQUIVALENT input (z3 / b0).
    ctrl_gt u_prev; //!< Controller output from the previous step u(k-1).
    ctrl_gt out;    //!< Current controller output.

} ctl_ladrc2_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

/**
 * @brief Dynamically upgrades/updates the 2nd-order LADRC parameters seamlessly.
 * @param[out] ladrc Pointer to the LADRC instance.
 * @param[in]  b0    Estimated system gain.
 * @param[in]  fc    Controller bandwidth (Hz).
 * @param[in]  fo    Observer bandwidth (Hz).
 * @param[in]  fs    Controller execution frequency (Hz).
 */
void ctl_upgrade_ladrc2(ctl_ladrc2_t* ladrc, parameter_gt b0, parameter_gt fc, parameter_gt fo, parameter_gt fs);

/**
 * @brief Initializes the 2nd-order LADRC controller to safe defaults.
 */
void ctl_init_ladrc2(ctl_ladrc2_t* ladrc, parameter_gt b0, parameter_gt fc, parameter_gt fo, parameter_gt fs);

/**
 * @brief Clears the internal observer states of the LADRC controller.
 */
GMP_STATIC_INLINE void ctl_clear_ladrc2(ctl_ladrc2_t* ladrc)
{
    ladrc->z1 = float2ctrl(0.0f);
    ladrc->z2 = float2ctrl(0.0f);
    ladrc->z3_u = float2ctrl(0.0f);
    ladrc->u_prev = float2ctrl(0.0f);
    ladrc->out = float2ctrl(0.0f);
}

/**
 * @brief Sets the output saturation limits.
 */
GMP_STATIC_INLINE void ctl_set_ladrc2_limit(ctl_ladrc2_t* ladrc, ctrl_gt out_max, ctrl_gt out_min)
{
    ladrc->out_max = out_max;
    ladrc->out_min = out_min;
}

/**
 * @brief Pre-loads the observer states for bumpless transfer mode switching.
 */
GMP_STATIC_INLINE void ctl_set_ladrc2_states(ctl_ladrc2_t* ladrc, ctrl_gt current_pos, ctrl_gt current_vel,
                                             ctrl_gt current_u)
{
    ladrc->z1 = current_pos;
    ladrc->z2 = current_vel;
    ladrc->u_prev = current_u;
}

/**
 * @brief Executes one step of the 2nd-Order LADRC.
 * @param[in,out] ladrc   Pointer to the LADRC instance.
 * @param[in]     ref_pos The target position reference.
 * @param[in]     ref_vel The target velocity reference.
 * @param[in]     ref_acc The target acceleration feedforward.
 * @param[in]     fbk_pos The current actual POSITION feedback.
 * @return ctrl_gt The calculated controller output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_ladrc2(ctl_ladrc2_t* ladrc, ctrl_gt ref_pos, ctrl_gt ref_vel, ctrl_gt ref_acc,
                                          ctrl_gt fbk_pos)
{
    // 1. 3rd-Order LESO (Linear Extended State Observer) Update
    // Error between estimated position and actual position feedback
    ctrl_gt e = ladrc->z1 - fbk_pos;

    // Calculate next states using pre-absorbed parameters
    // z1(k+1) = z1(k) + h * z2(k) - h_beta1 * e(k)
    ctrl_gt z1_next = ladrc->z1 + ctl_mul(ladrc->h, ladrc->z2) - ctl_mul(ladrc->h_beta1, e);

    // z2(k+1) = z2(k) + h_b0 * (Z3(k) + u(k-1)) - h_beta2 * e(k)
    ctrl_gt z2_next = ladrc->z2 + ctl_mul(ladrc->h_b0, ladrc->z3_u + ladrc->u_prev) - ctl_mul(ladrc->h_beta2, e);

    // z3_u(k+1) = z3_u(k) - h_beta3_b0 * e(k)
    ctrl_gt z3_u_next = ladrc->z3_u - ctl_mul(ladrc->h_beta3_b0, e);

    // Update observer states
    ladrc->z1 = z1_next;
    ladrc->z2 = z2_next;
    ladrc->z3_u = z3_u_next;

    // 2. LSEF (Linear State Error Feedback) & Disturbance Compensation
    // u = (Kpp/b0)*(ref_pos - z1) + (Kvp/b0)*(ref_vel - z2) + (1/b0)*ref_acc - z3_u
    ctrl_gt p_term = ctl_mul(ladrc->kpp_b0, ref_pos - ladrc->z1);
    ctrl_gt v_term = ctl_mul(ladrc->kvp_b0, ref_vel - ladrc->z2);
    ctrl_gt ff_term = ctl_mul(ladrc->b0_inv, ref_acc);

    ctrl_gt u = p_term + v_term + ff_term - ladrc->z3_u;

    // 3. Output Saturation (Implicit Anti-Windup)
    ladrc->out = ctl_sat(u, ladrc->out_max, ladrc->out_min);
    ladrc->u_prev = ladrc->out;

    return ladrc->out;
}

/**
 * @brief Gets the estimated velocity (z2) from the observer.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_ladrc2_velocity(const ctl_ladrc2_t* ladrc)
{
    return ladrc->z2;
}

/**
 * @brief Gets the estimated total disturbance in terms of input equivalent (PU).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_ladrc2_disturbance_pu(const ctl_ladrc2_t* ladrc)
{
    return ladrc->z3_u;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_LADRC2_H_
