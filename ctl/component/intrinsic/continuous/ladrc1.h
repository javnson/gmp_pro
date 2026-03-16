/**
 * @file ladrc1.h
 * @brief Implements a highly optimized, Fixed-Point friendly 1st-Order LADRC.
 * @warning To maintain stability with Euler discretization, ensure the sampling frequency (fs) is at least 10 times the observer bandwidth (fo).
 * 
 * @version 1.1
 * @date 2024-10-26
 *
 */

#ifndef _FILE_CTL_LADRC1_H_
#define _FILE_CTL_LADRC1_H_

#include <ctl/math_block/gmp_math.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup LADRC1 1st-Order LADRC
 * @brief A generic 1st-order Linear Active Disturbance Rejection Controller.
 * @details Highly optimized for fixed-point execution. The sample time (h) and 
 * system gain (b0) are absorbed into pre-calculated coefficients. The disturbance 
 * state is normalized to the input scale (z2_u = z2 / b0) to prevent overflow 
 * and eliminate division in the real-time step function.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* 1st-Order LADRC Controller                                                */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the 1st-Order LADRC execution.
 */
typedef struct _tag_ladrc1_t
{
    // --- Pre-calculated Absorbed Parameters (PU / Scaled) ---
    ctrl_gt h_b0;       //!< h * b0
    ctrl_gt h_beta1;    //!< h * 2 * wo
    ctrl_gt h_beta2_b0; //!< h * (wo^2) / b0
    ctrl_gt kp_b0;      //!< wc / b0

    // --- Limits ---
    ctrl_gt out_max; //!< Maximum output limit (Default: 1.0 PU).
    ctrl_gt out_min; //!< Minimum output limit (Default: -1.0 PU).

    // --- State Variables ---
    ctrl_gt z1;     //!< Estimated system state (filtered feedback).
    ctrl_gt z2_u;   //!< Estimated total disturbance EQUIVALENT input (z2 / b0).
    ctrl_gt u_prev; //!< Controller output from the previous step u(k-1).
    ctrl_gt out;    //!< Current controller output.

} ctl_ladrc1_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

/**
 * @brief Dynamically upgrades/updates the LADRC parameters seamlessly.
 * @details Recalculates all internal coefficients based on new bandwidths.
 * States are preserved to guarantee bumpless parameter transition.
 * @param[out] ladrc Pointer to the LADRC instance.
 * @param[in]  b0    Estimated system gain.
 * @param[in]  fc    Controller bandwidth (Hz).
 * @param[in]  fo    Observer bandwidth (Hz).
 * @param[in]  fs    Controller execution frequency (Hz).
 */
void ctl_upgrade_ladrc1(ctl_ladrc1_t* ladrc, parameter_gt b0, parameter_gt fc, parameter_gt fo, parameter_gt fs);

/**
 * @brief Initializes the 1st-order LADRC controller.
 * @details Sets up default limits (¡À1) and clears internal states.
 * @param[out] ladrc Pointer to the LADRC instance.
 * @param[in]  b0    Estimated system gain.
 * @param[in]  fc    Controller bandwidth (Hz).
 * @param[in]  fo    Observer bandwidth (Hz).
 * @param[in]  fs    Controller execution frequency (Hz).
 */
void ctl_init_ladrc1(ctl_ladrc1_t* ladrc, parameter_gt b0, parameter_gt fc, parameter_gt fo, parameter_gt fs);

/**
 * @brief Clears the internal observer states of the LADRC controller.
 */
GMP_STATIC_INLINE void ctl_clear_ladrc1(ctl_ladrc1_t* ladrc)
{
    ladrc->z1 = float2ctrl(0.0f);
    ladrc->z2_u = float2ctrl(0.0f);
    ladrc->u_prev = float2ctrl(0.0f);
    ladrc->out = float2ctrl(0.0f);
}

/**
 * @brief Sets the output saturation limits.
 */
GMP_STATIC_INLINE void ctl_set_ladrc1_limit(ctl_ladrc1_t* ladrc, ctrl_gt out_max, ctrl_gt out_min)
{
    ladrc->out_max = out_max;
    ladrc->out_min = out_min;
}

/**
 * @brief Pre-loads the observer states for bumpless transfer mode switching.
 */
GMP_STATIC_INLINE void ctl_set_ladrc1_states(ctl_ladrc1_t* ladrc, ctrl_gt current_fbk, ctrl_gt current_u)
{
    ladrc->z1 = current_fbk;
    ladrc->u_prev = current_u;
}

/**
 * @brief Executes one step of the 1st-Order LADRC.
 * @details Executed purely with additions and pre-scaled multiplications.
 * @param[in,out] ladrc Pointer to the LADRC instance.
 * @param[in]     ref   The target reference value.
 * @param[in]     fbk   The current actual feedback value.
 * @return ctrl_gt The calculated controller output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_ladrc1(ctl_ladrc1_t* ladrc, ctrl_gt ref, ctrl_gt fbk)
{
    // 1. LESO (Linear Extended State Observer) Update
    // e = z1 - y
    ctrl_gt e = ladrc->z1 - fbk;

    // Calculate next states using pre-absorbed parameters
    // z1(k+1) = z1(k) + h_b0 * (Z2(k) + u(k-1)) - h_beta1 * e(k)
    ctrl_gt z1_next = ladrc->z1 + ctl_mul(ladrc->h_b0, ladrc->z2_u + ladrc->u_prev) - ctl_mul(ladrc->h_beta1, e);

    // z2_u(k+1) = z2_u(k) - h_beta2_b0 * e(k)
    ctrl_gt z2_u_next = ladrc->z2_u - ctl_mul(ladrc->h_beta2_b0, e);

    // Update observer states
    ladrc->z1 = z1_next;
    ladrc->z2_u = z2_u_next;

    // 2. LSEF (Linear State Error Feedback) & Disturbance Compensation
    // u = Kp_b0 * (ref - z1) - z2_u
    ctrl_gt u0 = ctl_mul(ladrc->kp_b0, ref - ladrc->z1);
    ctrl_gt u = u0 - ladrc->z2_u;

    // 3. Output Saturation (Implicit Anti-Windup)
    // By saving the saturated output to u_prev, the observer naturally understands
    // the system bounds and prevents integral windup of the disturbance state.
    ladrc->out = ctl_sat(u, ladrc->out_max, ladrc->out_min);
    ladrc->u_prev = ladrc->out;

    return ladrc->out;
}

/**
 * @brief Gets the estimated total disturbance in terms of input equivalent (PU).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_ladrc1_disturbance_pu(const ctl_ladrc1_t* ladrc)
{
    return ladrc->z2_u;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_LADRC1_H_
