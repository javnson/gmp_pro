/**
 * @file ladrc_current_controller.h
 * @brief Implements a Per-Unit Linear Active Disturbance Rejection Controller (LADRC) for a motor current loop.
 * @details This module provides a robust current controller based on the LADRC
 * strategy. It uses a Linear Extended State Observer (LESO) to estimate and
 * actively compensate for total system disturbances, including parameter
 * variations (R, L), back-EMF, and other unmodeled dynamics. This makes the
 * controller highly robust and easy to tune.
 *
 * The controller is designed to operate entirely within the per-unit system.
 *
 * @version 1.0
 * @date 2025-08-06
 *
 * //tex:
 * // Plant Model: \frac{di_{pu}}{dt} = b_0 u_{pu} + f_{pu}
 * // Where f_{pu} is the total disturbance and b_0 = \frac{\omega_{base}}{L_{pu}}
 * //
 * // LESO (z1 -> i_pu, z2 -> f_pu):
 * // e = z_1 - i_{pu}
 * // \dot{z_1} = z_2 - 2\omega_o e + b_0 u_{pu}
 * // \dot{z_2} = -\omega_o^2 e
 * //
 * // Control Law:
 * // u_0 = \omega_c (r_{pu} - z_1)
 * // u_{pu} = \frac{u_0 - z_2}{b_0}
 *
 */

#ifndef _FILE_LADRC_CURRENT_CONTROLLER_H_
#define _FILE_LADRC_CURRENT_CONTROLLER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* LADRC Current Controller (Per-Unit)                                       */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup LADRC_CURRENT_PU LADRC Current Controller (Per-Unit)
 * @brief A robust, per-unit current controller using Active Disturbance Rejection.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Main structure for the LADRC per-unit current controller.
 */
typedef struct
{
    // --- Observer States ---
    ctrl_gt z1; ///< Estimated state (current, p.u.).
    ctrl_gt z2; ///< Estimated total disturbance (p.u. current/sec).

    // --- Controller Parameters ---
    parameter_gt wc; ///< Controller bandwidth (rad/s).
    parameter_gt wo; ///< Observer bandwidth (rad/s).
    parameter_gt b0; ///< System input gain (omega_base / L_pu).
    parameter_gt h;  ///< Sampling period (s).

    // --- Internal State ---
    ctrl_gt u_out_pu; ///< Stored output from the last step (p.u.).

} ctl_ladrc_current_pu_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the LADRC per-unit current controller.
 * @param[out] ladrc Pointer to the LADRC controller structure.
 * @param[in]  wc_rads Desired closed-loop controller bandwidth (rad/s).
 * @param[in]  wo_rads Desired observer bandwidth (rad/s). Typically 3-5x wc.
 * @param[in]  L_pu Motor inductance for the axis being controlled (p.u.).
 * @param[in]  omega_base The base electrical frequency of the per-unit system (rad/s).
 * @param[in]  sample_time_s The controller's sampling period in seconds.
 */
void ctl_init_ladrc_current_pu(ctl_ladrc_current_pu_t* ladrc, parameter_gt wc_rads, parameter_gt wo_rads,
                               parameter_gt L_pu, parameter_gt omega_base, parameter_gt sample_time_s);

/**
 * @brief Resets the internal states of the LADRC controller.
 * @param[out] ladrc Pointer to the LADRC controller structure.
 */
GMP_STATIC_INLINE void ctl_reset_ladrc_pu(ctl_ladrc_current_pu_t* ladrc)
{
    ladrc->z1 = 0.0f;
    ladrc->z2 = 0.0f;
    ladrc->u_out_pu = 0.0f;
}

/**
 * @brief Executes one step of the LADRC current controller.
 * @param[in,out] ladrc Pointer to the LADRC controller structure.
 * @param[in]  r_pu The current reference command (p.u.).
 * @param[in]  y_pu The measured feedback current (p.u.).
 * @return The calculated control voltage (p.u.).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_ladrc_current_pu(ctl_ladrc_current_pu_t* ladrc, ctrl_gt r_pu, ctrl_gt y_pu)
{
    // --- State Feedback & Disturbance Compensation ---
    // Calculate the preliminary control law based on the estimated states from the *previous* time step.
    ctrl_gt u0 = ladrc->wc * (r_pu - ladrc->z1);

    // Compensate for the total disturbance and apply the system gain.
    if (fabsf(ladrc->b0) > 1e-9f)
    {
        ladrc->u_out_pu = (u0 - ladrc->z2) / ladrc->b0;
    }
    else
    {
        ladrc->u_out_pu = 0.0f;
    }

    // --- Linear Extended State Observer (LESO) ---
    // Update the observer for the *next* time step using the output from *this* step.
    ctrl_gt error = ladrc->z1 - y_pu;
    ctrl_gt beta1 = 2.0f * ladrc->wo;
    ctrl_gt beta2 = ladrc->wo * ladrc->wo;

    ladrc->z1 = ladrc->z1 + ladrc->h * (ladrc->z2 - beta1 * error + ladrc->b0 * ladrc->u_out_pu);
    ladrc->z2 = ladrc->z2 + ladrc->h * (-beta2 * error);

    return ladrc->u_out_pu;
}

/**
 * @brief Gets the last calculated control output.
 * @param[in] ladrc Pointer to the LADRC controller structure.
 * @return The last control voltage output (p.u.).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_ladrc_output_pu(const ctl_ladrc_current_pu_t* ladrc)
{
    return ladrc->u_out_pu;
}

/**
 * @}
 */ // end of LADRC_CURRENT_PU group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_LADRC_CURRENT_CONTROLLER_H_
