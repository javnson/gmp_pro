/**
 * @file internal_model_ctrl.h
 * @brief Implements an Internal Model Controller (IMC) for SISO systems.
 * @details This module provides a robust controller based on the IMC principle.
 * It is designed for systems that can be approximated by a First-Order Plus
 * Dead-Time (FOPDT) model. The controller uses an internal model of the plant
 * to predict its response. The difference between the actual plant output and
 * the model's output (the disturbance) is used to generate a corrective
 * control action. This structure provides excellent setpoint tracking and
 * disturbance rejection.
 *
 * @version 1.0
 * @date 2025-08-07
 *
 * //tex:
 * // Plant Model (FOPDT): G_p(s) = \frac{K_p e^{-\theta_p s}}{\tau_p s + 1}
 * // IMC Controller: Q(s) = \frac{1}{K_p} \frac{\tau_p s + 1}{\lambda s + 1}
 * // The overall control law is: u(s) = Q(s) (r(s) - d(s))
 * // where d(s) is the estimated disturbance: d(s) = y_p(s) - y_m(s)
 *
 */

#ifndef _FILE_INTERNAL_MODEL_CTRL_H_
#define _FILE_INTERNAL_MODEL_CTRL_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Internal Model Controller (IMC)                                           */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup IMC_CONTROLLER Internal Model Controller (IMC)
 * @brief A robust model-based controller for SISO systems.
 * @{
 */

//================================================================================
// Type Defines
//================================================================================

#define IMC_MAX_DEAD_TIME_SAMPLES (50) // Maximum allowable dead time in samples

/**
 * @brief Initialization parameters for the IMC module.
 */
typedef struct
{
    // --- Plant Model Parameters (FOPDT) ---
    parameter_gt K_p;     ///< Plant gain.
    parameter_gt tau_p;   ///< Plant time constant (s).
    parameter_gt theta_p; ///< Plant dead time (s).

    // --- Controller Tuning ---
    parameter_gt lambda; ///< Desired closed-loop time constant (filter tuning parameter).

    // --- System Parameters ---
    parameter_gt f_ctrl; ///< Controller execution frequency (Hz).

} ctl_imc_init_t;

/**
 * @brief Main structure for the IMC controller.
 */
typedef struct
{
    // --- Output ---
    ctrl_gt u_out; ///< The calculated control output.

    // --- Internal Model State ---
    ctrl_gt y_m;                                       ///< The output of the internal plant model.
    ctrl_gt u_delay_buffer[IMC_MAX_DEAD_TIME_SAMPLES]; ///< Buffer for simulating dead time.
    uint16_t dead_time_samples;                        ///< Number of samples for dead time.
    uint16_t delay_buffer_idx;                         ///< Current index in the delay buffer.

    // --- Controller (Q) State ---
    ctrl_gt q_in_1;  ///< Previous input to the Q controller.
    ctrl_gt q_out_1; ///< Previous output from the Q controller.

    // --- Pre-calculated Parameters ---
    ctrl_gt a_p_d;  ///< Discrete-time pole for the plant model.
    ctrl_gt b_p_d;  ///< Discrete-time gain for the plant model.
    ctrl_gt a_q_d;  ///< Discrete-time pole for the Q controller.
    ctrl_gt b0_q_d; ///< Discrete-time numerator coefficient 0 for Q.
    ctrl_gt b1_q_d; ///< Discrete-time numerator coefficient 1 for Q.

} ctl_imc_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the IMC module.
 * @details Calculates the discrete-time equivalents of the plant model and the
 * IMC controller Q based on the continuous-time FOPDT parameters.
 * @param[out] imc  Pointer to the IMC structure.
 * @param[in]  init Pointer to the initialization parameters.
 * @return 0 on success, -1 on error (e.g., dead time too long).
 */
GMP_STATIC_INLINE int ctl_init_imc(ctl_imc_controller_t* imc, const ctl_imc_init_t* init)
{
    ctrl_gt Ts = 1.0f / (ctrl_gt)init->f_ctrl;

    // --- Discretize Plant Model (ZOH) ---
    imc->a_p_d = expf(-(Ts / (ctrl_gt)init->tau_p));
    imc->b_p_d = (ctrl_gt)init->K_p * (1.0f - imc->a_p_d);

    // --- Calculate Dead Time ---
    imc->dead_time_samples = (uint16_t)roundf((ctrl_gt)init->theta_p / Ts);
    if (imc->dead_time_samples >= IMC_MAX_DEAD_TIME_SAMPLES)
    {
        return -1; // Error: Dead time exceeds buffer size
    }

    // --- Discretize IMC Controller Q(s) (Tustin's method) ---
    // Q(s) = (1/Kp) * (tau_p*s + 1) / (lambda*s + 1)
    ctrl_gt lambda = (ctrl_gt)init->lambda;
    ctrl_gt tau_p = (ctrl_gt)init->tau_p;
    ctrl_gt K_p = (ctrl_gt)init->K_p;

    imc->a_q_d = (2.0f * lambda - Ts) / (2.0f * lambda + Ts);
    imc->b0_q_d = (1.0f / K_p) * (2.0f * tau_p + Ts) / (2.0f * lambda + Ts);
    imc->b1_q_d = (1.0f / K_p) * (Ts - 2.0f * tau_p) / (2.0f * lambda + Ts);

    // Reset states
    ctl_clear_imc(imc);
    return 0;
}

/**
 * @brief Resets the internal states of the IMC controller.
 * @param[out] imc Pointer to the IMC structure.
 */
GMP_STATIC_INLINE void ctl_clear_imc(ctl_imc_controller_t* imc)
{
    imc->u_out = 0.0f;
    imc->y_m = 0.0f;
    imc->q_in_1 = 0.0f;
    imc->q_out_1 = 0.0f;
    imc->delay_buffer_idx = 0;
    for (int i = 0; i < IMC_MAX_DEAD_TIME_SAMPLES; ++i)
    {
        imc->u_delay_buffer[i] = 0.0f;
    }
}

/**
 * @brief Executes one step of the Internal Model Control algorithm.
 * @param[out] imc    Pointer to the IMC structure.
 * @param[in]  r      The reference command for the system.
 * @param[in]  y_p    The measured output from the plant.
 * @return The calculated control signal `u` to be sent to the plant.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_imc(ctl_imc_controller_t* imc, ctrl_gt r, ctrl_gt y_p)
{
    // 1. Get the delayed input for the plant model from the buffer
    uint16_t read_idx =
        (imc->delay_buffer_idx + IMC_MAX_DEAD_TIME_SAMPLES - imc->dead_time_samples) % IMC_MAX_DEAD_TIME_SAMPLES;
    ctrl_gt u_delayed = imc->u_delay_buffer[read_idx];

    // 2. Update the internal plant model
    // y_m(k) = a_p_d * y_m(k-1) + b_p_d * u(k-d-1)
    imc->y_m = imc->a_p_d * imc->y_m + imc->b_p_d * u_delayed;

    // 3. Calculate the model error (disturbance estimate)
    ctrl_gt model_error = r - (y_p - imc->y_m);

    // 4. Pass the error through the Q controller
    // u_q(k) = a_q_d*u_q(k-1) + b0_q_d*e(k) + b1_q_d*e(k-1)
    ctrl_gt u_q = imc->a_q_d * imc->q_out_1 + imc->b0_q_d * model_error + imc->b1_q_d * imc->q_in_1;

    // Update Q controller state for next iteration
    imc->q_in_1 = model_error;
    imc->q_out_1 = u_q;

    // The output of Q is the new control signal
    imc->u_out = u_q;

    // 5. Store the new control output in the delay buffer
    imc->u_delay_buffer[imc->delay_buffer_idx] = imc->u_out;
    imc->delay_buffer_idx = (imc->delay_buffer_idx + 1) % IMC_MAX_DEAD_TIME_SAMPLES;

    return imc->u_out;
}

/** @} */ // end of IMC_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_INTERNAL_MODEL_CTRL_H_
