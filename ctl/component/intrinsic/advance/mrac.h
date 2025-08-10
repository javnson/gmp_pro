/**
 * @file adaptive_ctrl.h
 * @brief Implements a Model Reference Adaptive Controller (MRAC) for SISO systems.
 * @details This module provides an adaptive controller designed to make a plant's
 * output track the output of a stable reference model, even when the plant's
 * parameters are unknown or time-varying. It is suitable for applications like
 * a motor speed loop where inertia and friction can change.
 *
 * The controller continuously adjusts its gains based on the tracking error
 * to ensure the system behaves as desired.
 *
 * @version 1.0
 * @date 2025-08-07
 *
 * //tex:
 * // Reference Model: \dot{y}_m = -a_m y_m + b_m r
 * // Plant Model: \dot{y}_p = -a_p y_p + b_p u
 * // Control Law: u = k_r r - k_y y_p
 * // Adaptation Law (MIT Rule):
 * // \dot{k}_r = -\gamma_r e r
 * // \dot{k}_y = \gamma_y e y_p
 * // where e = y_p - y_m is the tracking error.
 *
 */

#ifndef _FILE_ADAPTIVE_CTRL_H_
#define _FILE_ADAPTIVE_CTRL_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Model Reference Adaptive Controller (MRAC)                                */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup ADAPTIVE_CONTROLLER Model Reference Adaptive Controller (MRAC)
 * @brief An adaptive controller for SISO systems with unknown or varying parameters.
 * @{
 */

//================================================================================
// Type Defines
//================================================================================

/**
 * @brief Initialization parameters for the MRAC module.
 */
typedef struct
{
    // --- Reference Model Parameters (Continuous Time) ---
    parameter_gt a_m; ///< The pole of the reference model (determines speed of response).
    parameter_gt b_m; ///< The gain of the reference model.

    // --- Adaptation Gains ---
    parameter_gt gamma_r; ///< Adaptation rate for the reference gain (k_r).
    parameter_gt gamma_y; ///< Adaptation rate for the feedback gain (k_y).

    // --- System Parameters ---
    parameter_gt f_ctrl; ///< Controller execution frequency (Hz).

} ctl_mrac_init_t;

/**
 * @brief Main structure for the MRAC controller.
 */
typedef struct
{
    // --- Outputs ---
    ctrl_gt u_out; ///< The calculated control output.

    // --- Adaptive Gains (State) ---
    ctrl_gt k_r; ///< The adaptive gain for the reference signal.
    ctrl_gt k_y; ///< The adaptive gain for the plant output feedback.

    // --- Reference Model State ---
    ctrl_gt y_m; ///< The output of the reference model.

    // --- Pre-calculated Parameters ---
    ctrl_gt a_m_d;     ///< Discrete-time pole for the reference model.
    ctrl_gt b_m_d;     ///< Discrete-time gain for the reference model.
    ctrl_gt gamma_r_d; ///< Discrete-time adaptation rate for k_r.
    ctrl_gt gamma_y_d; ///< Discrete-time adaptation rate for k_y.

} ctl_mrac_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the MRAC module.
 * @details Pre-calculates the discrete-time equivalents of the model and
 * adaptation laws from the continuous-time parameters.
 * @param[out] mrac Pointer to the MRAC structure.
 * @param[in]  init Pointer to the initialization parameters.
 */
void ctl_init_mrac(ctl_mrac_controller_t* mrac, const ctl_mrac_init_t* init);


/**
 * @brief Resets the internal states of the MRAC controller.
 * @details This is useful for re-starting the controller. It resets the adaptive
 * gains and the reference model state.
 * @param[out] mrac Pointer to the MRAC structure.
 */
GMP_STATIC_INLINE void ctl_clear_mrac(ctl_mrac_controller_t* mrac)
{
    mrac->u_out = 0.0f;
    mrac->k_r = 0.0f;
    mrac->k_y = 0.0f;
    mrac->y_m = 0.0f;
}

/**
 * @brief Executes one step of the Model Reference Adaptive Control algorithm.
 * @param[out] mrac   Pointer to the MRAC structure.
 * @param[in]  r      The reference command for the system.
 * @param[in]  y_p    The measured output from the plant (e.g., actual motor speed).
 * @return The calculated control signal `u` to be sent to the plant.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_mrac(ctl_mrac_controller_t* mrac, ctrl_gt r, ctrl_gt y_p)
{
    // 1. Update the reference model to get the desired output for this step
    // y_m(k) = a_m_d * y_m(k-1) + b_m_d * r(k-1)
    mrac->y_m = mrac->a_m_d * mrac->y_m + mrac->b_m_d * r;

    // 2. Calculate the tracking error
    ctrl_gt error = y_p - mrac->y_m;

    // 3. Update the adaptive gains using the discretized MIT rule
    // k(k) = k(k-1) - gamma * error * signal
    mrac->k_r -= mrac->gamma_r_d * error * r;
    mrac->k_y += mrac->gamma_y_d * error * y_p;

    // 4. Calculate the control law
    mrac->u_out = mrac->k_r * r - mrac->k_y * y_p;

    return mrac->u_out;
}

/**
 * @}
 */ // end of ADAPTIVE_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ADAPTIVE_CTRL_H_
