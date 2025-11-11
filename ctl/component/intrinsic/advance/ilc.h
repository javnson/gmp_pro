/**
 * @file ilc.h
 * @brief Implements an Iterative Learning Controller (ILC) for SISO systems.
 *
 * @version 1.0
 * @date 2025-08-07
 *

 *
 */

#ifndef _FILE_ITERATIVE_LEARNING_CTRL_H_
#define _FILE_ITERATIVE_LEARNING_CTRL_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Iterative Learning Controller (ILC)                                       */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup ILC_CONTROLLER Iterative Learning Controller (ILC)
 * @brief A controller that improves performance on repetitive tasks by learning from past errors.
 * @details This module provides a controller designed for systems that perform
 * repetitive tasks over a fixed duration. ILC improves tracking performance
 * by learning from the error of the previous iteration (or trial) and
 * updating the control signal for the next iteration. This allows the
 * controller to cancel out periodic disturbances and unmodeled dynamics,
 * achieving very high precision tracking.
 * The controller implements a P-type ILC update law:
 * @f[ u_k(t) = u_{k-1}(t) + L \cdot e_{k-1}(t) @f]
 * where k is the iteration number, t is the time step within the iteration,
 * u is the control signal, e is the tracking error @f( (r - y) @f), and L is the
 * learning gain. The output @f( u_k(t) @f) is typically used as a feed forward term
 * in conjunction with a feedback controller (e.g., PI).
 * @{
 */

//================================================================================
// Type Defines
//================================================================================

/**
 * @brief Initialization parameters for the ILC module.
 */
typedef struct
{
    // --- Controller Parameters ---
    parameter_gt learning_gain; ///< The learning gain (L).
    uint32_t trajectory_length; ///< The total number of time steps (N) in one iteration.

    // --- Trajectory Buffers (provided by user) ---
    ctrl_gt* u_k_buffer;         ///< Pointer to a buffer to store the current iteration's control signal.
    ctrl_gt* u_k_minus_1_buffer; ///< Pointer to a buffer storing the previous iteration's control signal.
    ctrl_gt* e_k_minus_1_buffer; ///< Pointer to a buffer storing the previous iteration's error signal.

} ctl_ilc_init_t;

/**
 * @brief Main structure for the ILC controller.
 */
typedef struct
{
    // --- Output ---
    ctrl_gt u_out; ///< The calculated control output for the current time step.

    // --- Buffers ---
    ctrl_gt* u_k;         ///< Stores the control signal for the current iteration.
    ctrl_gt* u_k_minus_1; ///< Stores the control signal from the previous iteration.
    ctrl_gt* e_k_minus_1; ///< Stores the error from the previous iteration.

    // --- State Variables ---
    uint32_t time_step;  ///< The current time index within the trajectory (0 to N-1).
    uint8_t is_learning; ///< Flag to enable/disable the learning process.

    // --- Parameters ---
    ctrl_gt learning_gain;      ///< Learning gain (L).
    uint32_t trajectory_length; ///< Total number of steps in the trajectory (N).

} ctl_ilc_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the ILC module.
 * @details Assigns the user-provided buffers and sets the controller parameters.
 * @param[out] ilc  Pointer to the ILC structure.
 * @param[in]  init Pointer to the initialization parameters.
 */
void ctl_init_ilc(ctl_ilc_controller_t* ilc, const ctl_ilc_init_t* init);

/**
 * @brief Resets the ILC controller to its initial state.
 * @details This function clears all stored trajectory data and resets the time step.
 * It should be called before starting a new learning sequence from scratch.
 * @param[out] ilc Pointer to the ILC structure.
 */
GMP_STATIC_INLINE void ctl_clear_ilc(ctl_ilc_controller_t* ilc)
{
    uint32_t i;

    ilc->u_out = 0.0f;
    ilc->time_step = 0;
    ilc->is_learning = 0; // Learning is disabled by default

    for (i = 0; i < ilc->trajectory_length; ++i)
    {
        ilc->u_k[i] = 0.0f;
        ilc->u_k_minus_1[i] = 0.0f;
        ilc->e_k_minus_1[i] = 0.0f;
    }
}

/**
 * @brief Prepares the controller for the next iteration.
 * @details This function should be called exactly once at the end of each completed
 * trajectory. It updates the stored control and error signals for the next run.
 * @param[out] ilc Pointer to the ILC structure.
 */
GMP_STATIC_INLINE void ctl_start_new_iteration(ctl_ilc_controller_t* ilc)
{
    uint32_t i;

    // The control signal from the completed iteration becomes the base for the next one.
    // The error from the completed iteration is now the error from the "previous" run.
    for (i = 0; i < ilc->trajectory_length; ++i)
    {
        ilc->u_k_minus_1[i] = ilc->u_k[i];
        // The error buffer e_k_minus_1 is now ready to be used for the new iteration.
    }

    // Reset the time step to the beginning of the trajectory
    ilc->time_step = 0;
}

/**
 * @brief Executes one step of the Iterative Learning Control algorithm.
 * @param[out] ilc    Pointer to the ILC structure.
 * @param[in]  r      The reference command for the current time step.
 * @param[in]  y_p    The measured output from the plant for the current time step.
 * @return The calculated control signal `u` for the current time step.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_ilc(ctl_ilc_controller_t* ilc, ctrl_gt r, ctrl_gt y_p)
{
    if (ilc->time_step >= ilc->trajectory_length)
    {
        // Trajectory is finished for this iteration, hold the last output
        return ilc->u_out;
    }

    // 1. Calculate the ILC update law
    if (ilc->is_learning)
    {
        // u_k(t) = u_{k-1}(t) + L * e_{k-1}(t)
        ilc->u_out = ilc->u_k_minus_1[ilc->time_step] + ctl_mul(ilc->learning_gain, ilc->e_k_minus_1[ilc->time_step]);
    }
    else
    {
        // If learning is disabled, just use the control signal from the last converged iteration
        ilc->u_out = ilc->u_k_minus_1[ilc->time_step];
    }

    // 2. Calculate and store the error for the *next* iteration
    ctrl_gt current_error = r - y_p;
    ilc->e_k_minus_1[ilc->time_step] = current_error; // This will be e_{k-1} in the next run

    // 3. Store the control signal for the *next* iteration
    ilc->u_k[ilc->time_step] = ilc->u_out;

    // 4. Advance the time step
    ilc->time_step++;

    return ilc->u_out;
}

/**
 * @brief Enables the learning process.
 * @param[out] ilc Pointer to the ILC structure.
 */
GMP_STATIC_INLINE void ctl_enable_ilc_learning(ctl_ilc_controller_t* ilc)
{
    ilc->is_learning = 1;
}

/**
 * @brief Disables the learning process.
 * @details When disabled, the ILC acts as a feedforward controller using the
 * last learned trajectory. This is useful after the error has converged.
 * @param[out] ilc Pointer to the ILC structure.
 */
GMP_STATIC_INLINE void ctl_disable_ilc_learning(ctl_ilc_controller_t* ilc)
{
    ilc->is_learning = 0;
}

/** @} */ // end of ILC_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ITERATIVE_LEARNING_CTRL_H_
