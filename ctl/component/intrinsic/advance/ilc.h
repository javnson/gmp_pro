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
typedef struct _tag_ilc_t
{
    ctrl_gt* u_k;         //!< Pointer to buffer for current control signal
    ctrl_gt* u_k_minus_1; //!< Pointer to buffer for previous control signal
    ctrl_gt* e_k_minus_1; //!< Pointer to buffer for previous error signal

    ctrl_gt learning_gain;      //!< Learning gain (L)
    uint32_t trajectory_length; //!< Total number of samples in one iteration
    uint32_t time_step;         //!< Current sample index
    fast_gt is_learning;        //!< Flag to enable/disable learning
    ctrl_gt u_out;              //!< Current output
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
    ilc->u_out = float2ctrl(0.0f);
    ilc->time_step = 0;
}

/**
 * @brief Prepares the controller for the next iteration.
 * @details This function should be called exactly once at the end of each completed
 * trajectory. It updates the stored control and error signals for the next run.
 * @param[out] ilc Pointer to the ILC structure.
 */
//GMP_STATIC_INLINE void ctl_start_new_iteration(ctl_ilc_controller_t* ilc)
//{
//    uint32_t i;
//
//    // The control signal from the completed iteration becomes the base for the next one.
//    // The error from the completed iteration is now the error from the "previous" run.
//    for (i = 0; i < ilc->trajectory_length; ++i)
//    {
//        ilc->u_k_minus_1[i] = ilc->u_k[i];
//        // The error buffer e_k_minus_1 is now ready to be used for the new iteration.
//    }
//
//    // Reset the time step to the beginning of the trajectory
//    ilc->time_step = 0;
//}

/**
 * @brief Executes one step of the Iterative Learning Control algorithm.
 * @param[out] ilc    Pointer to the ILC structure.
 * @param[in]  r      The reference command for the current time step.
 * @param[in]  y_p    The measured output from the plant for the current time step.
 * @return The calculated control signal `u` for the current time step.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_ilc(ctl_ilc_controller_t* ilc, ctrl_gt r, ctrl_gt y_p)
{
    // 修复 2：极度关键的内存越界保护 (Buffer Overflow Protection)
    if (ilc->time_step >= ilc->trajectory_length)
    {
        // 轨迹已结束，拒绝越界访问数组，直接返回最后的输出
        return ilc->u_out;
    }

    if (ilc->is_learning)
    {
        // u_k(t) = u_{k-1}(t) + L * e_{k-1}(t)
        ilc->u_out = ilc->u_k_minus_1[ilc->time_step] + ctl_mul(ilc->learning_gain, ilc->e_k_minus_1[ilc->time_step]);
    }
    else
    {
        ilc->u_out = ilc->u_k_minus_1[ilc->time_step];
    }

    // Calculate and store the error for the *next* iteration
    ctrl_gt current_error = r - y_p;
    ilc->e_k_minus_1[ilc->time_step] = current_error;

    // Store the control signal for the *next* iteration
    ilc->u_k[ilc->time_step] = ilc->u_out;

    // Advance the time step
    ilc->time_step++;

    return ilc->u_out;
}

/**
 * @brief Completes the current iteration and prepares for the next one (Epoch Swap).
 * @details 修复 3：通过指针交换实现 O(1) 的极速“世代交替”，必须在每次轨迹周期结束时调用。
 */
GMP_STATIC_INLINE void ctl_finish_ilc_iteration(ctl_ilc_controller_t* ilc)
{
    // Swap the pointers: current control u_k becomes previous u_k_minus_1
    ctrl_gt* temp_ptr = ilc->u_k_minus_1;
    ilc->u_k_minus_1 = ilc->u_k;
    ilc->u_k = temp_ptr; // The old u_k_minus_1 becomes the new working buffer for u_k

    // Reset the time step for the new iteration
    ilc->time_step = 0;
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
