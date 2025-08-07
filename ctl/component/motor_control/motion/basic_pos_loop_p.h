/**
 * @file basic_pos_loop_p.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements a basic proportional (P) position controller.
 * @details This module provides a simple and effective position controller. It takes
 * a target position (composed of full revolutions and an angle) and a feedback
 * position, calculates the error, and multiplies it by a proportional gain (Kp)
 * to produce a speed reference. The output is saturated to a defined limit.
 * This controller forms the outermost loop in a cascaded position control system.
 *
 * @version 0.2
 * @date 2025-08-06
 *
 * @copyright Copyright GMP(c) 2024
 */

#ifndef _FILE_BASIC_POS_LOOP_P_H_
#define _FILE_BASIC_POS_LOOP_P_H_

#include <ctl/component/intrinsic/discrete/divider.h>
#include <ctl/math_block/gmp_math.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Basic Proportional Position Controller                                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup POSITION_CONTROLLER Basic Position Controller
 * @brief A simple P-controller for a position loop.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Main structure for the basic position P-controller.
 */
typedef struct
{
    // --- Configuration ---
    ctrl_gt kp;          ///< Proportional gain for the position controller.
    ctrl_gt speed_limit; ///< Maximum output speed reference (saturation limit).

    // --- Target & Feedback ---
    int32_t target_revs;  ///< The integer part of the target position (full revolutions).
    ctrl_gt target_angle; ///< The fractional part of the target position (0.0 to 1.0).
    int32_t actual_revs;  ///< The integer part of the feedback position.
    ctrl_gt actual_angle; ///< The fractional part of the feedback position.

    // --- Output ---
    ctrl_gt speed_ref; ///< The output speed reference fed to the velocity loop.

    // --- Execution Control ---
    ctl_divider_t div; ///< Divider to run the position loop at a lower frequency.

} ctl_pos_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the position controller structure to safe defaults.
 * @param[out] pc Pointer to the position controller structure.
 */
GMP_STATIC_INLINE void ctl_init_pos_controller(ctl_pos_controller_t* pc)
{
    pc->kp = 0.0f;
    pc->speed_limit = 0.0f;
    pc->target_revs = 0;
    pc->target_angle = 0.0f;
    pc->actual_revs = 0;
    pc->actual_angle = 0.0f;
    pc->speed_ref = 0.0f;
    ctl_set_divider(&pc->div, 1);
}

/**
 * @brief Sets up the parameters for the position controller.
 * @param[out] pc Pointer to the position controller structure.
 * @param[in]  kp Proportional gain.
 * @param[in]  speed_limit Maximum output speed reference.
 * @param[in]  division The frequency division factor for the controller execution.
 */
GMP_STATIC_INLINE void ctl_setup_pos_controller(ctl_pos_controller_t* pc, ctrl_gt kp, ctrl_gt speed_limit,
                                                uint32_t division)
{
    pc->kp = kp;
    pc->speed_limit = fabsf(speed_limit);
    ctl_set_divider(&pc->div, division);
}

/**
 * @brief Sets the target position for the controller.
 * @param[out] pc Pointer to the position controller structure.
 * @param[in]  target_revs The target number of full revolutions.
 * @param[in]  target_angle The fractional target angle (0.0 to 1.0).
 */
GMP_STATIC_INLINE void ctl_set_pos_target(ctl_pos_controller_t* pc, int32_t target_revs, ctrl_gt target_angle)
{
    pc->target_revs = target_revs;
    pc->target_angle = target_angle;
}

/**
 * @brief Provides the controller with the current motor position feedback.
 * @param[out] pc Pointer to the position controller structure.
 * @param[in]  actual_revs The current number of full revolutions.
 * @param[in]  actual_angle The current angle within the revolution (0.0 to 1.0).
 */
GMP_STATIC_INLINE void ctl_input_pos_feedback(ctl_pos_controller_t* pc, int32_t actual_revs, ctrl_gt actual_angle)
{
    pc->actual_revs = actual_revs;
    pc->actual_angle = actual_angle;
}

/**
 * @brief Provides position feedback using only an angle, tracking revolutions internally.
 * @param[out] pc Pointer to the position controller structure.
 * @param[in]  actual_angle The current angle within the revolution (0.0 to 1.0).
 */
GMP_STATIC_INLINE void ctl_input_pos_feedback_angle_only(ctl_pos_controller_t* pc, ctrl_gt actual_angle)
{
    ctrl_gt delta_ang = actual_angle - pc->actual_angle;
    pc->actual_angle = actual_angle;
    if (delta_ang < -0.5f)
    {
        pc->actual_revs++;
    }
    else if (delta_ang > 0.5f)
    {
        pc->actual_revs--;
    }
}

/**
 * @brief Executes one step of the position control loop.
 * @param[out] pc Pointer to the position controller structure.
 */
GMP_STATIC_INLINE void ctl_step_pos_controller(ctl_pos_controller_t* pc)
{
    if (ctl_step_divider(&pc->div))
    {
        // 1. Calculate the error in full revolutions.
        int32_t rev_error = pc->target_revs - pc->actual_revs;

        // 2. Calculate the error in the fractional angle.
        ctrl_gt angle_error = pc->target_angle - pc->actual_angle;

        // Correct for angle wrap-around to ensure the shortest path is taken.
        if (angle_error > 0.5f)
        {
            angle_error -= 1.0f;
        }
        else if (angle_error < -0.5f)
        {
            angle_error += 1.0f;
        }

        // 3. Combine the errors into a single total position error.
        // The revolution error is scaled by 1.0 (representing a full turn)
        ctrl_gt total_error = (ctrl_gt)rev_error + angle_error;

        // 4. Apply the proportional gain to get the speed reference.
        pc->speed_ref = ctl_mul(pc->kp, total_error);

        // 5. Saturate the output to the defined speed limit.
        pc->speed_ref = ctl_sat(pc->speed_ref, pc->speed_limit, -pc->speed_limit);
    }
}

/**
 * @brief Gets the calculated speed reference output.
 * @param[in] pc Pointer to the position controller structure.
 * @return The current speed reference to be sent to the velocity controller.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_pos_speed_ref(const ctl_pos_controller_t* pc)
{
    return pc->speed_ref;
}

/** @} */ // end of POSITION_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_BASIC_POS_LOOP_P_H_
