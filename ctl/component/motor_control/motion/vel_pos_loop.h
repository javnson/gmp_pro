/**
 * @file vel_pos_loop.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements a velocity and position controller.
 * @version 0.2
 * @date 2026-01-19
 *
 * @copyright Copyright GMP(c) 2024
 */

#ifndef _FILE_VEC_POS_LOOP_P_H_
#define _FILE_VEC_POS_LOOP_P_H_

#include <ctl/component/motor_control/basic/motor_universal_interface.h>

#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Basic Velocity and Position Controller                                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup VELOCITY_POSITION_CONTROLLER Basic Velocity and Position Controller
 * @brief A simple velocity and position loop.
 * @details This module provides a simple and effective position controller. It takes
 * a target position (composed of full revolutions and an angle) and a feedback
 * position, calculates the error, and multiplies it by a proportional gain (Kp)
 * to produce a speed reference. The output is saturated to a defined limit.
 * This controller forms the outermost loop in a cascaded position control system.
 * @{
 */

typedef struct tag_vel_pos_controller
{
    // --- Inputs (updated each cycle) ---
    rotation_ift* pos_if; //!< @brief Standard rotation input interface.
    velocity_ift* spd_if; //!< @brief Standard velocity input interface.

    // --- Controller ---
    ctl_pid_t vel_ctrl;  ///< Velocity PI controller.
    ctl_pid_t pos_ctrl;  ///< Position PI controller.

    // --- Target & Feedback ---
    int32_t target_revs;  ///< The integer part of the target position (full revolutions).
    ctrl_gt target_angle; ///< The fractional part of the target position (0.0 to 1.0).
    ctrl_gt target_velocity; ///< The target velocity.

    // --- Parameters ---
    ctrl_gt speed_limit; ///< Maximum output speed reference (saturation limit).
    ctrl_gt cur_limit;    ///< Maximum output current limit.

    // --- Output ---
    ctrl_gt cur_output; ///< The output current to current controller.

    // --- Execution Control ---
    ctl_divider_t div_velocity; ///< Divider to run the velocity loop at a lower frequency.
    ctl_divider_t div_position; ///< Divider to run the position loop at a lower frequency.

    // --- Flags ---
    fast_gt flag_enable_velocity_ctrl; ///< Flag to enable/disable velocity control.
    fast_gt flag_enable_position_ctrl; ///< Flag to enable/disable position control.
} ctl_vel_pos_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the velocity and position controller structure to safe defaults.
 * @param[out] ctrl Pointer to the velocity and position controller structure. @ref ctl_vel_pos_controller_t
 */
GMP_STATIC_INLINE void ctl_clear_vel_pos_ctrl(ctl_vel_pos_controller_t* ctrl)
{
    ctl_clear_pid(&ctrl->vel_ctrl);
    ctl_clear_pid(&ctrl->pos_ctrl);
    ctl_clear_divider(&ctrl->div_velocity);
    ctl_clear_divider(&ctrl->div_position);
    ctrl->cur_output = 0;
}

/**
 * @brief Initializes the velocity and position controller structure to safe defaults.
 * @param[out] ctrl Pointer to the velocity and position controller structure.
 * @param[in]  vel_kp Proportional gain.
 * @param[in]  pos_kp Proportional gain.
 * @param[in]  vel_ki Integral gain.
 * @param[in]  pos_ki Integral gain.
 * @param[in]  speed_limit Maximum output speed reference.
 * @param[in]  cur_limit Maximum output current limit.
 * @param[in]  vel_division The frequency division factor for the velocity controller execution.
 * @param[in]  pos_division The frequency division factor for the position controller execution.
 * @param[in]  fs Controller execution frequency (Hz).
 */
void ctl_init_vec_pos_controller(ctl_vel_pos_controller_t* ctrl, parameter_gt vel_kp, parameter_gt pos_kp,
    parameter_gt vel_ki, parameter_gt pos_ki, 
    parameter_gt speed_limit, parameter_gt cur_limit, uint32_t vel_division, uint32_t pos_division, parameter_gt fs);

/**
 * @brief Executes one step of the velocity and position control loop.
 * @param[out] ctrl      Pointer to the velocity and position controller structure.
 */

GMP_STATIC_INLINE void ctl_step_vel_pos_controller(ctl_vel_pos_controller_t* ctrl)
{
    // Position Control Loop
    if (ctrl->flag_enable_position_ctrl)
    {
        if (ctl_step_divider(&ctrl->div_position))
        {
            // Calculate position error
            int32_t rev_error = ctrl->target_revs - ctrl->pos_if->revolutions;
            ctrl_gt ang_error = ctrl->target_angle - ctrl->pos_if->position;

            // Total position error
            ctrl_gt pos_error = (ctrl_gt)rev_error + ang_error;

            // Update position controller
            ctrl->target_velocity = ctl_step_pid_par(&ctrl->pos_ctrl,pos_error);
        }
    }

    // Velocity Control Loop
    if (ctrl->flag_enable_velocity_ctrl)
    {
        if (ctl_step_divider(&ctrl->div_velocity))
        {
            // Calculate velocity error
            ctrl_gt spd_feedback = ctrl->spd_if->speed;
            ctrl_gt spd_error = ctrl->target_velocity - spd_feedback;

            // Update velocity controller
            ctrl->cur_output = ctl_step_pid_par(&ctrl->vel_ctrl, spd_error);
        }
    }
}

/** @} */ // end of VELOCITY_POSITION_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_VEC_POS_LOOP_P_H_
