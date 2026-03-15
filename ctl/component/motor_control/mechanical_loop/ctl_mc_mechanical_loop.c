/**
 * @file motion_init.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2024-09-30
 * 
 * @copyright Copyright GMP(c) 2024
 * 
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// velocity and position loop

#include <ctl/component/motor_control/mechanical_loop/basic_mechanical_loop.h>

/**
 * @brief Initializes the velocity and position controller structure to safe defaults.
 * @param[out] ctrl Pointer to the velocity and position controller structure.
 * @param[in]  vel_kp Proportional gain.
 * @param[in]  pos_kp Proportional gain.
 * @param[in]  vel_ki Integral gain.
 * @param[in]  pos_ki Integral gain.
 * @param[in]  speed_limit Maximum output speed reference.
 * @param[in]  speed_slope_limit Maximum slope limit in pu/s.
 * @param[in]  cur_limit Maximum output current limit.
 * @param[in]  vel_division The frequency division factor for the velocity controller execution.
 * @param[in]  pos_division The frequency division factor for the position controller execution.
 * @param[in]  fs Controller execution frequency (Hz).
 */
void ctl_init_vel_pos_ctrl(ctl_vel_pos_controller_t* ctrl, parameter_gt vel_kp, parameter_gt pos_kp,
                           parameter_gt vel_ki, parameter_gt pos_ki, parameter_gt speed_limit,
                           parameter_gt speed_slope_limit, parameter_gt cur_limit, uint32_t vel_division,
                           uint32_t pos_division, parameter_gt fs)
{
    // Initialize velocity controller
    ctl_init_pid(&ctrl->vel_ctrl, vel_kp, vel_ki, 0, fs);

    // Initialize position controller
    ctl_init_pid(&ctrl->pos_ctrl, pos_kp, pos_ki, 0, fs);

    // Initialize velocity slope controller
    ctl_init_slope_limiter(&ctrl->vel_traj, speed_slope_limit, -speed_slope_limit, fs / vel_division);

    // Initialize dividers
    ctl_init_divider(&ctrl->div_velocity, vel_division);
    ctl_init_divider(&ctrl->div_position, pos_division);

    // Set other parameters
    ctrl->speed_limit = speed_limit;
    ctl_set_pid_limit(&ctrl->pos_ctrl, speed_limit, -speed_limit);
    ctrl->cur_limit = cur_limit;
    ctl_set_pid_limit(&ctrl->vel_ctrl, cur_limit, -cur_limit);
    ctrl->cur_output = 0;
    ctrl->target_revs = 0;
    ctrl->target_angle = 0;
    ctrl->target_velocity = 0;
    ctrl->current_spd_set = 0;

    // Enable controllers by default
    ctrl->flag_enable_velocity_ctrl = 1;
    ctrl->flag_enable_position_ctrl = 1;

    ctl_clear_vel_pos_ctrl(ctrl);
}
