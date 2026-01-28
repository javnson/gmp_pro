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

////////////////////////////////////////////////////////////////////////////
//// Basic Pos loop
//
#include <ctl/component/motor_control/motion/basic_pos_loop_p.h>

void ctl_init_pos_controller(ctl_pos_controller_t* pc, parameter_gt kp, parameter_gt speed_limit, uint32_t division)
{
    pc->target_revs = 0;
    pc->target_angle = 0;
    pc->actual_revs = 0;
    pc->actual_angle = 0;
    pc->speed_ref = 0;
    pc->kp = float2ctrl(kp);
    pc->speed_limit = float2ctrl(fabsf(speed_limit));

    ctl_init_divider(&pc->div, division);
}

//////////////////////////////////////////////////////////////////////////
// Knob Wrapper

#include <ctl/component/motor_control/motion/knob_pos_loop.h>

//////////////////////////////////////////////////////////////////////////
// LADRC Speed controller
#include <ctl/component/motor_control/motion/ladrc_spd_ctrl.h>

void ctl_init_ladrc_speed_pu(ctl_ladrc_speed_pu_t* ladrc, parameter_gt wc_rads, parameter_gt wo_rads,
                             parameter_gt Kt_pu, parameter_gt H_s, parameter_gt sample_time_s)
{
    ladrc->wc = wc_rads;
    ladrc->wo = wo_rads;
    ladrc->h = sample_time_s;

    // Calculate the system gain b0
    if (H_s > 1e-9f)
    {
        ladrc->b0 = Kt_pu / H_s;
    }
    else
    {
        ladrc->b0 = 0.0f; // Avoid division by zero
    }

    // Reset states
    ctl_clear_ladrc_speed_pu(ladrc);
}

//////////////////////////////////////////////////////////////////////////
// NLADRC

#include <ctl/component/motor_control/motion/nladrc_spd_ctrl.h>

void ctl_init_nladrc(ctl_nladrc_controller_t* adrc, parameter_gt fs, parameter_gt r, parameter_gt omega_o,
                     parameter_gt omega_c, parameter_gt b0, parameter_gt alpha1, parameter_gt alpha2)
{
    ctrl_gt h = 1.0f / fs;

    // --- Key Point Analysis 1: Initialize Tracking Differentiator (TD) ---
    // The role of the TD is to arrange a smooth transition for a given input v(t) while providing its differential signal.
    // v1(k) -> tracks v(t)
    // v2(k) -> tracks the derivative of v(t)
    // 'r' is the speed factor that determines the tracking speed. A larger 'r' means faster tracking but may introduce overshoot.
    adrc->td.x1 = 0.0f;
    adrc->td.x2 = 0.0f;
    adrc->td.h = h;
    adrc->td.r = r;

    // --- Key Point Analysis 2: Initialize Extended State Observer (ESO) ---
    // The ESO is the core of ADRC. It treats the system as an integrator chain model.
    // z1 -> tracks the system output y (e.g., motor speed)
    // z2 -> tracks the derivative of y (e.g., motor acceleration)
    // z3 -> tracks the "total disturbance" f, which includes all internal and external uncertainties like load, friction, parameter variations, etc.
    // The observer bandwidth 'omega_o' determines the ESO's response speed and noise immunity. Higher bandwidth means faster tracking but more sensitivity to noise.
    // The observer gains (beta) are typically configured based on the bandwidth omega_o.
    adrc->eso.z1 = 0.0f;
    adrc->eso.z2 = 0.0f;
    adrc->eso.z3 = 0.0f;
    adrc->eso.h = h;
    adrc->eso.b0 = b0; // b0 is the estimate of the plant's control gain, a critical tuning parameter.
    adrc->eso.beta01 = 3.0f * omega_o;
    adrc->eso.beta02 = 3.0f * omega_o * omega_o;
    adrc->eso.beta03 = omega_o * omega_o * omega_o;

    // --- Key Point Analysis 3: Initialize Nonlinear State Error Feedback (NLSEF) ---
    // NLSEF generates the control signal based on the error between the TD's output (target trajectory) and the ESO's estimates (actual states).
    // The controller bandwidth 'omega_c' determines the closed-loop response speed.
    // Kp and Kd are typically configured based on the bandwidth omega_c.
    // alpha1, alpha2, and delta are parameters for the nonlinear fal() function, giving the controller the characteristic of "high gain for small errors, low gain for large errors".
    adrc->nlsef.kp = omega_c * omega_c;
    adrc->nlsef.kd = 2.0f * omega_c;
    adrc->nlsef.alpha1 = alpha1;
    adrc->nlsef.alpha2 = alpha2;
    adrc->nlsef.delta = h * omega_c; // delta can also be configured automatically based on the controller bandwidth.

    adrc->output = 0.0f;
    adrc->out_max = 1.0f; // Default limits
    adrc->out_min = -1.0f;
}

//////////////////////////////////////////////////////////////////////////
// sinusoidal trajectory

#include <ctl/component/motor_control/motion/sinusoidal_trajectory.h>

void ctl_init_sin_planner(ctl_sin_planner_t* planner, ctrl_gt initial_pos)
{
    planner->current_pos = initial_pos;
    planner->current_time = 0.0f;
    planner->is_active = 0;
    planner->start_pos = initial_pos;
    planner->delta_pos = 0.0f;
    planner->total_time = 0.0f;
}

//////////////////////////////////////////////////////////////////////////
// s curve trajectory

#include <ctl/component/motor_control/motion/s_curve_trajectory.h>

void ctl_init_s_curve(ctl_s_curve_planner_t* planner, ctrl_gt max_accel, ctrl_gt max_jerk, ctrl_gt initial_vel)
{
    planner->max_accel = max_accel;
    planner->max_jerk = max_jerk;
    planner->state = SCURVE_STATE_IDLE;
    planner->current_vel = initial_vel;
    planner->current_accel = 0.0f;
    planner->target_vel = initial_vel;
    planner->last_target_vel = initial_vel;
    planner->accel_target = 0.0f;
    planner->accel_dir = 1.0f;
    planner->const_accel_steps = 0;
    planner->step_counter = 0;
}

//////////////////////////////////////////////////////////////////////////
// trapezoidal trajectory

#include <ctl/component/motor_control/motion/trapezoidal_trajectory.h>

void ctl_init_trap_planner(ctl_trap_planner_t* planner, ctrl_gt max_vel, ctrl_gt max_accel, ctrl_gt initial_pos)
{
    planner->max_vel = fabsf(max_vel);
    planner->max_accel = fabsf(max_accel);
    planner->pos_deadband = 1e-4f; // Default deadband
    planner->current_pos = initial_pos;
    planner->current_vel = 0.0f;
    planner->target_pos = initial_pos;
}

//////////////////////////////////////////////////////////////////////////
// velocity and position loop

#include <ctl/component/motor_control/motion/vel_pos_loop.h>

void ctl_init_vel_pos_ctrl(ctl_vel_pos_controller_t* ctrl, parameter_gt vel_kp, parameter_gt pos_kp,
                           parameter_gt vel_ki, parameter_gt pos_ki, parameter_gt speed_limit, parameter_gt cur_limit,
                           uint32_t vel_division, uint32_t pos_division, parameter_gt fs)
{
    // Initialize velocity controller
    ctl_init_pid(&ctrl->vel_ctrl, vel_kp, vel_ki, 0, fs);

    // Initialize position controller
    ctl_init_pid(&ctrl->pos_ctrl, pos_kp, pos_ki, 0, fs);

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

    // Enable controllers by default
    ctrl->flag_enable_velocity_ctrl = 1;
    ctrl->flag_enable_position_ctrl = 1;

    ctl_clear_vel_pos_ctrl(ctrl);
}
