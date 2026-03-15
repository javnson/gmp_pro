


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
