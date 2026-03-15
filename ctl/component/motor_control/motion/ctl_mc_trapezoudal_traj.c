



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
