

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
