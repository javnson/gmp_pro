

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// trapezoidal trajectory

#include <ctl/component/motor_control/motion/trapezoidal_traj.h>

/**
 * @brief Initializes the trapezoidal planner and aligns all variables to Speed PU base.
 */
void ctl_init_trap_planner(ctl_trap_planner_t* planner, const ctl_trap_planner_init_t* init)
{
    // Safe Guards
    parameter_gt fs_safe = (init->fs_motion > 1e-6f) ? init->fs_motion : 1000.0f;
    parameter_gt omega_base = (init->omega_base > 1e-6f) ? init->omega_base : 1.0f;

    // 1. Assign limits directly from user input (already in PU and PU/s)
    parameter_gt max_vel_limit_phy = init->max_vel_pu;
    parameter_gt max_accel_step_phy = init->max_accel_pu / fs_safe; // accel increment per tick

    // 2. Calculate Kinematic Conversion Factors
    // Speed is PU based on Omega_base.
    // So 1.0 PU Speed = Omega_base rad/s = (Omega_base / 2*PI) Revs/s.
    // In one tick (1/fs), delta_Revs = V_pu * [ Omega_base / (2*PI*fs) ]
    parameter_gt scale_v_to_rev_phy = omega_base / (CTL_PARAM_CONST_2PI * fs_safe);

    // The "C" Constant for braking: C * S_brake_revs = V_pu^2
    // C = 2 * A_step_pu / scale_v_to_rev
    // (Notice how clean this remains after the PU variable substitution)
    parameter_gt c_brake_phy = (2.0f * max_accel_step_phy) / scale_v_to_rev_phy;

    // Arrival tolerance: Distance covered by one step of deceleration
    parameter_gt tol_revs_phy = max_accel_step_phy * scale_v_to_rev_phy * 1.5f;

    // 3. Assign to structure
    planner->max_vel_limit = float2ctrl(max_vel_limit_phy);
    planner->max_accel_step = float2ctrl(max_accel_step_phy);
    planner->scale_v_to_rev = float2ctrl(scale_v_to_rev_phy);
    planner->coef_brake_s_to_vsq = float2ctrl(c_brake_phy);
    planner->arrival_tol_revs = float2ctrl(tol_revs_phy);

    planner->tracking_err_limit = float2ctrl(init->tracking_err_limit);

    // Calculate debounce limit based on physical time
    planner->divergence_limit = (uint32_t)(init->fault_time_ms * fs_safe / 1000.0f);
    if (planner->divergence_limit < 1)
        planner->divergence_limit = 1;

    // Default States
    planner->planner_revs = 0;
    planner->planner_angle = float2ctrl(0.0f);
    planner->planner_vel_pu = float2ctrl(0.0f);
    planner->target_revs = 0;
    planner->target_angle = float2ctrl(0.0f);

    planner->pos_if = NULL;
    planner->div_shared = NULL;

    planner->divergence_cnt = 0;
    planner->flag_enable = 0;
    planner->flag_fault_divergence = 0;
    planner->flag_target_reached = 1;
}
