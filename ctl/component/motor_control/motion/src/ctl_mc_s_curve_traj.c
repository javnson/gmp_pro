
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// s curve trajectory

#include <ctl/component/motor_control/motion/s_curve_traj.h>

/**
 * @brief Initializes the S-Curve planner and perfectly absorbs all dimension constants.
 */
void ctl_init_scurve_planner(ctl_scurve_planner_t* planner, const ctl_scurve_planner_init_t* init)
{
    // Safe Guards
    parameter_gt fs_safe = (init->fs_motion > 1e-6f) ? init->fs_motion : 1000.0f;
    parameter_gt omega_base = (init->omega_base > 1e-6f) ? init->omega_base : 1.0f;
    parameter_gt dt_phy = 1.0f / fs_safe;

    // 1. Direct PU assignments
    planner->dt = float2ctrl(dt_phy);
    planner->max_vel_limit = float2ctrl(init->max_vel_pu);
    planner->max_accel_limit = float2ctrl(init->max_accel_pu);

    // Max Accel change per tick = J_max * dt
    planner->max_jerk_step = float2ctrl(init->max_jerk_pu * dt_phy);

    // 2. Kinematic Conversion (Scale PU Speed to Delta Revs per Tick)
    // 1.0 PU Velocity = omega_base rad/s = (omega_base / 2*PI) revs/s
    parameter_gt scale_v_to_rev_phy = (omega_base / CTL_PARAM_CONST_2PI) * dt_phy;
    planner->scale_v_to_rev = float2ctrl(scale_v_to_rev_phy);

    // 3. Perfect S-Curve Braking Constants Absorption
    // S_brake = V^2 / (2*A_max) + V * A_max / (2*J_max)  [Output in Revs]
    parameter_gt k1 = (1.0f / (2.0f * init->max_accel_pu)) * scale_v_to_rev_phy;
    parameter_gt k2 = (init->max_accel_pu / (2.0f * init->max_jerk_pu)) * scale_v_to_rev_phy;

    // V_flare = A^2 / (2*J_max) [Output in PU Velocity]
    parameter_gt k3 = 1.0f / (2.0f * init->max_jerk_pu);

    planner->coef_k1_vsq = float2ctrl(k1);
    planner->coef_k2_v = float2ctrl(k2);
    planner->coef_k3_flare = float2ctrl(k3);

    // 4. Dynamic Tolerances (Based on the computational step size)
    // Tolerate the distance of one tick of deceleration
    planner->arrival_tol_vel = float2ctrl(init->max_accel_pu * dt_phy * 1.5f);
    planner->arrival_tol_revs = float2ctrl(init->max_vel_pu * scale_v_to_rev_phy * 2.0f);

    // 5. Protection Settings
    planner->tracking_err_limit = float2ctrl(init->tracking_err_limit);
    planner->divergence_limit = (uint32_t)(init->fault_time_ms * fs_safe / 1000.0f);
    if (planner->divergence_limit < 1)
        planner->divergence_limit = 1;

    // 6. Default States
    planner->planner_revs = 0;
    planner->planner_angle = float2ctrl(0.0f);
    planner->planner_vel_pu = float2ctrl(0.0f);
    planner->planner_acc_pu = float2ctrl(0.0f);
    planner->target_revs = 0;
    planner->target_angle = float2ctrl(0.0f);

    planner->pos_if = NULL;
    planner->div_shared = NULL;

    planner->divergence_cnt = 0;
    planner->flag_enable = 0;
    planner->flag_fault_divergence = 0;
    planner->flag_target_reached = 1;
}
