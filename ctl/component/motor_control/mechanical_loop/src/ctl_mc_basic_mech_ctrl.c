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

#include <ctl/component/motor_control/mechanical_loop/basic_mech_ctrl.h>

/**
 * @brief Auto-tunes the Mechanical Loop PI parameters with rigorous Per-Unit (PU) scaling.
 * * @details
 * To ensure ZERO physical unit conversions in the real-time `step` function, 
 * all physical gains must be pre-multiplied by PU scaling factors during initialization.
 * * **1. Velocity Loop (PI -> PU):**
 * Controller equation: @f$ I_{pu} = K_{p\_pu} \cdot \omega_{pu} + \dots @f$
 * Substitute physical variables: @f$ \frac{I_q}{I_{base}} = K_{p\_pu} \cdot \frac{\omega_{rad}}{\omega_{base}} @f$
 * Thus, the relationship to physical gain @f$ K_{p\_phy} = \frac{J \cdot \omega_{cv}}{K_t} @f$ is:
 * @f[ K_{p\_pu} = K_{p\_phy} \cdot \frac{\omega_{base}}{I_{base}} = \frac{J \cdot \omega_{cv} \cdot \omega_{base}}{K_t \cdot I_{base}} @f]
 * * **2. Position Loop (P-only -> PU):**
 * Controller equation: @f$ \omega_{pu} = K_{pp\_pu} \cdot \theta_{pu} @f$
 * Substitute physical variables: @f$ \frac{\omega_{rad}}{\omega_{base}} = K_{pp\_pu} \cdot \frac{\theta_{rad}}{2\pi} @f$
 * Thus, the relationship to physical bandwidth @f$ K_{pp\_phy} = \omega_{cp} @f$ is:
 * @f[ K_{pp\_pu} = \omega_{cp} \cdot \frac{2\pi}{\omega_{base}} @f]
 */
void ctl_autotuning_mech_ctrl(ctl_mech_ctrl_init_t* init)
{
    // Protect against division by zero
    parameter_gt kt = (init->torque_const > 1e-6f) ? init->torque_const : 1.0f;
    parameter_gt i_base = (init->i_base > 1e-6f) ? init->i_base : 1.0f;
    parameter_gt w_base = (init->omega_base > 1e-6f) ? init->omega_base : 1.0f;

    // Calculate PU scaling factors
    parameter_gt scale_vel_to_pu = w_base / i_base;
    parameter_gt scale_pos_to_pu = CTL_PARAM_CONST_2PI / w_base;

    // --- Velocity Loop Tuning ---
    parameter_gt w_cv = CTL_PARAM_CONST_2PI * init->target_vel_bw;
    parameter_gt kp_phy = (init->inertia * w_cv) / kt;

    // Convert to PU gain
    init->vel_kp = kp_phy * scale_vel_to_pu;

    parameter_gt ki_phy;
    if (init->damping > 1e-6f)
    {
        ki_phy = (init->damping * w_cv) / kt;
    }
    else
    {
        ki_phy = kp_phy * (w_cv / 10.0f);
    }
    // Convert to PU gain
    init->vel_ki = ki_phy * scale_vel_to_pu;

    // --- Position Loop Tuning (P-only) ---
    parameter_gt w_cp = CTL_PARAM_CONST_2PI * init->target_pos_bw;

    // Convert to PU gain
    init->pos_kp = w_cp * scale_pos_to_pu;
    init->pos_ki = 0;
}

void ctl_init_mech_ctrl(ctl_mech_ctrl_t* ctrl, const ctl_mech_ctrl_init_t* init)
{
    parameter_gt fs_mech = init->fs / (parameter_gt)init->mech_division;

    // 1. Initialize Velocity Controller (PI) with PU gains
    ctl_init_pid(&ctrl->vel_ctrl, init->vel_kp, init->vel_ki, 0.0f, fs_mech);
    ctl_set_pid_limit(&ctrl->vel_ctrl, init->cur_limit, -init->cur_limit);
    ctl_set_pid_int_limit(&ctrl->vel_ctrl, init->cur_limit, -init->cur_limit);

    // 2. Initialize Position Controller (P-only) with PU gains
    ctl_init_pid(&ctrl->pos_ctrl, init->pos_kp, init->pos_ki, 0.0f, fs_mech);
    ctl_set_pid_limit(&ctrl->pos_ctrl, init->speed_limit, -init->speed_limit);

    // 3. Initialize Velocity Trajectory (Slope Limiter)
    // 【核心修正】：将用户提供的物理加速度 (PU/s) 换算为单步增量 (PU/tick)
    parameter_gt slope_per_tick = init->speed_slope_limit / fs_mech;

    // 假设底层 slope_limiter 在 fs=1.0 传入时直接使用我们算好的单步增量
    // 或者直接调用 ctl_set_slope_limiter_slopes 强行覆盖保证绝对准确
    ctl_init_slope_limiter(&ctrl->vel_traj, slope_per_tick, -slope_per_tick, 1.0f);
    ctl_set_slope_limiter_slopes(&ctrl->vel_traj, float2ctrl(slope_per_tick), float2ctrl(-slope_per_tick));

    // 4. Initialize Divider
    ctl_init_divider(&ctrl->div_mech, init->mech_division);

    // 5. Apply Settings & Clear States
    ctrl->speed_limit = float2ctrl(init->speed_limit);
    ctrl->active_mode = MECH_MODE_DISABLE;
    ctrl->pos_if = NULL;
    ctrl->spd_if = NULL;

    ctl_clear_mech_ctrl(ctrl);
}
