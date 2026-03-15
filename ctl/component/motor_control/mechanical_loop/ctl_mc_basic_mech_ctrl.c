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
 * @brief Auto-tunes the Mechanical Loop PI parameters based on physical models.
 * * @details
 * **1. Velocity Loop (PI):**
 * Based on the motor mechanical equation: @f$ T_e = J \frac{d\omega}{dt} + B\omega @f$
 * To achieve a closed-loop bandwidth of @f$ \omega_{cv} = 2\pi \cdot f_{vel\_bw} @f$:
 * @f[ K_{p\_vel} = \frac{J \cdot \omega_{cv}}{K_t} @f]
 * @f[ K_{i\_vel} = \frac{B \cdot \omega_{cv}}{K_t} @f]
 * (If B is negligible or unknown, we place the PI zero at 1/10th of the bandwidth: 
 * @f$ K_{i\_vel} = K_{p\_vel} \cdot \frac{\omega_{cv}}{10} @f$)
 * * **2. Position Loop (P only):**
 * To achieve a position bandwidth of @f$ \omega_{cp} = 2\pi \cdot f_{pos\_bw} @f$:
 * @f[ K_{p\_pos} = \omega_{cp} @f]
 * Note: Position loop uses P-only to form a standard P-PI cascade, avoiding overshoot.
 */
void ctl_autotuning_mech_ctrl(ctl_mech_ctrl_init_t* init)
{
    // Ensure valid torque constant to avoid division by zero
    parameter_gt kt = (init->torque_const > 1e-6f) ? init->torque_const : 1.0f;

    // --- Velocity Loop Tuning ---
    parameter_gt w_cv = CTL_PARAM_CONST_2PI * init->target_vel_bw;

    init->vel_kp = (init->inertia * w_cv) / kt;

    // Use actual damping if provided, otherwise estimate a stable integral gain
    if (init->damping > 1e-6f)
    {
        init->vel_ki = (init->damping * w_cv) / kt;
    }
    else
    {
        init->vel_ki = init->vel_kp * (w_cv / 10.0f);
    }

    // --- Position Loop Tuning (P-only) ---
    parameter_gt w_cp = CTL_PARAM_CONST_2PI * init->target_pos_bw;
    init->pos_kp = w_cp;

    // Position integral is strictly zero in standard P-PI cascade
    // init->pos_ki = 0.0f; // Not needed as we use the generic PID structure
}

void ctl_init_mech_ctrl(ctl_mech_ctrl_t* ctrl, const ctl_mech_ctrl_init_t* init)
{
    parameter_gt fs_mech = init->fs_current / (parameter_gt)init->mech_division;

    // 1. Initialize Velocity Controller (PI)
    ctl_init_pid(&ctrl->vel_ctrl, init->vel_kp, init->vel_ki, 0.0f, fs_mech);
    ctl_set_pid_limit(&ctrl->vel_ctrl, init->cur_limit, -init->cur_limit);
    ctl_set_pid_int_limit(&ctrl->vel_ctrl, init->cur_limit, -init->cur_limit); // Prevent integral windup

    // 2. Initialize Position Controller (P-only)
    // We pass 0.0f for Ki and Kd to ensure it acts as a pure P controller
    ctl_init_pid(&ctrl->pos_ctrl, init->pos_kp, 0.0f, 0.0f, fs_mech);
    ctl_set_pid_limit(&ctrl->pos_ctrl, init->speed_limit, -init->speed_limit);

    // 3. Initialize Velocity Trajectory (Slope Limiter)
    // The slope is applied per mechanical step.
    ctl_init_slope_limiter(&ctrl->vel_traj, init->speed_slope_limit, -init->speed_slope_limit, fs_mech);

    // 4. Initialize Divider
    ctl_init_divider(&ctrl->div_mech, init->mech_division);

    // 5. Apply Settings & Clear States
    ctrl->speed_limit = float2ctrl(init->speed_limit);
    ctrl->active_mode = MECH_MODE_DISABLE;
    ctrl->pos_if = NULL;
    ctrl->spd_if = NULL;

    ctl_clear_mech_ctrl(ctrl);
}
