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

#include <ctl/component/motor_control/mechanical_loop/ladrc_spd_ctrl.h>

/**
 * @brief Auto-tunes and Initializes the LADRC Speed Controller.
 * * @details
 * **Per-Unit Gain Calculation ($b_{0\_pu}$):**
 * The physical plant is modeled as: $J \dot{\omega} = K_t I_q + T_L$
 * In standard 1st-order form: $\dot{\omega} = \frac{K_t}{J} I_q + f$  => $b_{0\_phy} = \frac{K_t}{J}$
 * * To operate strictly in PU domain:
 * $\dot{\omega}_{pu} \cdot \Omega_{base} = b_{0\_phy} \cdot (I_{pu} \cdot I_{base}) + f$
 * $\dot{\omega}_{pu} = \left( b_{0\_phy} \cdot \frac{I_{base}}{\Omega_{base}} \right) I_{pu} + f_{pu}$
 * * Thus, the PU system gain is:
 * $$ b_{0\_pu} = \frac{K_t}{J} \cdot \frac{I_{base}}{\Omega_{base}} $$
 */
void ctl_autotuning_ladrc_spd_ctrl(ctl_ladrc_spd_init_t* init, ctl_ladrc_spd_ctrl_t* ctrl)
{
    // 1. Safe Guards
    parameter_gt kt = (init->torque_const > 1e-6f) ? init->torque_const : 1.0f;
    parameter_gt i_base = (init->i_base > 1e-6f) ? init->i_base : 1.0f;
    parameter_gt w_base = (init->omega_base > 1e-6f) ? init->omega_base : 1.0f;
    parameter_gt fs_mech = init->fs / (parameter_gt)init->mech_division;

    // 2. Calculate Per-Unit System Gain (b0_pu)
    parameter_gt b0_phy = kt / init->inertia;
    parameter_gt b0_pu = b0_phy * (i_base / w_base);

    // 3. Initialize the core 1st-Order LADRC
    // Note: We use the float/parameter_gt init function here. The limits are set to +/- 1.0 internally by default.
    ctl_init_ladrc1(&ctrl->ladrc_core, b0_pu, init->target_wc, init->target_wo, fs_mech);

    // Override default LADRC limits with user's specific current limits
    ctl_set_ladrc1_limit(&ctrl->ladrc_core, float2ctrl(init->cur_limit), float2ctrl(-init->cur_limit));

    // 4. Initialize Velocity Trajectory (Convert PU/s to PU/tick)
    parameter_gt slope_per_tick = init->speed_slope_limit / fs_mech;
    ctl_init_slope_limiter(&ctrl->vel_traj, slope_per_tick, -slope_per_tick, 1.0f);
    ctl_set_slope_limiter_slopes(&ctrl->vel_traj, float2ctrl(slope_per_tick), float2ctrl(-slope_per_tick));

    // 5. Initialize Divider
    ctl_init_divider(&ctrl->div_mech, init->mech_division);

    // 6. Apply Settings & Clear States
    ctrl->speed_limit = float2ctrl(init->speed_limit);
    ctrl->cur_limit = float2ctrl(init->cur_limit);
    ctrl->active_mode = LADRC_SPD_MODE_DISABLE;
    ctrl->spd_if = NULL;

    ctl_clear_ladrc_spd_ctrl(ctrl);
}
