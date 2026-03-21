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

#include <ctl/component/motor_control/mechanical_loop/mit_mech_ctrl.h>

/**
 * @brief Auto-tunes the FSF Gains using Pole Placement and absorbs PU scaling.
 * * @details
 * **1. Physical Model Pole Placement:**
 * $$ K_{pp\_phy} = \frac{J \omega_n^2}{K_t} $$
 * $$ K_{vp\_phy} = \frac{2 \zeta J \omega_n}{K_t} $$
 * $$ K_{ff\_phy} = \frac{J}{K_t} $$
 * * **2. Per-Unit Scaling Conversion:**
 * We must convert the physical gains to PU gains so the real-time controller 
 * operates entirely without unit conversion constants.
 * - Position Control Law: $I_{pu} \cdot I_{base} = K_{pp\_phy} \cdot (\theta_{pu} \cdot 2\pi)$
 * $\implies K_{pp} = K_{pp\_phy} \cdot \frac{2\pi}{I_{base}}$
 * - Velocity Control Law: $I_{pu} \cdot I_{base} = K_{vp\_phy} \cdot (\omega_{pu} \cdot \Omega_{base})$
 * $\implies K_{vp} = K_{vp\_phy} \cdot \frac{\Omega_{base}}{I_{base}}$
 * - Accel Feedforward: $I_{pu} \cdot I_{base} = K_{ff\_phy} \cdot (\alpha_{pu} \cdot \Omega_{base})$
 * $\implies K_{ff} = K_{ff\_phy} \cdot \frac{\Omega_{base}}{I_{base}}$
 */
void ctl_autotuning_mit_pos_ctrl(ctl_mit_pos_init_t* init)
{
    // Protect against division by zero
    parameter_gt kt = (init->torque_const > 1e-6f) ? init->torque_const : 1.0f;
    parameter_gt i_base = (init->i_base > 1e-6f) ? init->i_base : 1.0f;
    parameter_gt w_base = (init->omega_base > 1e-6f) ? init->omega_base : 1.0f;
    parameter_gt damping = (init->damping_ratio > 1e-6f) ? init->damping_ratio : 1.0f;

    // 1. Calculate physical target dynamics
    parameter_gt wn = CTL_PARAM_CONST_2PI * init->target_bw;
    parameter_gt wn_sq = wn * wn;

    // 2. Base Physical Gains
    parameter_gt k_pp_phy = (init->inertia * wn_sq) / kt;
    parameter_gt k_vp_phy = (2.0f * damping * init->inertia * wn) / kt;
    parameter_gt k_ff_phy = init->inertia / kt;

    // 3. Absorb Per-Unit Conversions into Real-time Gains
    parameter_gt scale_pos_to_pu = CTL_PARAM_CONST_2PI / i_base;
    parameter_gt scale_vel_to_pu = w_base / i_base;

    init->k_pp = k_pp_phy * scale_pos_to_pu;
    init->k_vp = k_vp_phy * scale_vel_to_pu;
    init->k_ff = k_ff_phy * scale_vel_to_pu; // Assuming acceleration ref is provided as PU/s
}

void ctl_init_mit_pos_ctrl(ctl_mit_pos_ctrl_t* ctrl, const ctl_mit_pos_init_t* init)
{
    // Convert initialization parameters to real-time control variables
    ctrl->k_pp = float2ctrl(init->k_pp);
    ctrl->k_vp = float2ctrl(init->k_vp);
    ctrl->k_ff = float2ctrl(init->k_ff);

    ctrl->cur_limit = float2ctrl(init->cur_limit);

    ctl_init_divider(&ctrl->div_mech, init->mech_division);

    ctrl->pos_if = NULL;
    ctrl->spd_if = NULL;
    ctrl->flag_enable = 0;
    ctrl->cur_output = float2ctrl(0.0f);
}
