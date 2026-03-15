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

#include <ctl/component/motor_control/mechanical_loop/mit_mech_ctrl.h>

/**
 * @brief Auto-tunes the FSF Gains using Pole Placement.
 * * @details
 * The mechanical system is modeled as a double integrator (assuming ideal inner current loop):
 * $$ J \ddot{\theta} = K_t \cdot I_q $$
 * * The control law (ignoring feedforward for stability analysis) is:
 * $$ I_q = K_{pp}(\theta_{ref} - \theta) + K_{vp}(\dot{\theta}_{ref} - \dot{\theta}) $$
 * * Substituting the control law into the plant yields the closed-loop characteristic equation:
 * $$ s^2 + \frac{K_t K_{vp}}{J} s + \frac{K_t K_{pp}}{J} = 0 $$
 * * Equating this to the standard second-order system equation:
 * $$ s^2 + 2\zeta\omega_n s + \omega_n^2 = 0 $$
 * * We can directly solve for the required physical gains:
 * $$ K_{pp} = \frac{J \omega_n^2}{K_t} $$
 * $$ K_{vp} = \frac{2 \zeta J \omega_n}{K_t} $$
 * * The Feedforward gain $K_{ff}$ is derived purely from the inverse plant model:
 * $$ K_{ff} = \frac{J}{K_t} $$
 */
void ctl_autotuning_mit_pos_ctrl(ctl_mit_pos_init_t* init)
{
    // Protect against division by zero
    parameter_gt kt = (init->torque_const > 1e-6f) ? init->torque_const : 1.0f;
    parameter_gt damping = (init->damping_ratio > 1e-6f) ? init->damping_ratio : 1.0f;

    // Convert target bandwidth (Hz) to natural frequency (rad/s)
    parameter_gt wn = CTL_PARAM_CONST_2PI * init->target_bw;
    parameter_gt wn_sq = wn * wn;

    // Pole Placement Calculus
    init->k_pp = (init->inertia * wn_sq) / kt;
    init->k_vp = (2.0f * damping * init->inertia * wn) / kt;

    // Inertia Feedforward
    init->k_ff = init->inertia / kt;
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
