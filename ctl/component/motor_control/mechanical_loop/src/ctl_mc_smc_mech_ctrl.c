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

#include <ctl/component/motor_control/mechanical_loop/smc_mech_ctrl.h>


/**
 * @brief Auto-tunes the SMC parameters and scales them to the PU system.
 * * @details
 * **1. Physical Sliding Surface & Equivalent Control:**
 * Sliding surface: @f$ s_{phy} = \lambda_{phy} \theta_{err\_rad} + \omega_{err\_rad} @f$
 * Equivalent gain: @f$ \eta_{2\_phy} = \frac{J \lambda_{phy}}{K_t} @f$
 * Switching gain: @f$ \rho_{phy} = \frac{T_{reject}}{K_t} @f$
 * * **2. Per-Unit Absorption:**
 * To execute `ctl_step_smc` entirely in PU, the sliding surface becomes:
 * @f$ s_{pu} = \lambda_{pu} \theta_{err\_pu} + \omega_{err\_pu} @f$
 * By equating @f$ s_{phy} = \Omega_{base} \cdot s_{pu} @f$, we find the PU slope:
 * @f[ \lambda_{pu} = \lambda_{phy} \cdot \frac{2\pi}{\Omega_{base}} @f]
 * * The output equation @f$ I_{pu} \cdot I_{base} = \eta_{2\_phy} \cdot (\omega_{err\_pu} \cdot \Omega_{base}) @f$ yields:
 * @f[ \eta_{2\_pu} = \eta_{2\_phy} \cdot \frac{\Omega_{base}}{I_{base}} @f]
 * @f[ \rho_{pu} = \frac{\rho_{phy}}{I_{base}} @f]
 * @f[ K_{ff\_pu} = K_{ff\_phy} \cdot \frac{\Omega_{base}}{I_{base}} @f]
 */
void ctl_autotuning_smc_mech_ctrl(ctl_smc_mech_init_t* init)
{
    // Protect against division by zero
    parameter_gt kt = (init->torque_const > 1e-6f) ? init->torque_const : 1.0f;
    parameter_gt i_base = (init->i_base > 1e-6f) ? init->i_base : 1.0f;
    parameter_gt w_base = (init->omega_base > 1e-6f) ? init->omega_base : 1.0f;

    // 1. Physical Target Dynamics
    parameter_gt lambda_phy = CTL_PARAM_CONST_2PI * init->target_bw;
    parameter_gt eta2_phy = (init->inertia * lambda_phy) / kt;
    parameter_gt rho_phy = init->dist_reject_torque / kt;
    parameter_gt k_ff_phy = init->inertia / kt;

    // 2. PU Scaling Factors
    parameter_gt scale_vel_to_pu = w_base / i_base;

    // 3. PU Parameter Assignment
    // Scale lambda to match the PU definition of x1(1 rev) and x2(1 w_base)
    init->lambda = lambda_phy * (CTL_PARAM_CONST_2PI / w_base);

    // No position-proportional (x1) term in pure inertia equivalent control
    init->eta11 = 0.0f;
    init->eta12 = 0.0f;

    // Velocity-proportional (x2) equivalent gain
    parameter_gt eta2_pu = eta2_phy * scale_vel_to_pu;
    init->eta21 = eta2_pu;
    init->eta22 = eta2_pu;

    // Switching Gain
    init->rho = rho_phy / i_base;

    // Feedforward Gain
    init->k_ff = k_ff_phy * scale_vel_to_pu;
}

void ctl_init_smc_mech_ctrl(ctl_smc_mech_ctrl_t* ctrl, const ctl_smc_mech_init_t* init)
{
    // Initialize the internal Sliding Mode Controller core directly with PU gains
    ctl_init_smc(&ctrl->smc_core, float2ctrl(init->eta11), float2ctrl(init->eta12), float2ctrl(init->eta21),
                 float2ctrl(init->eta22), float2ctrl(init->rho), float2ctrl(init->lambda), float2ctrl(0.001f));

    ctrl->cur_limit = float2ctrl(init->cur_limit);
    ctrl->k_ff = float2ctrl(init->k_ff);

    ctl_init_divider(&ctrl->div_mech, init->mech_division);

    ctrl->pos_if = NULL;
    ctrl->spd_if = NULL;
    ctrl->flag_enable = 0;
    ctrl->cur_output = float2ctrl(0.0f);
}
