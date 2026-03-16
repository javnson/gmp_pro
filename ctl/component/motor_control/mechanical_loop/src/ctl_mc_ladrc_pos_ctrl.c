
#include <gmp_core.h>

#include <ctl/component/motor_control/mechanical_loop/ladrc_pos_ctrl.h>

/**
 * @brief Auto-tunes and Initializes the LADRC Position Controller.
 * * @details
 * **Per-Unit Gain Calculation ($b_{0\_pu}$):**
 * The physical plant is modeled as: $J \ddot{\theta}_{rad} = K_t I_q + T_L$
 * We configure the internal LADRC to operate strictly in `Revolutions`.
 * Since $\theta_{rad} = \theta_{rev} \cdot 2\pi$, the plant becomes:
 * $\ddot{\theta}_{rev} \cdot 2\pi = \frac{K_t}{J} (I_{pu} \cdot I_{base}) + f$
 * $\ddot{\theta}_{rev} = \left( \frac{K_t \cdot I_{base}}{J \cdot 2\pi} \right) I_{pu} + f_{rev}$
 * * Thus, the mathematically pure PU system gain is:
 * $$ b_{0\_pu} = \frac{K_t \cdot I_{base}}{J \cdot 2\pi} $$
 */
void ctl_autotuning_ladrc_pos_ctrl(ctl_ladrc_pos_init_t* init, ctl_ladrc_pos_ctrl_t* ctrl)
{
    // 1. Safe Guards
    parameter_gt kt = (init->torque_const > 1e-6f) ? init->torque_const : 1.0f;
    parameter_gt i_base = (init->i_base > 1e-6f) ? init->i_base : 1.0f;
    parameter_gt w_base = (init->omega_base > 1e-6f) ? init->omega_base : 1.0f;
    parameter_gt fs_mech = init->fs / (parameter_gt)init->mech_division;

    // 2. Calculate Per-Unit System Gain (b0_pu in revs domain)
    parameter_gt b0_phy = kt / init->inertia;
    parameter_gt b0_pu = b0_phy * i_base / CTL_PARAM_CONST_2PI;

    // 3. Initialize the core 2nd-Order LADRC
    ctl_init_ladrc2(&ctrl->ladrc_core, b0_pu, init->target_wc, init->target_wo, fs_mech);

    // Override default LADRC limits with user's specific current limits
    ctl_set_ladrc2_limit(&ctrl->ladrc_core, float2ctrl(init->cur_limit), float2ctrl(-init->cur_limit));

    // 4. Calculate Conversion Scale (w_base to revs/s)
    // 1 PU speed = w_base (rad/s) = w_base / 2pi (revs/s)
    ctrl->scale_w_to_revs = float2ctrl(w_base / CTL_PARAM_CONST_2PI);

    // 5. Initialize Divider and State
    ctl_init_divider(&ctrl->div_mech, init->mech_division);
    ctrl->cur_limit = float2ctrl(init->cur_limit);

    ctl_clear_ladrc_pos_ctrl(ctrl);
    ctrl->flag_enable = 0;
}
