#include "smc_mech_ctrl.h"

/**
 * @brief Auto-tunes the SMC parameters based on the physical motor equation.
 * * @details
 * **1. Plant Model:**
 * @f[ J \ddot{\theta} = K_t I_q - T_L \implies \ddot{\theta} = \frac{K_t}{J} I_q - \frac{T_L}{J} @f]
 * Define state variables as tracking errors: @f$ x_1 = \theta_{ref} - \theta @f$, @f$ x_2 = \dot{\theta}_{ref} - \dot{\theta} @f$.
 * Then @f$ \dot{x}_1 = x_2 @f$ and @f$ \dot{x}_2 = \ddot{\theta}_{ref} - \frac{K_t}{J}I_q + \frac{T_L}{J} @f$.
 * * **2. Sliding Surface Design:**
 * @f[ s = \lambda x_1 + x_2 @f]
 * To reach and stay on the sliding surface (@f$ s \dot{s} < 0 @f$), let @f$ \dot{s} = 0 @f$ to find Equivalent Control:
 * @f[ \dot{s} = \lambda x_2 + \ddot{\theta}_{ref} - \frac{K_t}{J}I_{q\_eq} = 0 \implies I_{q\_eq} = \frac{J}{K_t}(\lambda x_2 + \ddot{\theta}_{ref}) @f]
 * * **3. Mapping to `smc.h` Structure:**
 * The standard SMC structure given is: @f$ u = \eta_1 x_1 + \eta_2 x_2 + \rho \text{sgn}(s) @f$.
 * Equating this to our Equivalent Control (handling @f$ \ddot{\theta}_{ref} @f$ via external feedforward):
 * - @f$ \eta_1 = 0 @f$ (No position proportional term needed in pure inertia equivalent control).
 * - @f$ \eta_2 = \frac{J \lambda}{K_t} @f$ (Velocity proportional equivalent term).
 * - @f$ \rho = \frac{T_{reject}}{K_t} @f$ (Switching gain required to overcome unknown disturbance @f$ T_L @f$).
 */
void ctl_autotuning_smc_mech_ctrl(ctl_smc_mech_init_t* init)
{
    // Protect against division by zero
    parameter_gt kt = (init->torque_const > 1e-6f) ? init->torque_const : 1.0f;

    // 1. Sliding surface slope (lambda) defines the error decay rate e^{-\lambda t}
    // Set lambda = 2 * PI * target_bw
    init->lambda = CTL_PARAM_CONST_2PI * init->target_bw;

    // 2. Equivalent control gains mapping (\eta_1 and \eta_2)
    // No x1 term in the inertia-only mechanical equivalent model
    init->eta11 = 0.0f;
    init->eta12 = 0.0f;

    // The x2 term (\eta_2) provides the equivalent damping force to stay on the surface
    parameter_gt eta2_val = (init->inertia * init->lambda) / kt;
    init->eta21 = eta2_val;
    init->eta22 = eta2_val;

    // 3. Switching Gain (\rho)
    // Must be large enough to overcome the maximum disturbance torque
    init->rho = init->dist_reject_torque / kt;

    // 4. Feedforward Gain (for external acceleration reference)
    init->k_ff = init->inertia / kt;
}

void ctl_init_smc_mech_ctrl(ctl_smc_mech_ctrl_t* ctrl, const ctl_smc_mech_init_t* init)
{
    // Initialize the internal Sliding Mode Controller core
    ctl_init_smc(&ctrl->smc_core, float2ctrl(init->eta11), float2ctrl(init->eta12), float2ctrl(init->eta21),
                 float2ctrl(init->eta22), float2ctrl(init->rho), float2ctrl(init->lambda));

    // Transfer limits and feedforward gain
    ctrl->cur_limit = float2ctrl(init->cur_limit);
    ctrl->k_ff = float2ctrl(init->k_ff);

    // Setup divider
    ctl_init_divider(&ctrl->div_mech, init->mech_division);

    // Clear interfaces and states
    ctrl->pos_if = NULL;
    ctrl->spd_if = NULL;
    ctrl->flag_enable = 0;
    ctrl->cur_output = float2ctrl(0.0f);
}
