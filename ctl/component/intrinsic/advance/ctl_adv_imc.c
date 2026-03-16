#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// IMC
#include <ctl/component/intrinsic/advance/imc.h>

int ctl_init_imc(ctl_imc_controller_t* imc, const ctl_imc_init_t* init)
{
    ctrl_gt Ts = 1.0f / (ctrl_gt)init->f_ctrl;

    // --- Discretize Plant Model (ZOH) ---
    imc->a_p_d = expf(-(Ts / (ctrl_gt)init->tau_p));
    imc->b_p_d = (ctrl_gt)init->K_p * (1.0f - imc->a_p_d);

    // --- Calculate Dead Time ---
    imc->dead_time_samples = (uint16_t)roundf((ctrl_gt)init->theta_p / Ts);
    if (imc->dead_time_samples >= IMC_MAX_DEAD_TIME_SAMPLES)
    {
        return -1; // Error: Dead time exceeds buffer size
    }

    // --- Discretize IMC Controller Q(s) (Tustin's method) ---
    // Q(s) = (1/Kp) * (tau_p*s + 1) / (lambda*s + 1)
    ctrl_gt lambda = (ctrl_gt)init->lambda;
    ctrl_gt tau_p = (ctrl_gt)init->tau_p;
    ctrl_gt K_p = (ctrl_gt)init->K_p;

    imc->a_q_d = (2.0f * lambda - Ts) / (2.0f * lambda + Ts);
    imc->b0_q_d = (1.0f / K_p) * (2.0f * tau_p + Ts) / (2.0f * lambda + Ts);
    imc->b1_q_d = (1.0f / K_p) * (Ts - 2.0f * tau_p) / (2.0f * lambda + Ts);

    // Reset states
    ctl_clear_imc(imc);
    return 0;
}
