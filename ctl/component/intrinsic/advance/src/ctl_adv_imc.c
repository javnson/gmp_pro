#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// IMC
#include <ctl/component/intrinsic/advance/imc.h>

int ctl_init_imc(ctl_imc_controller_t* imc, const ctl_imc_init_t* init)
{
    // 1. 防呆与除零保护
    gmp_base_assert(init->f_ctrl > 0.0);
    gmp_base_assert(fabsf(init->K_p) > 1e-9f); // Kp 不能为 0，因为后续要作除数

    // 2. 纯物理参数域计算，严禁出现 ctrl_gt 强转！
    parameter_gt Ts = 1.0 / init->f_ctrl;

    // --- Discretize Plant Model (ZOH) ---
    parameter_gt a_p_d_f = expf(-(Ts / init->tau_p));
    parameter_gt b_p_d_f = init->K_p * (1.0 - a_p_d_f);

    // --- Calculate Dead Time ---
    imc->dead_time_samples = (uint16_t)roundf(init->theta_p / Ts);
    if (imc->dead_time_samples >= IMC_MAX_DEAD_TIME_SAMPLES)
    {
        return -1; // Error: Dead time exceeds buffer size
    }

    // --- Discretize IMC Controller Q(s) (Tustin's method) ---
    // Q(s) = (1/Kp) * (tau_p*s + 1) / (lambda*s + 1)
    parameter_gt lambda = init->lambda;
    parameter_gt tau_p = init->tau_p;
    parameter_gt K_p = init->K_p;

    parameter_gt den = 2.0 * lambda + Ts;
    parameter_gt inv_den = 1.0 / den;

    parameter_gt a_q_d_f = (2.0 * lambda - Ts) * inv_den;
    parameter_gt b0_q_d_f = (2.0 * tau_p + Ts) * inv_den / K_p;
    parameter_gt b1_q_d_f = (Ts - 2.0 * tau_p) * inv_den / K_p;

    // 3. 所有高精度浮点计算完成后，安全固化到定点控制域！
    imc->a_p_d = float2ctrl(a_p_d_f);
    imc->b_p_d = float2ctrl(b_p_d_f);
    imc->a_q_d = float2ctrl(a_q_d_f);
    imc->b0_q_d = float2ctrl(b0_q_d_f);
    imc->b1_q_d = float2ctrl(b1_q_d_f);

    ctl_clear_imc(imc);
    return 0;
}
