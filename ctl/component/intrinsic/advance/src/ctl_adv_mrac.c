#include <gmp_core.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// MRAC
#include <ctl/component/intrinsic/advance/mrac.h>

void ctl_init_mrac(ctl_mrac_controller_t* mrac, const ctl_mrac_init_t* init)
{
    // 1. 基础防呆保护
    gmp_base_assert(init->f_ctrl > 0.0f);
    gmp_base_assert(init->a_m > 1e-9f); // 防除零保护

    // 2. 在纯物理浮点域 (parameter_gt) 中进行高精度计算
    parameter_gt Ts = 1.0f / init->f_ctrl;

    // Discretize the reference model using Zero-Order Hold (ZOH)
    parameter_gt a_m_d_f = expf(-(init->a_m * Ts));
    parameter_gt b_m_d_f = (init->b_m / init->a_m) * (1.0f - a_m_d_f);

    // Discretize the adaptation rates
    parameter_gt gamma_r_d_f = init->gamma_r * Ts;
    parameter_gt gamma_y_d_f = init->gamma_y * Ts;

    // 3. 计算完毕后，安全固化到定点控制域
    mrac->a_m_d = float2ctrl(a_m_d_f);
    mrac->b_m_d = float2ctrl(b_m_d_f);
    mrac->gamma_r_d = float2ctrl(gamma_r_d_f);
    mrac->gamma_y_d = float2ctrl(gamma_y_d_f);

    // Reset states
    ctl_clear_mrac(mrac);
}
