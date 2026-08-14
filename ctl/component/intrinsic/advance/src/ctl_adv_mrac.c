#include <ctl/math_block/gmp_math.h>

//////////////////////////////////////////////////////////////////////////
// MRAC
#include <ctl/component/intrinsic/advance/mrac.h>

void ctl_init_mrac(ctl_mrac_controller_t* mrac, const ctl_mrac_init_t* init)
{
    // 1. Validate the model and adaptation parameters.
    gmp_ctl_assert(init->f_ctrl > 0.0f);
    gmp_ctl_assert(init->a_m > CTL_PARAM_CONST_EPSILON); // a_m is used as a denominator.

    // 2. Calculate coefficients in the parameter domain.
    parameter_gt Ts = 1.0f / init->f_ctrl;

    // Discretize the reference model using Zero-Order Hold (ZOH)
    parameter_gt a_m_d_f = param_exp(-(init->a_m * Ts));
    parameter_gt b_m_d_f = (init->b_m / init->a_m) * (1.0f - a_m_d_f);

    // Discretize the adaptation rates
    parameter_gt gamma_r_d_f = init->gamma_r * Ts;
    parameter_gt gamma_y_d_f = init->gamma_y * Ts;

    // 3. Quantize only the final coefficients into the control domain.
    mrac->a_m_d = real2ctrl(a_m_d_f);
    mrac->b_m_d = real2ctrl(b_m_d_f);
    mrac->gamma_r_d = real2ctrl(gamma_r_d_f);
    mrac->gamma_y_d = real2ctrl(gamma_y_d_f);

    // Reset states
    ctl_clear_mrac(mrac);
}
