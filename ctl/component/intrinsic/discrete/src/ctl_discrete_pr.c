/**
 * @file ctl_discrete_pr.c
 * @brief Core algebraic discretization calculators and basic non-tunable initializers.
 */

#include <ctl/math_block/gmp_math.h>

//////////////////////////////////////////////////////////////////////////
// PR / QPR controller

#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

void ctl_calc_resonant_ctrl_coef(ctl_resonant_coef_t* coef, parameter_gt target_kr, parameter_gt target_freq_resonant,
                                 parameter_gt fs)
{
    gmp_ctl_assert(coef != NULL);
    gmp_ctl_assert(fs > CTL_PARAM_CONST_ZERO);

    /* 1. Rigid Nyquist Guardrails Enforcement */
    if ((target_freq_resonant <= CTL_PARAM_CONST_ZERO) ||
        (target_freq_resonant >= (fs * CTL_PARAM_CONST_1_OVER_2)) || (target_kr < CTL_PARAM_CONST_ZERO))
    {
        return; /* Safeguard against unit-circle overflow */
    }

    /* 2. Bilinear Tustin Discretization Calculation Mapping */
    parameter_gt T = CTL_PARAM_CONST_1 / fs;
    parameter_gt wr = CTL_PARAM_CONST_2PI * target_freq_resonant;
    parameter_gt wr_sq_T_sq = wr * wr * T * T;
    parameter_gt den = wr_sq_T_sq + CTL_PARAM_CONST_4;
    parameter_gt inv_den = CTL_PARAM_CONST_1 / den;

    /* Secure floating-point conversion down into ctrl_gt fixed representation bounds */
    coef->b0 = param2ctrl(target_kr * CTL_PARAM_CONST_2 * T * inv_den);
    coef->b2 = param2ctrl(-target_kr * CTL_PARAM_CONST_2 * T * inv_den);
    coef->a1 = param2ctrl(CTL_PARAM_CONST_2 * (CTL_PARAM_CONST_4 - wr_sq_T_sq) * inv_den);
    coef->a2 = (-CTL_CTRL_CONST_1);
}

void ctl_init_resonant_controller(resonant_ctrl_t* r, parameter_gt kr, parameter_gt freq_resonant, parameter_gt fs)
{
    gmp_ctl_assert(r != NULL);
    ctl_calc_resonant_ctrl_coef(&r->coef, kr, freq_resonant, fs);
    ctl_clear_resonant_controller(r);
}

void ctl_init_pr_controller(pr_ctrl_t* pr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                            parameter_gt fs)
{
    gmp_ctl_assert(pr != NULL);
    pr->kp = param2ctrl(kp);
    ctl_init_resonant_controller(&pr->resonant_part, kr, freq_resonant, fs);
}

void ctl_calc_qr_ctrl_coef(ctl_qr_coef_t* coef, parameter_gt kr, parameter_gt wc, parameter_gt wr,
                           parameter_gt k_tustin)
{
    gmp_ctl_assert(coef != NULL);

    parameter_gt k_sq = k_tustin * k_tustin;
    parameter_gt wr_sq = wr * wr;
    parameter_gt D0 = k_sq + (CTL_PARAM_CONST_2 * wc * k_tustin) + wr_sq;

    if (D0 < CTL_PARAM_CONST_EPSILON)
    {
        D0 = CTL_PARAM_CONST_EPSILON; /* Guard against division by zero. */
    }
    parameter_gt inv_D0 = CTL_PARAM_CONST_1 / D0;

    /* Calculate completely in parameter_gt, then quantize each final coefficient once. */
    parameter_gt b0_val = (CTL_PARAM_CONST_2 * kr * wc * k_tustin) * inv_D0;
    coef->b0 = param2ctrl(b0_val);
    coef->b2 = param2ctrl(-b0_val);

    coef->a1 = param2ctrl((CTL_PARAM_CONST_2 * k_sq - CTL_PARAM_CONST_2 * wr_sq) * inv_D0);
    coef->a2 = param2ctrl((CTL_PARAM_CONST_2 * wc * k_tustin - k_sq - wr_sq) * inv_D0);
}

void ctl_init_qr_controller(qr_ctrl_t* qr, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                            parameter_gt fs)
{
    gmp_ctl_assert(qr != NULL);
    gmp_ctl_assert(fs > CTL_PARAM_CONST_ZERO);

    parameter_gt wr = CTL_PARAM_CONST_2PI * freq_resonant;
    parameter_gt wc = CTL_PARAM_CONST_2PI * freq_cut;
    parameter_gt k_val = CTL_PARAM_CONST_2 * fs;

    /* Fixed: Redirected to specific QR algebraic engine instead of pure resonant calculation */
    ctl_calc_qr_ctrl_coef(&qr->coef, kr, wc, wr, k_val);
    ctl_clear_qr_controller(qr);
}

void ctl_init_qr_controller_prewarped(qr_ctrl_t* qr, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                                      parameter_gt fs)
{
    gmp_ctl_assert(qr != NULL);
    gmp_ctl_assert(fs > CTL_PARAM_CONST_ZERO);

    parameter_gt wr = CTL_PARAM_CONST_2PI * freq_resonant;
    parameter_gt wc = CTL_PARAM_CONST_2PI * freq_cut;

    parameter_gt half_angle = CTL_PARAM_CONST_PI * freq_resonant / fs;
    if (half_angle < CTL_PARAM_CONST_EPSILON)
        half_angle = CTL_PARAM_CONST_EPSILON;
    if (half_angle > (CTL_PARAM_CONST_PI * CTL_PARAM_CONST_1_OVER_2 - CTL_PARAM_CONST_EPSILON))
        half_angle = CTL_PARAM_CONST_PI * CTL_PARAM_CONST_1_OVER_2 - CTL_PARAM_CONST_EPSILON;

    parameter_gt k_pre = wr / param_tan(half_angle);

    ctl_calc_qr_ctrl_coef(&qr->coef, kr, wc, wr, k_pre);
    ctl_clear_qr_controller(qr);
}

void ctl_init_qpr_controller(qpr_ctrl_t* qpr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                             parameter_gt freq_cut, parameter_gt fs)
{
    gmp_ctl_assert(qpr != NULL);
    qpr->kp = param2ctrl(kp);
    ctl_init_qr_controller(&qpr->resonant_part, kr, freq_resonant, freq_cut, fs);
}

void ctl_init_qpr_controller_prewarped(qpr_ctrl_t* qpr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                                       parameter_gt freq_cut, parameter_gt fs)
{
    gmp_ctl_assert(qpr != NULL);
    qpr->kp = param2ctrl(kp);
    ctl_init_qr_controller_prewarped(&qpr->resonant_part, kr, freq_resonant, freq_cut, fs);
}
