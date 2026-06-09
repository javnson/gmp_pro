/**
 * @file ctl_discrete_pr.c
 * @brief Core algebraic discretization calculators and basic non-tunable initializers.
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// PR / QPR controller

#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

void ctl_calc_resonant_ctrl_coef(ctl_resonant_coef_t* coef, parameter_gt target_kr, parameter_gt target_freq_resonant,
                                 parameter_gt fs)
{
    gmp_base_assert(coef != NULL);
    gmp_base_assert(fs > 0.0f);

    /* 1. Rigid Nyquist Guardrails Enforcement */
    if ((target_freq_resonant <= 0.0f) || (target_freq_resonant >= (fs * 0.5f)) || (target_kr < 0.0f))
    {
        return; /* Safeguard against unit-circle overflow */
    }

    /* 2. Bilinear Tustin Discretization Calculation Mapping */
    parameter_gt T = 1.0f / fs;
    parameter_gt wr = CTL_PARAM_CONST_2PI * target_freq_resonant;
    parameter_gt wr_sq_T_sq = wr * wr * T * T;
    parameter_gt den = wr_sq_T_sq + 4.0f;
    parameter_gt inv_den = 1.0f / den;

    /* Secure floating-point conversion down into ctrl_gt fixed representation bounds */
    coef->b0 = float2ctrl(target_kr * 2.0f * T * inv_den);
    coef->b2 = float2ctrl(-target_kr * 2.0f * T * inv_den);
    coef->a1 = float2ctrl(2.0f * (4.0f - wr_sq_T_sq) * inv_den);
    coef->a2 = float2ctrl(-1.0f);
}

void ctl_init_resonant_controller(resonant_ctrl_t* r, parameter_gt kr, parameter_gt freq_resonant, parameter_gt fs)
{
    gmp_base_assert(r != NULL);
    ctl_calc_resonant_ctrl_coef(&r->coef, kr, freq_resonant, fs);
    ctl_clear_resonant_controller(r);
}

void ctl_init_pr_controller(pr_ctrl_t* pr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                            parameter_gt fs)
{
    gmp_base_assert(pr != NULL);
    pr->kp = float2ctrl(kp);
    ctl_init_resonant_controller(&pr->resonant_part, kr, freq_resonant, fs);
}

void ctl_calc_qr_ctrl_coef(ctl_qr_coef_t* coef, parameter_gt kr, parameter_gt wc, parameter_gt wr,
                           parameter_gt k_tustin)
{
    gmp_base_assert(coef != NULL);

    parameter_gt k_sq = k_tustin * k_tustin;
    parameter_gt wr_sq = wr * wr;
    parameter_gt D0 = k_sq + (2.0f * wc * k_tustin) + wr_sq;

    if (D0 < 1e-9f)
    {
        D0 = 1e-9f; /* Guard rails against math exceptions */
    }
    parameter_gt inv_D0 = 1.0f / D0;

    /* Crucial Optimization Fix: Extracted float evaluations wrapped safely into float2ctrl macros */
    parameter_gt b0_val = (2.0f * kr * wc * k_tustin) * inv_D0;
    coef->b0 = float2ctrl(b0_val);
    coef->b2 = float2ctrl(-b0_val); /* Corrected previous variable typo block assignment */

    coef->a1 = float2ctrl((2.0f * k_sq - 2.0f * wr_sq) * inv_D0);
    coef->a2 = float2ctrl((2.0f * wc * k_tustin - k_sq - wr_sq) * inv_D0);
}

void ctl_init_qr_controller(qr_ctrl_t* qr, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                            parameter_gt fs)
{
    gmp_base_assert(qr != NULL);
    gmp_base_assert(fs > 0.0f);

    parameter_gt wr = CTL_PARAM_CONST_2PI * freq_resonant;
    parameter_gt wc = CTL_PARAM_CONST_2PI * freq_cut;
    parameter_gt k_val = 2.0f * fs;

    /* Fixed: Redirected to specific QR algebraic engine instead of pure resonant calculation */
    ctl_calc_qr_ctrl_coef(&qr->coef, kr, wc, wr, k_val);
    ctl_clear_qr_controller(qr);
}

void ctl_init_qr_controller_prewarped(qr_ctrl_t* qr, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                                      parameter_gt fs)
{
    gmp_base_assert(qr != NULL);
    gmp_base_assert(fs > 0.0f);

    parameter_gt wr = CTL_PARAM_CONST_2PI * freq_resonant;
    parameter_gt wc = CTL_PARAM_CONST_2PI * freq_cut;

    parameter_gt half_angle = CTL_PARAM_CONST_PI * freq_resonant / fs;
    if (half_angle < 1e-6f)
        half_angle = 1e-6f;
    if (half_angle > (CTL_PARAM_CONST_PI * 0.5f - 1e-6f))
        half_angle = (CTL_PARAM_CONST_PI * 0.5f - 1e-6f);

    parameter_gt k_pre = wr / tanf(half_angle);

    ctl_calc_qr_ctrl_coef(&qr->coef, kr, wc, wr, k_pre);
    ctl_clear_qr_controller(qr);
}

void ctl_init_qpr_controller(qpr_ctrl_t* qpr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                             parameter_gt freq_cut, parameter_gt fs)
{
    gmp_base_assert(qpr != NULL);
    qpr->kp = float2ctrl(kp);
    ctl_init_qr_controller(&qpr->resonant_part, kr, freq_resonant, freq_cut, fs);
}

void ctl_init_qpr_controller_prewarped(qpr_ctrl_t* qpr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                                       parameter_gt freq_cut, parameter_gt fs)
{
    gmp_base_assert(qpr != NULL);
    qpr->kp = float2ctrl(kp);
    ctl_init_qr_controller_prewarped(&qpr->resonant_part, kr, freq_resonant, freq_cut, fs);
}
