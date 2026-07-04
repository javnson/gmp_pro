/**
 * @file ctl_discrete_pr_tuner.c
 * @brief Realization of standardized tuning conversion, verification, and backward decompilation.
 */

#include <gmp_core.h>

#include <ctl/component/intrinsic/discrete/proportional_resonant_tuner.h>

/*---------------------------------------------------------------------------*/
/* Pure Resonant (R) Tuner Realization Engine                                */
/*---------------------------------------------------------------------------*/

void ctl_tune_resonant_compile(ctl_resonant_tuner_t* tuner, parameter_gt fs)
{
    gmp_base_assert(tuner != NULL);
    ctl_calc_resonant_ctrl_coef(&tuner->shadow_coef, tuner->target_kr, tuner->target_freq_resonant, fs);
    tuner->flag_update_pending = 1;
}

void ctl_tune_resonant(ctl_resonant_tuner_t* tuner, parameter_gt kr, parameter_gt freq_resonant, parameter_gt fs)
{
    gmp_base_assert(tuner != NULL);
    tuner->target_kr = kr;
    tuner->target_freq_resonant = freq_resonant;
    ctl_tune_resonant_compile(tuner, fs);
}

void ctl_init_resonant_tuner_from_ctrl(ctl_resonant_tuner_t* tuner, const resonant_ctrl_t* active_ctrl, parameter_gt fs)
{
    gmp_base_assert(tuner != NULL);
    gmp_base_assert(active_ctrl != NULL);
    gmp_base_assert(fs > 0.0f);

    parameter_gt float_a1 = ctrl2float(active_ctrl->coef.a1);
    parameter_gt float_a2 = ctrl2float(active_ctrl->coef.a2);
    parameter_gt float_b0 = ctrl2float(active_ctrl->coef.b0);
    parameter_gt float_b2 = ctrl2float(active_ctrl->coef.b2);

    /* Mathematical Structural Consistency Verification */
    if ((fabsf(float_a2 - (-1.0f)) > 1e-4f) || (fabsf(float_b0 + float_b2) > 1e-4f))
    {
        tuner->target_kr = 0.0f;
        tuner->target_freq_resonant = 0.0f;
        tuner->shadow_coef.b0 = float2ctrl(0.0f);
        tuner->shadow_coef.b2 = float2ctrl(0.0f);
        tuner->shadow_coef.a1 = float2ctrl(2.0f);
        tuner->shadow_coef.a2 = float2ctrl(-1.0f);
        tuner->flag_update_pending = 0;
        return;
    }

    parameter_gt T = 1.0f / fs;
    parameter_gt wr_sq_T_sq = 4.0f * (2.0f - float_a1) / (2.0f + float_a1);
    if (wr_sq_T_sq < 0.0f)
        wr_sq_T_sq = 0.0f;

    parameter_gt wr = sqrtf(wr_sq_T_sq);
    tuner->target_freq_resonant = wr / (CTL_PARAM_CONST_2PI * T);

    parameter_gt den = wr_sq_T_sq + 4.0f;
    tuner->target_kr = (float_b0 * den) / (2.0f * T);

    tuner->shadow_coef = active_ctrl->coef;
    tuner->flag_update_pending = 0;
}

void ctl_init_tunable_resonant_controller(resonant_ctrl_t* r, ctl_resonant_tuner_t* tuner, parameter_gt kr,
                                          parameter_gt freq_resonant, parameter_gt fs)
{
    gmp_base_assert(r != NULL);
    gmp_base_assert(tuner != NULL);

    ctl_tune_resonant(tuner, kr, freq_resonant, fs);
    r->coef = tuner->shadow_coef;
    ctl_clear_resonant_controller(r);
    tuner->flag_update_pending = 0;
}

/*---------------------------------------------------------------------------*/
/* Quasi-Resonant (QR) Tuner Realization Engine                              */
/*---------------------------------------------------------------------------*/

void ctl_tune_qr_compile(ctl_qr_tuner_t* tuner, parameter_gt fs)
{
    gmp_base_assert(tuner != NULL);
    gmp_base_assert(fs > 0.0f);

    /* 1. Rigid Nyquist Guardrails Enforcement for QR Tuning Targets */
    if ((tuner->target_freq_resonant <= 0.0f) || (tuner->target_freq_resonant >= (fs * 0.5f)) ||
        (tuner->target_freq_cut <= 0.0f) || (tuner->target_freq_cut >= (fs * 0.5f)) || (tuner->target_kr < 0.0f))
    {
        return;
    }

    parameter_gt wr = CTL_PARAM_CONST_2PI * tuner->target_freq_resonant;
    parameter_gt wc = CTL_PARAM_CONST_2PI * tuner->target_freq_cut;
    parameter_gt k_val = 2.0f * fs; /* Standard Tustin default operator */

    /* Fixed: Properly map and route Prewarped frequency correction modal branch */
    if (tuner->method_mode == CTL_TUNE_QR_PREWARPED)
    {
        parameter_gt half_angle = CTL_PARAM_CONST_PI * tuner->target_freq_resonant / fs;
        if (half_angle < 1e-6f)
            half_angle = 1e-6f;
        if (half_angle > (CTL_PARAM_CONST_PI * 0.5f - 1e-6f))
            half_angle = (CTL_PARAM_CONST_PI * 0.5f - 1e-6f);

        k_val = wr / tanf(half_angle);
    }

    /* Compile parameters safely through floating core engine */
    ctl_calc_qr_ctrl_coef(&tuner->shadow_coef, tuner->target_kr, wc, wr, k_val);
    tuner->flag_update_pending = 1;
}

/* Fixed: Aligned messy interface parameters naming block back to contract */
void ctl_tune_qr(ctl_qr_tuner_t* tuner, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                 ctl_tune_qr_mode_e mode, parameter_gt fs)
{
    gmp_base_assert(tuner != NULL);

    tuner->target_kr = kr;
    tuner->target_freq_resonant = freq_resonant;
    tuner->target_freq_cut = freq_cut;
    tuner->method_mode = mode;

    ctl_tune_qr_compile(tuner, fs);
}

void ctl_init_qr_tuner_from_ctrl(ctl_qr_tuner_t* tuner, const qr_ctrl_t* active_ctrl, ctl_tune_qr_mode_e mode,
                                 parameter_gt fs)
{
    gmp_base_assert(tuner != NULL);
    gmp_base_assert(active_ctrl != NULL);
    gmp_base_assert(fs > 0.0f);

    parameter_gt float_b0 = ctrl2float(active_ctrl->coef.b0);
    parameter_gt float_b2 = ctrl2float(active_ctrl->coef.b2);
    parameter_gt float_a1 = ctrl2float(active_ctrl->coef.a1);
    parameter_gt float_a2 = ctrl2float(active_ctrl->coef.a2);

    /* Mathematical Consistency Verification */
    if (fabsf(float_b0 + float_b2) > 1e-4f)
    {
        tuner->target_kr = 0.0f;
        tuner->target_freq_resonant = 0.0f;
        tuner->target_freq_cut = 0.0f;
        tuner->method_mode = mode;
        tuner->shadow_coef.b0 = float2ctrl(0.0f);
        tuner->shadow_coef.b2 = float2ctrl(0.0f);
        tuner->shadow_coef.a1 = float2ctrl(2.0f);
        tuner->shadow_coef.a2 = float2ctrl(-1.0f);
        tuner->flag_update_pending = 0;
        return;
    }

    tuner->method_mode = mode;
    parameter_gt k_tustin = 2.0f * fs;

    /* Multi-variable backward deconvolution decoupling */
    parameter_gt num_ratio = 2.0f - 2.0f * float_a2 - 2.0f * float_a1;
    parameter_gt den_ratio = 2.0f - 2.0f * float_a2 + 2.0f * float_a1;
    if (den_ratio < 1e-9f)
        den_ratio = 1e-9f;

    parameter_gt wr_sq_derived = (k_tustin * k_tustin) * (num_ratio / den_ratio);
    if (wr_sq_derived < 0.0f)
        wr_sq_derived = 0.0f;
    parameter_gt wr_derived = sqrtf(wr_sq_derived);

    if (mode == CTL_TUNE_QR_PREWARPED)
    {
        parameter_gt half_angle = wr_derived * (0.5f / fs);
        if (half_angle < 1e-6f)
            half_angle = 1e-6f;
        if (half_angle > (CTL_PARAM_CONST_PI * 0.5f - 1e-6f))
            half_angle = (CTL_PARAM_CONST_PI * 0.5f - 1e-6f);
        k_tustin = wr_derived / tanf(half_angle);

        wr_sq_derived = (k_tustin * k_tustin) * (num_ratio / den_ratio);
        if (wr_sq_derived < 0.0f)
            wr_sq_derived = 0.0f;
        wr_derived = sqrtf(wr_sq_derived);
    }
    tuner->target_freq_resonant = wr_derived / CTL_PARAM_CONST_2PI;

    parameter_gt D0_derived = (2.0f * k_tustin * k_tustin - 2.0f * wr_sq_derived) / float_a1;
    if (fabsf(float_a1) < 1e-6f)
    {
        D0_derived = (4.0f * k_tustin * k_tustin) / (1.0f + float_a1 - float_a2);
    }

    parameter_gt wc_derived = (float_a2 * D0_derived + (k_tustin * k_tustin) + wr_sq_derived) / (2.0f * k_tustin);
    if (wc_derived < 0.0f)
        wc_derived = 0.0f;
    tuner->target_freq_cut = wc_derived / CTL_PARAM_CONST_2PI;

    if (wc_derived > 1e-6f)
    {
        tuner->target_kr = (float_b0 * D0_derived) / (2.0f * wc_derived * k_tustin);
    }
    else
    {
        tuner->target_kr = 0.0f;
    }

    tuner->shadow_coef = active_ctrl->coef;
    tuner->flag_update_pending = 0;
}

void ctl_init_tunable_qr_controller(qr_ctrl_t* r, ctl_qr_tuner_t* tuner, parameter_gt kr, parameter_gt freq_resonant,
                                    parameter_gt freq_cut, ctl_tune_qr_mode_e mode, parameter_gt fs)
{
    gmp_base_assert(r != NULL);
    gmp_base_assert(tuner != NULL);

    ctl_tune_qr(tuner, kr, freq_resonant, freq_cut, mode, fs);
    r->coef = tuner->shadow_coef;
    ctl_clear_qr_controller(r);
    tuner->flag_update_pending = 0;
}
