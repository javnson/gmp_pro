/**
 * @file ctl_resonant_tuner.c
 * @brief Realization of unified mathematical conversions and single-line flash reuse.
 */

#include <gmp_core.h>

#include <ctl/component/intrinsic/discrete/proportional_resonant_tuner.h>

void ctl_tune_resonant_compile(ctl_resonant_tuner_t* tuner, parameter_gt fs)
{
    ctl_calc_resonant_ctrl_coef(&tuner->shadow_coef, tuner->target_kr, tuner->target_freq_resonant, fs);

    /* Arm the flag for Main ISR deployment synchronization */
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

    /* Mathematical Consistency Verification */
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

    /* Reverse-engineer coefficients back to physical fields */
    parameter_gt T = 1.0f / fs;
    parameter_gt wr_sq_T_sq = 4.0f * (2.0f - float_a1) / (2.0f + float_a1);
    if (wr_sq_T_sq < 0.0f)
        wr_sq_T_sq = 0.0f;

    parameter_gt wr = sqrtf(wr_sq_T_sq);
    tuner->target_freq_resonant = wr / (CTL_PARAM_CONST_2PI * T);

    parameter_gt den = wr_sq_T_sq + 4.0f;
    tuner->target_kr = (float_b0 * den) / (2.0f * T);

    /* Direct block copy to synchronize current operational real assets */
    tuner->shadow_coef = active_ctrl->coef;
    tuner->flag_update_pending = 0;
}

void ctl_init_tunable_resonant_controller(resonant_ctrl_t* r, ctl_resonant_tuner_t* tuner, parameter_gt kr,
                                          parameter_gt freq_resonant, parameter_gt fs)
{
    gmp_base_assert(r != NULL);
    gmp_base_assert(tuner != NULL);

    /* 1. Setup the Tuner Companion using standard pipelines */
    ctl_tune_resonant(tuner, kr, freq_resonant, fs);

    /* 2. Directly flash parameters into the controller registers for instant cold start */
    r->coef = tuner->shadow_coef;

    /* 3. Clear historical error/output state accumulators */
    ctl_clear_resonant_controller(r);

    /* 4. Flush the update pending flag since entities are synchronized at birth */
    tuner->flag_update_pending = 0;
}

/**
 * @brief Standardized standalone initialization for the primary execution controller.
 * @details ADVANCED FLASH REUSE: Allocates a temporary runtime tuner on the stack 
 * to redirect calculations through ctl_init_tunable_resonant_controller, completely 
 * erasing redundant Bilinear transformation assembly footprints from the flash.
 */
void ctl_init_resonant_controller(resonant_ctrl_t* r, parameter_gt kr, parameter_gt freq_resonant, parameter_gt fs)
{
    ctl_resonant_tuner_t stack_temporary_tuner;

    /* Redirect and pipe everything into the unified calculation engine */
    ctl_init_tunable_resonant_controller(r, &stack_temporary_tuner, kr, freq_resonant, fs);
}

void ctl_tune_qr_compile(ctl_qr_tuner_t* tuner, parameter_gt fs)
{
    // Standard Tustin K = 2 * Fs
    parameter_gt k_val = 2.0f * fs;

    ctl_calc_resonant_ctrl_coef(&tuner->shadow_coef, tuner->target_kr, tuner->target_freq_resonant,
                                tuner->target_freq_cut, k_val);

    /* Arm the flag for Main ISR deployment synchronization */
    tuner->flag_update_pending = 1;
}

void ctl_tune_qr(ctl_resonant_tuner_t* tuner, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                 parameter_gt fs)
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

    /* Mathematical Consistency Verification: check numerator perfect anti-symmetry */
    if (fabsf(float_b0 + float_b2) > 1e-4f)
    {
        /* Safe default fallback for uninitialized black-box core */
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

    /* Analytical multi-variable backward deconvolution decoupling */
    /* Step A: Extract center frequency utilizing algebraic ratio properties */
    /* Continuous-discrete mapping ratio: wr_sq = k_sq * (2 - 2*a2 - 2*a1) / (2 - 2*a2 + 2*a1) */
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
        /* In prewarped mode, the active_ctrl was generated under modified K_pre. 
           We must re-evaluate K based on the newly extracted target wr. */
        parameter_gt half_angle = wr_derived * (0.5f / fs);
        if (half_angle < 1e-6f)
            half_angle = 1e-6f;
        if (half_angle > (CTL_PARAM_CONST_PI * 0.5f - 1e-6f))
            half_angle = (CTL_PARAM_CONST_PI * 0.5f - 1e-6f);
        k_tustin = wr_derived / tanf(half_angle);

        /* Re-run frequency matching with the accurate K_pre scaling factor */
        wr_sq_derived = (k_tustin * k_tustin) * (num_ratio / den_ratio);
        if (wr_sq_derived < 0.0f)
            wr_sq_derived = 0.0f;
        wr_derived = sqrtf(wr_sq_derived);
    }
    tuner->target_freq_resonant = wr_derived / CTL_PARAM_CONST_2PI;

    /* Step B: Decouple cutoff frequency from a2 attenuation constant */
    /* D0 = (2 * k_sq - 2 * wr_sq) / a1 */
    parameter_gt D0_derived = (2.0f * k_tustin * k_tustin - 2.0f * wr_sq_derived) / float_a1;
    if (fabsf(float_a1) < 1e-6f)
    {
        /* Alternate derivation from a2 if a1 hits zero crossing */
        D0_derived = (4.0f * k_tustin * k_tustin) / (1.0f + float_a1 - float_a2);
    }

    /* wc = (a2 * D0 + k_sq + wr_sq) / (2 * k_tustin) */
    parameter_gt wc_derived = (float_a2 * D0_derived + (k_tustin * k_tustin) + wr_sq_derived) / (2.0f * k_tustin);
    if (wc_derived < 0.0f)
        wc_derived = 0.0f;
    tuner->target_freq_cut = wc_derived / CTL_PARAM_CONST_2PI;

    /* Step C: Decouple kr gain asset from b0 multiplier */
    if (wc_derived > 1e-6f)
    {
        tuner->target_kr = (float_b0 * D0_derived) / (2.0f * wc_derived * k_tustin);
    }
    else
    {
        tuner->target_kr = 0.0f;
    }

    /* Pull structural images to seal synchronization */
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
