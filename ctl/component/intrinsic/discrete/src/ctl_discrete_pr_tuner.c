/**
 * @file ctl_resonant_tuner.c
 * @brief Realization of unified mathematical conversions and single-line flash reuse.
 */

#include <gmp_core.h>

#include <ctl/component/intrinsic/discrete/proportional_resonant_tuner.h>

void ctl_tune_resonant_compile(ctl_resonant_tuner_t* tuner, parameter_gt fs)
{
    gmp_base_assert(tuner != NULL);
    gmp_base_assert(fs > 0.0f);

    /* 1. Rigid Nyquist Guardrails Enforcement */
    if ((tuner->target_freq_resonant <= 0.0f) || (tuner->target_freq_resonant >= (fs * 0.5f)) ||
        (tuner->target_kr < 0.0f))
    {
        return; /* Block compilation safely to protect loop from divergence or fixed-point overflow */
    }

    /* 2. Execute Tustin Bilinear Discretization Calculation */
    parameter_gt T = 1.0f / fs;
    parameter_gt wr = CTL_PARAM_CONST_2PI * tuner->target_freq_resonant;
    parameter_gt wr_sq_T_sq = wr * wr * T * T;
    parameter_gt den = wr_sq_T_sq + 4.0f;
    parameter_gt inv_den = 1.0f / den;

    tuner->shadow_coef.b0 = float2ctrl(tuner->target_kr * 2.0f * T * inv_den);
    tuner->shadow_coef.b2 = float2ctrl(-tuner->target_kr * 2.0f * T * inv_den);
    tuner->shadow_coef.a1 = float2ctrl(2.0f * (4.0f - wr_sq_T_sq) * inv_den);
    tuner->shadow_coef.a2 = float2ctrl(-1.0f);

    /* 3. Arm the flag for Main ISR deployment synchronization */
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
