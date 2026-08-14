#include <ctl/math_block/gmp_math.h>

#include <ctl/component/digital_power/inv/inv_zero_ctrl.h>

void ctl_auto_tuning_zero_inv(inv_zero_ctrl_init_t* zero_init, const gfl_inv_ctrl_init_t* gfl_init)
{
    parameter_gt loop_bw;

    gmp_ctl_assert(zero_init);
    gmp_ctl_assert(gfl_init);
    gmp_ctl_assert(gfl_init->fs > 0.0f);
    gmp_ctl_assert(gfl_init->grid_filter_L > 0.0f);
    gmp_ctl_assert(gfl_init->v_base > 0.0f);

    loop_bw = gfl_init->current_loop_bw;

    zero_init->fs = gfl_init->fs;
    zero_init->kp =
        CTL_PARAM_CONST_2PI * loop_bw * gfl_init->grid_filter_L * gfl_init->i_base / gfl_init->v_base;
    zero_init->kr = 5.0f * zero_init->kp;
    zero_init->freq_resonant = gfl_init->freq_base;
    zero_init->freq_cut = gfl_init->freq_base / 10.0f;
    zero_init->tune_mode = CTL_TUNE_QR_PREWARPED;
    zero_init->output_limit_max = 0.5f;
    zero_init->output_limit_min = -0.5f;
}

void ctl_update_zero_inv_coeff(inv_zero_ctrl_t* zero, const inv_zero_ctrl_init_t* init)
{
    gmp_ctl_assert(zero);
    gmp_ctl_assert(init);
    gmp_ctl_assert(init->fs > 0.0f);
    gmp_ctl_assert(init->freq_resonant > 0.0f);
    gmp_ctl_assert(init->freq_resonant < init->fs * 0.5f);
    gmp_ctl_assert(init->freq_cut > 0.0f);
    gmp_ctl_assert(init->freq_cut < init->fs * 0.5f);
    gmp_ctl_assert(init->output_limit_max >= init->output_limit_min);

    ctl_init_tunable_qr_controller(&zero->qpr.resonant_part, &zero->tuner, init->kr,
                                   init->freq_resonant, init->freq_cut, init->tune_mode, init->fs);
    zero->qpr.kp = float2ctrl(init->kp);
    zero->kp_shadow = zero->qpr.kp;
    zero->output_limit_max = float2ctrl(init->output_limit_max);
    zero->output_limit_min = float2ctrl(init->output_limit_min);
    zero->flag_tune_pending = 0;
}

void ctl_init_zero_inv(inv_zero_ctrl_t* zero, const inv_zero_ctrl_init_t* init)
{
    gmp_ctl_assert(zero);

    zero->i0 = NULL;
    zero->v0_sink = NULL;
    zero->i0_set = float2ctrl(0.0f);

    ctl_update_zero_inv_coeff(zero, init);
    ctl_clear_zero_inv(zero);
    zero->flag_enable = 0;
}

void ctl_tune_zero_inv_ctrl(inv_zero_ctrl_t* zero, parameter_gt kp, parameter_gt kr,
                            parameter_gt freq_resonant, parameter_gt freq_cut,
                            ctl_tune_qr_mode_e mode, parameter_gt fs)
{
    gmp_ctl_assert(zero);

    if ((fs <= 0.0f) || (freq_resonant <= 0.0f) || (freq_resonant >= fs * 0.5f) ||
        (freq_cut <= 0.0f) || (freq_cut >= fs * 0.5f) || (kp < 0.0f) || (kr < 0.0f))
    {
        return;
    }

    ctl_tune_qr(&zero->tuner, kr, freq_resonant, freq_cut, mode, fs);
    if (zero->tuner.flag_update_pending)
    {
        zero->kp_shadow = float2ctrl(kp);
        zero->flag_tune_pending = 1;
    }
}
