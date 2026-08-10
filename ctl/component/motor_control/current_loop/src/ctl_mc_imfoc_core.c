/** @file ctl_mc_imfoc_core.c @brief Initialization for the standalone ACIM current loop. */

#include <gmp_core.h>
#include <ctl/component/motor_control/current_loop/imfoc_core.h>

//=================================================================================================
// Output limiting

void ctl_set_im_ifoc_saturation(im_ifoc_ctrl_t* mc, parameter_gt volt_rect_pu, parameter_gt volt_circle_pu)
{
    ctl_vector2_t max;
    ctl_vector2_t min;
    gmp_base_assert(mc && volt_rect_pu >= 0.0f && volt_circle_pu >= 0.0f);
    mc->max_vs_rect = float2ctrl(volt_rect_pu);
    mc->max_vs_mag = float2ctrl(volt_circle_pu);
    mc->max_vs_mag_sq = ctl_mul(mc->max_vs_mag, mc->max_vs_mag);
    max.dat[0] = max.dat[1] = mc->max_vs_rect;
    min.dat[0] = min.dat[1] = -mc->max_vs_rect;
    ctl_set_dq_pi_circle_limit_sq(&mc->idq_ctrl, mc->max_vs_mag_sq);
    ctl_set_dq_pi_rect_limit(&mc->idq_ctrl, &max, &min);
    ctl_set_pid_int_limit(&mc->idq_ctrl.axis[phase_d], ctl_mul(float2ctrl(0.8f), mc->max_vs_rect),
                          -ctl_mul(float2ctrl(0.8f), mc->max_vs_rect));
    ctl_set_pid_int_limit(&mc->idq_ctrl.axis[phase_q], ctl_mul(float2ctrl(0.8f), mc->max_vs_rect),
                          -ctl_mul(float2ctrl(0.8f), mc->max_vs_rect));
}

//=================================================================================================
// Consultant-based initialization and auto-tuning

void ctl_autotune_and_init_im_ifoc_consultant(im_ifoc_ctrl_t* mc, const ctl_consultant_im_t* motor,
                                              const ctl_consultant_pu_im_t* pu, parameter_gt fs,
                                              parameter_gt v_bus, parameter_gt v_phase_limit,
                                              parameter_gt current_loop_bw)
{
    parameter_gt fs_safe = (fs > 1.0f) ? fs : 10000.0f;
    parameter_gt bw = current_loop_bw;
    parameter_gt omega_bw;
    parameter_gt gain_scale;
    parameter_gt voltage_limit_pu;
    parameter_gt ff_scale;

    gmp_base_assert(mc && motor && pu);
    gmp_base_assert(pu->V_s_base > 0.0f && pu->I_s_base > 0.0f && pu->W_base > 0.0f);
    if (bw <= 0.0f) bw = fs_safe / (9.0f * CTL_PARAM_CONST_PI);
    omega_bw = CTL_PARAM_CONST_2PI * bw;
    gain_scale = pu->I_s_base / pu->V_s_base;

    ctl_init_filter_iir1_lpf(&mc->filter_iuvw[phase_U], fs_safe, fs_safe / 3.0f);
    ctl_init_filter_iir1_lpf(&mc->filter_iuvw[phase_V], fs_safe, fs_safe / 3.0f);
    ctl_init_filter_iir1_lpf(&mc->filter_iuvw[phase_W], fs_safe, fs_safe / 3.0f);
    ctl_init_filter_iir1_lpf(&mc->filter_udc, fs_safe, fs_safe / 3.0f);

    ctl_init_dq_pi(&mc->idq_ctrl,
                   motor->sigma_Ls * omega_bw * gain_scale,
                   motor->R_eq * omega_bw * gain_scale,
                   motor->sigma_Ls * omega_bw * gain_scale,
                   motor->R_eq * omega_bw * gain_scale, fs_safe);
    ctl_enable_dq_pi_feedforward(&mc->idq_ctrl);
    ctl_enable_dq_pi_circle_limit(&mc->idq_ctrl);
    ctl_enable_dq_pi_rect_limit(&mc->idq_ctrl);

    voltage_limit_pu = v_phase_limit * 1.41421356237f / pu->V_s_base;
    ctl_set_im_ifoc_saturation(mc, voltage_limit_pu, voltage_limit_pu);
    mc->max_dcbus_voltage = float2ctrl(v_bus / pu->V_s_base);

    ff_scale = pu->W_base * pu->I_s_base / pu->V_s_base;
    mc->sf_dec_lsigma = float2ctrl(motor->sigma_Ls * ff_scale);
    mc->sf_dec_backemf = float2ctrl(motor->Lm_sq_over_Lr * ff_scale);

    mc->adc_iuvw = NULL;
    mc->adc_udc = NULL;
    mc->field_pos_if = NULL;
    mc->synchronous_spd_if = NULL;
    mc->flag_enable_current_ctrl = 0;
    mc->flag_enable_decouple = 1;
    mc->flag_enable_bus_compensation = 0;
    mc->flag_enable_vdq_feedforward = 0;
    ctl_vector2_clear(&mc->idq_ref);
    ctl_vector2_clear(&mc->vdq_ref);
    ctl_clear_im_ifoc(mc);
}

//=================================================================================================
// Raw motor-parameter compatibility initializer

void ctl_autotune_and_init_im_ifoc(im_ifoc_ctrl_t* mc, const im_ifoc_init_t* init)
{
    ctl_consultant_im_t motor;
    ctl_consultant_pu_im_t pu;
    parameter_gt omega_base;
    gmp_base_assert(mc && init);
    omega_base = (init->freq_base > 0.0f)
                     ? CTL_PARAM_CONST_2PI * init->freq_base
                     : init->spd_base * 1000.0f * CTL_PARAM_CONST_PI / 30.0f * init->pole_pairs;
    ctl_consultant_im_init(&motor, (uint32_t)init->pole_pairs, init->mtr_Rs, init->mtr_Rr,
                           init->mtr_Ls, init->mtr_Lr, init->mtr_Lm);
    ctl_consultant_pu_im_init(&pu, init->v_base, init->i_base, omega_base,
                              (uint32_t)init->pole_pairs, 1.0f);
    ctl_autotune_and_init_im_ifoc_consultant(mc, &motor, &pu, init->fs, init->v_bus,
                                             init->v_phase_limit, init->current_loop_bw);
}
