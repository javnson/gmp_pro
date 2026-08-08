/**
 * @file ctl_mc_foc_core.c
 * @brief Initialization and auto-tuning for the PMSM FOC current loop.
 */

#include <gmp_core.h>
#include <ctl/component/motor_control/current_loop/foc_core.h>

static void ctl_auto_tuning_foc_core_common(mc_foc_init_t* init)
{
    parameter_gt tau;
    parameter_gt control_delay;
    parameter_gt filter_delay;
    ctl_filter_IIR1_t temp_filter;

    gmp_base_assert(init);
    gmp_base_assert(init->fs > 0.0f);

    init->current_adc_fc = init->fs / 3.0f;
    init->voltage_adc_fc = init->fs / 3.0f;
    tau = 1.5f / init->fs;
    init->current_loop_bw = 1.0f / (3.0f * tau * CTL_PARAM_CONST_2PI);

    control_delay = CTL_PARAM_CONST_2PI * init->current_loop_bw * tau;
    ctl_init_filter_iir1_lpf(&temp_filter, init->fs, init->current_adc_fc);
    filter_delay = ctl_get_filter_iir1_phase_lag(&temp_filter, init->fs, init->current_loop_bw);
    init->current_phase_lag = control_delay + filter_delay;
}

void ctl_auto_tuning_foc_core_pi(mc_foc_init_t* init)
{
    parameter_gt omega_bw;
    parameter_gt gain_scale;

    ctl_auto_tuning_foc_core_common(init);
    gmp_base_assert(init->v_base > 0.0f);
    gmp_base_assert(init->mtr_Ld > 0.0f);
    gmp_base_assert(init->mtr_Lq > 0.0f);

    omega_bw = init->current_loop_bw * CTL_PARAM_CONST_2PI;
    gain_scale = init->i_base / init->v_base;

    /* Parallel PI gains in the per-unit current/voltage system. */
    init->kpd = init->mtr_Ld * omega_bw * gain_scale;
    init->kid = init->mtr_Rs * omega_bw * gain_scale;
    init->kpq = init->mtr_Lq * omega_bw * gain_scale;
    init->kiq = init->mtr_Rs * omega_bw * gain_scale;
}

void ctl_auto_tuning_foc_core_ladrc1(mc_foc_init_t* init)
{
    ctl_auto_tuning_foc_core_common(init);
    gmp_base_assert(init->i_base > 0.0f);
    gmp_base_assert(init->mtr_Ld > 0.0f);
    gmp_base_assert(init->mtr_Lq > 0.0f);

    /* di_pu/dt = Vbase/(Ibase*L) * u_pu + total disturbance. */
    init->ladrc_b0d = init->v_base / (init->i_base * init->mtr_Ld);
    init->ladrc_b0q = init->v_base / (init->i_base * init->mtr_Lq);
    init->ladrc_fcd = init->current_loop_bw;
    init->ladrc_fcq = init->current_loop_bw;
    init->ladrc_fod = 2.0f * init->current_loop_bw;
    init->ladrc_foq = 2.0f * init->current_loop_bw;
}

void ctl_auto_tuning_foc_core(mc_foc_init_t* init)
{
#ifndef ENABLE_FOC_LADRC_CTRL
    ctl_auto_tuning_foc_core_pi(init);
#else
    ctl_auto_tuning_foc_core_ladrc1(init);
#endif
}

static void ctl_init_foc_core_common(mc_foc_core_t* mc, const mc_foc_init_t* init)
{
    int i;
    parameter_gt omega_base_elec;
    parameter_gt scale_fac;

    gmp_base_assert(mc);
    gmp_base_assert(init);

    for (i = 0; i < 3; ++i)
        ctl_init_filter_iir1_lpf(&mc->filter_iuvw[i], init->fs, init->current_adc_fc);
    ctl_init_filter_iir1_lpf(&mc->filter_udc, init->fs, init->voltage_adc_fc);

    ctl_init_lead_form3(&mc->lead_compensator[phase_d], init->current_phase_lag, init->current_loop_bw, init->fs);
    ctl_init_lead_form3(&mc->lead_compensator[phase_q], init->current_phase_lag, init->current_loop_bw, init->fs);

    omega_base_elec = (init->spd_base * 1000.0f) * CTL_PARAM_CONST_PI / 30.0f * init->pole_pairs;
    scale_fac = omega_base_elec * init->i_base / init->v_base;
    mc->coef_ff_decouple[phase_d] = float2ctrl(init->mtr_Lq * scale_fac);
    mc->coef_ff_decouple[phase_q] = float2ctrl(init->mtr_Ld * scale_fac);

    mc->max_vs_mag = float2ctrl((init->v_phase_limit * 1.41421356237f) / init->v_base);
    mc->max_vs_mag_sq = ctl_mul(mc->max_vs_mag, mc->max_vs_mag);
    mc->max_vs_rect = mc->max_vs_mag;
    mc->max_dcbus_voltage = float2ctrl(init->v_bus / init->v_base);

    mc->flag_enable_current_ctrl = 0;
    mc->flag_enable_theta_calc = 1;
    mc->flag_enable_lead_compensator = 0;
    mc->flag_enable_decouple = 0;
    mc->flag_enable_bus_compensation = 0;
    mc->flag_enable_vdq_feedforward = 0;
}

#ifndef ENABLE_FOC_LADRC_CTRL
void ctl_init_foc_core_pi(mc_foc_core_t* mc, const mc_foc_init_t* init)
{
    ctl_vector2_t limit_max;
    ctl_vector2_t limit_min;

    ctl_init_foc_core_common(mc, init);
    ctl_init_dq_pi(&mc->idq_ctrl, init->kpd, init->kid, init->kpq, init->kiq, init->fs);

    limit_max.dat[0] = limit_max.dat[1] = mc->max_vs_rect;
    limit_min.dat[0] = limit_min.dat[1] = -mc->max_vs_rect;
    ctl_set_dq_pi_circle_limit_sq(&mc->idq_ctrl, mc->max_vs_mag_sq);
    ctl_set_dq_pi_rect_limit(&mc->idq_ctrl, &limit_max, &limit_min);
    ctl_enable_dq_pi_feedforward(&mc->idq_ctrl);
    ctl_enable_dq_pi_circle_limit(&mc->idq_ctrl);
    ctl_enable_dq_pi_rect_limit(&mc->idq_ctrl);
    ctl_set_foc_core_saturation(mc, ctrl2float(mc->max_vs_rect), ctrl2float(mc->max_vs_mag));
    ctl_clear_foc_core(mc);
}
#else
void ctl_init_foc_core_ladrc1(mc_foc_core_t* mc, const mc_foc_init_t* init)
{
    ctl_vector2_t limit_max;
    ctl_vector2_t limit_min;

    ctl_init_foc_core_common(mc, init);
    ctl_init_dq_ladrc1(&mc->idq_ctrl, init->ladrc_b0d, init->ladrc_fcd, init->ladrc_fod,
                       init->ladrc_b0q, init->ladrc_fcq, init->ladrc_foq, init->fs);

    limit_max.dat[0] = limit_max.dat[1] = mc->max_vs_rect;
    limit_min.dat[0] = limit_min.dat[1] = -mc->max_vs_rect;
    ctl_set_dq_ladrc1_circle_limit_sq(&mc->idq_ctrl, mc->max_vs_mag_sq);
    ctl_set_dq_ladrc1_rect_limit(&mc->idq_ctrl, &limit_max, &limit_min);
    ctl_enable_dq_ladrc1_feedforward(&mc->idq_ctrl);
    ctl_enable_dq_ladrc1_circle_limit(&mc->idq_ctrl);
    ctl_enable_dq_ladrc1_rect_limit(&mc->idq_ctrl);
    ctl_set_foc_core_saturation(mc, ctrl2float(mc->max_vs_rect), ctrl2float(mc->max_vs_mag));
    ctl_clear_foc_core(mc);
}
#endif

void ctl_init_foc_core(mc_foc_core_t* mc, mc_foc_init_t* init)
{
#ifndef ENABLE_FOC_LADRC_CTRL
    ctl_init_foc_core_pi(mc, init);
#else
    ctl_init_foc_core_ladrc1(mc, init);
#endif
}

void ctl_init_foc_core_basic(mc_foc_core_t* mc, parameter_gt kp, parameter_gt ki,
                             parameter_gt max_vs_pu, parameter_gt fs)
{
    mc_foc_init_t init = {0};
    init.fs = fs;
    init.v_bus = 1.0f;
    init.v_phase_limit = max_vs_pu;
    init.v_base = 1.41421356237f;
    init.i_base = 1.0f;
    init.current_adc_fc = fs / 3.0f;
    init.voltage_adc_fc = fs / 3.0f;
    init.current_loop_bw = fs / (9.0f * CTL_PARAM_CONST_PI);
    init.kpd = init.kpq = kp;
    init.kid = init.kiq = ki;
#ifdef ENABLE_FOC_LADRC_CTRL
    /* In LADRC mode kp=b0 and ki=fc; observer bandwidth defaults to 2*fc. */
    init.ladrc_b0d = init.ladrc_b0q = kp;
    init.ladrc_fcd = init.ladrc_fcq = ki;
    init.ladrc_fod = init.ladrc_foq = 2.0f * ki;
    ctl_init_foc_core_ladrc1(mc, &init);
#else
    ctl_init_foc_core_pi(mc, &init);
#endif
}

void ctl_set_foc_core_saturation(mc_foc_core_t* mc, parameter_gt volt_rect_saturation,
                                 parameter_gt volt_cir_saturation)
{
    ctl_vector2_t limit_max;
    ctl_vector2_t limit_min;

    gmp_base_assert(volt_rect_saturation >= 0.0f);
    gmp_base_assert(volt_cir_saturation >= 0.0f);
    mc->max_vs_rect = float2ctrl(volt_rect_saturation);
    mc->max_vs_mag = float2ctrl(volt_cir_saturation);
    mc->max_vs_mag_sq = ctl_mul(mc->max_vs_mag, mc->max_vs_mag);
    limit_max.dat[0] = limit_max.dat[1] = mc->max_vs_rect;
    limit_min.dat[0] = limit_min.dat[1] = -mc->max_vs_rect;

#ifndef ENABLE_FOC_LADRC_CTRL
    ctl_set_dq_pi_circle_limit_sq(&mc->idq_ctrl, mc->max_vs_mag_sq);
    ctl_set_dq_pi_rect_limit(&mc->idq_ctrl, &limit_max, &limit_min);
    ctl_set_pid_int_limit(&mc->idq_ctrl.axis[phase_d], ctl_mul(float2ctrl(0.8f), mc->max_vs_rect),
                          -ctl_mul(float2ctrl(0.8f), mc->max_vs_rect));
    ctl_set_pid_int_limit(&mc->idq_ctrl.axis[phase_q], ctl_mul(float2ctrl(0.8f), mc->max_vs_rect),
                          -ctl_mul(float2ctrl(0.8f), mc->max_vs_rect));
#else
    ctl_set_dq_ladrc1_circle_limit_sq(&mc->idq_ctrl, mc->max_vs_mag_sq);
    ctl_set_dq_ladrc1_rect_limit(&mc->idq_ctrl, &limit_max, &limit_min);
#endif
}
