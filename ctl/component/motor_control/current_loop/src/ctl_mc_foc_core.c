/**
 * @file ctl_motor_init.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Motor controller basic structure

#include <ctl/component/motor_control/current_loop/foc_core.h>

void ctl_auto_tuning_foc_core(mc_foc_init_t* init)
{
    init->current_adc_fc = init->fs / 3;
    init->voltage_adc_fc = init->fs / 3;

    // Control Delay, 3 ~ 5 is available
    parameter_gt tau = 1.5f / init->fs;
    init->current_loop_bw = 1.0f / (3.0f * tau * CTL_PARAM_CONST_2PI);

    // per unit gain: I_base / V_base
    parameter_gt kp_scale = init->i_base / init->v_base;

    parameter_gt omega_bw = init->current_loop_bw * CTL_PARAM_CONST_2PI;

    // kp = Ldq * BW
    init->kpd = init->mtr_Ld * omega_bw * kp_scale;
    init->kpq = init->mtr_Lq * omega_bw * kp_scale;

    // Set zero to system pole
    init->kid = init->mtr_Rs / init->mtr_Ld;
    init->kiq = init->mtr_Rs / init->mtr_Lq;

    // controller delay
    parameter_gt control_delay = CTL_PARAM_CONST_2PI * init->current_loop_bw * tau;

    // input filter delay
    // Create a LPF object and calculate phase lag
    ctl_filter_IIR1_t temp_filter;
    ctl_init_filter_iir1_lpf(&temp_filter, init->fs, init->current_adc_fc);
    parameter_gt filter_delay = ctl_get_filter_iir1_phase_lag(&temp_filter, init->fs, init->current_loop_bw);

    // current controller phase lag
    init->current_phase_lag = control_delay + filter_delay;
}

void ctl_init_foc_core(mc_foc_core_t* mc, mc_foc_init_t* init)
{
    int i;

    // 1. Filter Init
    for (i = 0; i < 3; ++i)
    {
        ctl_init_filter_iir1_lpf(&mc->filter_iuvw[i], init->fs, init->current_adc_fc);
    }
    ctl_init_filter_iir1_lpf(&mc->filter_udc, init->fs, init->voltage_adc_fc);

    // 2. PID Init
    ctl_init_pid(&mc->idq_ctrl[phase_d], init->kpd, init->kid, 0, init->fs);
    ctl_init_pid(&mc->idq_ctrl[phase_q], init->kpq, init->kiq, 0, init->fs);

    // 3. Lead Compensator Init
    ctl_init_lead_form3(&mc->lead_compensator[phase_d], init->current_phase_lag, init->current_loop_bw, init->fs);
    ctl_init_lead_form3(&mc->lead_compensator[phase_q], init->current_phase_lag, init->current_loop_bw, init->fs);

    // 4. Decoupling Coefficient Calculation
    // krpm, A, V
    parameter_gt omega_base_elec = (init->spd_base * 1000.0f) * CTL_PARAM_CONST_PI / 30.0f * init->pole_pairs;
    // Scale factor to convert (pu_speed * pu_current) -> pu_voltage
    parameter_gt scale_fac = omega_base_elec * init->i_base / init->v_base;

    mc->coef_ff_decouple[phase_d] = init->mtr_Lq * scale_fac;
    mc->coef_ff_decouple[phase_q] = init->mtr_Ld * scale_fac;

    // enable phasor calculate
    mc->flag_enable_theta_calc = 1;

    // 5. Voltage Limits Initialization
    // 设定最大输出电压模值 (SVPWM 内切圆半径 或 过调制半径)
    // 假设 v_base 定义为物理电压值，v_phase_limit 为物理限幅值
    mc->max_vs_mag = (init->v_phase_limit * 1.4142f) / init->v_base;

    // 设定母线电压补偿基准
    // 用于 step 函数中的: v_scale = max_dcbus_voltage / mc->udc;
    mc->max_dcbus_voltage = init->v_bus / init->v_base;

    // dq轴实现方形限幅
    ctl_set_pid_limit(&mc->idq_ctrl[phase_d], mc->max_vs_mag, -mc->max_vs_mag);
    ctl_set_pid_int_limit(&mc->idq_ctrl[phase_d], mc->max_vs_mag, -mc->max_vs_mag);

    ctl_set_pid_limit(&mc->idq_ctrl[phase_q], mc->max_vs_mag, -mc->max_vs_mag);
    ctl_set_pid_int_limit(&mc->idq_ctrl[phase_q], mc->max_vs_mag, -mc->max_vs_mag);

    // 6. Flags Initialization (Safe defaults)
    mc->flag_enable_current_ctrl = 0;     // 默认不使能
    mc->flag_enable_theta_calc = 1;       // 默认开启角度计算
    mc->flag_enable_lead_compensator = 0; // 默认关闭超前补偿(需谨慎开启)
    mc->flag_enable_decouple = 0;         // 默认关闭解耦
    mc->flag_enable_bus_compensation = 0; // 默认关闭母线补偿
    mc->flag_enable_vdq_feedforward = 0;  // 默认关闭前馈

    // 7. Clear all states
    ctl_clear_foc_core(mc);
}
