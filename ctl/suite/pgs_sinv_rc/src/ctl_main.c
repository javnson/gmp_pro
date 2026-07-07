/**
 * @file ctl_main.c
 * @author GMP Library Contributors
 * @brief 20kW Single-Phase Inverter/AFE Top-level Implementation.
 */

#include <gmp_core.h>

#include "ctl_main.h"

//=================================================================================================
// global controller variables

// System framework
cia402_sm_t cia402_sm;

// Control Law Core
spll_sogi_t pll;
ctl_sms_pq_t pq_meter;
ctl_sinv_ref_gen_t ref_gen;
ctl_sinv_rc_core_t rc_core;
ctl_ramp_generator_t rg;

// FDRC controller static memory
#define FDRC_ARRAY_SIZE ((int)(CONTROLLER_FREQUENCY / 50.0f) + 10)
static ctrl_gt fdrc_buffer[FDRC_ARRAY_SIZE];

// Input channel

// Output channel
single_phase_H_modulation_t hpwm;

// Protection module
ctl_sinv_protect_t protection;

// ADC Calibrator
adc_bias_calibrator_t adc_calibrator;
volatile fast_gt flag_enable_adc_calibrator = 1;
volatile fast_gt index_adc_calibrator = 0;

// User commands
ctrl_gt g_p_ref_user = float2ctrl(0.0f);
ctrl_gt g_q_ref_user = float2ctrl(0.0f);

ctrl_gt openloop_v_ref = float2ctrl(0.5f);
vector2_gt phasor;

//=================================================================================================
// CTL initialize routine

void ctl_init(void)
{
    //
    // stop here and wait for user start the motor controller
    //
    ctl_fast_disable_output();

    //
    // SINV current controller init objects
    //
    ctl_sinv_rc_init_t rc_init = {0};

    rc_init.fs = CONTROLLER_FREQUENCY;
    rc_init.freq_grid = 50.0f;
    rc_init.v_base = CTRL_VOLTAGE_BASE;
    rc_init.i_base = CTRL_CURRENT_BASE;
    rc_init.v_bus = CTRL_DCBUS_VOLTAGE;
    rc_init.L_ac = 0.003f; // 根据实际滤波器电感修改
    rc_init.R_ac = 0.1f;   // 寄生电阻估算

    ctl_auto_tuning_sinv_rc(&rc_init);
    ctl_init_sinv_rc_core(&rc_core, &rc_init, fdrc_buffer, FDRC_ARRAY_SIZE);

    // attach SIHV RC module to ADC peripheral
    ctl_attach_sinv_rc(&rc_core, &adc_v_bus.control_port, &adc_v_grid.control_port, &adc_i_ac.control_port);

    // init PLL & PQ controller
    ctl_init_single_phase_pll(&pll, 10.0f, 0.02f, 20.0f, 50.0f, CONTROLLER_FREQUENCY);
    ctl_init_sms_pq(&pq_meter, CONTROLLER_FREQUENCY, 50.0f, 200.0f);

    // Command generator, I_max(pu), V_min(pu), P_slope(pu/s), Q_slope(pu/s)
    ctl_init_sinv_ref_gen(&ref_gen, CTRL_CURRENT_BASE * 1.5f, 0.1f, 10.0f, 20.0f, CONTROLLER_FREQUENCY);

    // freerun angle reference generator, 50Hz, [0, 1] range for pu angle
    ctl_init_ramp_generator_via_freq(&rg, CONTROLLER_FREQUENCY, 50.0f, 1, 0);

    //
    // init H PWM modulator
    //
    ctl_init_single_phase_H_modulation(&hpwm, CTRL_PWM_CMP_MAX + 1, CTRL_PWM_DEADBAND_CMP, float2ctrl(0.5f));
    //hpwm.flag_enable_dbcomp = 1; // 开启死区补偿

    //
    // init and config CiA402 standard state machine
    //
    init_cia402_state_machine(&cia402_sm);
    cia402_sm.minimum_transit_delay[3] = 100; // Operation Enabled 保持 100ms 后才正式切入负载

    //
    // init and config Protection module
    //
    ctl_sinv_prot_init_t prot_init = {0};
    prot_init.error_mask =
        SINV_PROT_BIT_HW_TZ | SINV_PROT_BIT_DC_OVP_FAST | SINV_PROT_BIT_AC_OCP_FAST | SINV_PROT_BIT_CTRL_DIVERGE;
    prot_init.warning_mask = SINV_PROT_BIT_AC_OVP_RMS | SINV_PROT_BIT_AC_UVP_RMS | SINV_PROT_BIT_PLL_FREQ_ERR;
    prot_init.v_bus_max = CTRL_PROT_VBUS_MAX;
    prot_init.i_ac_max = CTRL_MAX_HW_CURRENT * 0.9f; // 硬件极限的 90%
    ctl_init_sinv_protect(&protection, &prot_init);

    //
    // init ADC Calibrator
    //
    ctl_init_adc_calibrator(&adc_calibrator, 20, 0.707f, CONTROLLER_FREQUENCY);
    if (flag_enable_adc_calibrator)
    {
        ctl_enable_adc_calibrator(&adc_calibrator);
    }

    //
    // incremental compilation configuration
    //

#if (BUILD_LEVEL == 1)
    // Disable FDRC and feed forward
    rc_core.flag_enable_fdrc = 0;
    rc_core.flag_enable_lead_comp = 0;
#endif

#if (BUILD_LEVEL == 2)
    // 并网 PQ 控制模式：禁止 FDRC，开启电压前馈
    rc_core.flag_enable_fdrc = 0;
    rc_core.flag_enable_lead_comp = 1;
#endif
}

//=================================================================================================
// CTL endless loop routine

void ctl_mainloop(void)
{
    cia402_dispatch(&cia402_sm);

#if (BUILD_LEVEL == 3)
    // 并网性能优化模式：延时切入 FDRC
    // 当并网稳定运行超过 200 个控制周期后，开启重复控制器消除畸变
    if (cia402_sm.state_word.bits.operation_enabled && cia402_sm.current_state_counter > 200)
    {
        rc_core.flag_enable_fdrc = 1;
    }
    else
    {
        rc_core.flag_enable_fdrc = 0;
    }
    rc_core.flag_enable_lead_comp = 1;
#endif
}

void gmp_pil_sim_step(const gmp_sim_rx_buf_t* rx, gmp_sim_tx_buf_t* tx)
{
#if defined ENBALE_GMP_DL_PIL_SIM
    ctl_input_callback_pil(rx);

    ctl_dispatch();

    ctl_output_callback_pil(tx);
#endif // defined ENBALE_GMP_DL_PIL_SIM
}

#if defined ENBALE_GMP_DL_PIL_SIM
time_gt gmp_base_get_ctrl_tick(void)
{
    return mtr_ctrl.isr_tick / ((uint32_t)CONTROLLER_FREQUENCY / 1000);
}
#endif // defined ENBALE_GMP_DL_PIL_SIM

//=================================================================================================
// Controller Tasks

gmp_task_status_t tsk_protect(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    // 执行慢保护检测 (交流有效值过欠压、温度、电网频率)
    //    ctl_task_sinv_protect_slow(&protection, pq_meter.v_rms, pll.frequency,
    //                               float2ctrl(25.0f), // Mock temp: 实际应接入板载温度传感器
    //                               pq_meter.i_rms);

    if (protection.active_errors != 0)
    {
        cia402_fault_request(&cia402_sm);
    }

    return GMP_TASK_DONE;
}

//=================================================================================================
// CiA402 default callback routine

//
// Check if PLL is locked.
// return 1 if PLL is locked, 0 if PLL hasn't locked
//
fast_gt ctl_check_pll_locked(void)
{
    // 准入条件：
    // 1. 电网电压幅值在 0.8pu ~ 1.2pu 之间 (防止断路器未闭合或严重欠压)
    // 2. PLL 内部频率误差必须小于系统设定的容忍度 (例如 0.005 PU)
    ctrl_gt v_mag_pu = pll.v_mag;
    ctrl_gt f_err_abs = ctl_abs(pll.freq_error);

    if ((v_mag_pu > float2ctrl(0.8f)) && (v_mag_pu < float2ctrl(1.2f)))
    {
        if (f_err_abs < CTRL_SPLL_EPSILON)
        {
            return 1;
        }
    }
    return 0;
}

//
// ADC Auto calibrate routine
//
fast_gt ctl_exec_adc_calibration(void)
{
    if (flag_enable_adc_calibrator)
    {
        if (ctl_is_adc_calibrator_cmpt(&adc_calibrator) && ctl_is_adc_calibrator_result_valid(&adc_calibrator))
        {
            if (index_adc_calibrator == 0) // 通道 0: AC 电流零偏校准
            {
                adc_i_ac.bias += ctl_get_adc_calibrator_result(&adc_calibrator);
                index_adc_calibrator += 1;

                ctl_clear_adc_calibrator(&adc_calibrator);
                ctl_enable_adc_calibrator(&adc_calibrator);
            }
            else if (index_adc_calibrator == 1) // 通道 1: 交流电压零偏校准
            {
                adc_v_grid.bias += ctl_get_adc_calibrator_result(&adc_calibrator);
                index_adc_calibrator += 1;

                flag_enable_adc_calibrator = 0;
                clear_all_controllers(); // 校准完毕，清理滤波器内残余历史值
            }
        }

        // ADC calibrate is not complete
        return 0;
    }

    // skip calibrate routine
    return 1;
}

void clear_all_controllers(void)
{
    ctl_clear_single_phase_pll(&pll);
    ctl_clear_sinv_rc_core(&rc_core);
    ctl_clear_sinv_ref_gen(&ref_gen);
    ctl_clear_single_phase_H_modulation(&hpwm);
}

void ctl_enable_pwm(void)
{
    ctl_fast_enable_output();
}

void ctl_disable_pwm(void)
{
    ctl_fast_disable_output();
}
