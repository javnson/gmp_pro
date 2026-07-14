/**
 * @file ctl_main.c
 * @author GMP Library Contributors
 * @brief Top-level implementation for the Four-Switch Buck-Boost (FSBB) Converter.
 * @details Implements a dual-loop PU controller with smooth mode transitions.
 */

#include <gmp_core.h>

#include "ctl_main.h"

//=================================================================================================
// global controller variables

// System framework
cia402_sm_t cia402_sm;
//ctl_dcdc_protect_t protection;

// Control Law Core
ctl_dcdc_core_t dcdc_core;

// Input channel
adc_channel_t adc_v_in;
adc_channel_t adc_v_out;
adc_channel_t adc_i_L;
adc_channel_t adc_i_load;

// Output channel
fsbb_modulator_t fsbb_mod;

// ADC Calibrator
adc_bias_calibrator_t adc_calibrator;
volatile fast_gt flag_enable_adc_calibrator = 0;
volatile fast_gt index_adc_calibrator = 0;

// User commands
ctrl_gt g_v_out_ref_user = float2ctrl(0.0f);
ctrl_gt g_i_limit_user = float2ctrl(0.5f);
ctrl_gt v_req;

//// 标志位
//volatile fast_gt flag_system_running = 0;
//volatile fast_gt flag_error = 0;

//=================================================================================================
// CTL initialize routine

void ctl_init(void)
{
    //
    // stop here and wait for user start the motor controller
    //
    ctl_fast_disable_output();

    //
    // FSBB controller init objects
    //
    ctl_4switch_buckboost_hardware_t fsbb_init = {0};

    fsbb_init.C_farad = FSBB_PARAM_COUT;

    fsbb_init.fs = CONTROLLER_FREQUENCY;

    fsbb_init.v_in_min = FSBB_INPUT_VOLTAGE_MIN;
    fsbb_init.v_in_max = FSBB_INPUT_VOLTAGE_MAX;

    fsbb_init.v_out_max = FSBB_OUTPUT_VOLTAGE_MAX;
    fsbb_init.v_out_min = FSBB_OUTPUT_VOLTAGE_MIN;

    fsbb_init.L_henry = FSBB_PARAM_L;
    fsbb_init.R_esr_ohm = FSBB_PARAM_L_ESR;

    fsbb_init.C_farad = FSBB_PARAM_COUT;
    fsbb_init.R_load_min = FSBB_PARAM_RLOAD_MIN;

    fsbb_init.v_base = CTRL_VOLTAGE_BASE;
    fsbb_init.i_base = CTRL_CURRENT_BASE;

    fsbb_init.slope_v_pu_s = float2ctrl(1.0f);
    fsbb_init.slope_i_pu_s = float2ctrl(1.0f);

    fsbb_init.i_out_max = float2ctrl(1.0f);
    fsbb_init.i_out_min = -float2ctrl(1.0f);

    fsbb_init.fc_current_loop = 800.0f;
    fsbb_init.fc_voltage_loop = 40.0f;

    ctl_dcdc_core_init_t core_init = {0};
    ctl_dcdc_blueprint_fsbb_cascade(&core_init, &fsbb_init);

    // init FSBB controller core
    ctl_init_dcdc_core(&dcdc_core, &core_init);

    // attach FSBB with ADC peripheral
    ctl_attach_dcdc_core(&dcdc_core, &adc_v_in.control_port, &adc_v_out.control_port, &adc_i_L.control_port,
                         &adc_i_load.control_port);

    // --- 2.2 调制器初始化 (带过渡区) ---

    v_req = float2ctrl(0.6f);

    // 配置占空比限制 [0.05, 0.95] 保证自举电容充电
    // 过渡区设置在 Vin 的 90% 到 110% 之间

    //
    // init FSBB PWM modulator
    //
    ctl_init_fsbb_modulator(&fsbb_mod, CTRL_PWM_CMP_MAX, float2ctrl(0.95f), float2ctrl(0.05f), float2ctrl(0.90f),
                            float2ctrl(1.10f));

    //
    // init and config CiA402 standard state machine
    //
    init_cia402_state_machine(&cia402_sm);
    cia402_sm.minimum_transit_delay[3] = 100; // 稳定运行 100ms 后才正式使能指令

    //
    // init and config Protection module
    //
    //    ctl_dcdc_prot_init_t prot_init = {0};
    //    prot_init.v_out_max = CTRL_PROT_VOUT_MAX;
    //    prot_init.i_L_max = CTRL_PROT_IL_MAX;
    //    ctl_init_dcdc_protect(&protection, &prot_init);

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
}

//=================================================================================================
// CTL endless loop routine

void ctl_mainloop(void)
{
    cia402_dispatch(&cia402_sm);
    return;
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

    //    if (protection.active_errors != 0)
    //    {
    //        cia402_fault_request(&cia402_sm);
    //    }
    return GMP_TASK_DONE;
}

//=================================================================================================
// CiA402 default callback routine

//
// ADC Auto calibrate routine
//
fast_gt ctl_exec_adc_calibration(void)
{
    if (!flag_enable_adc_calibrator)
        return 1;

    if (ctl_is_adc_calibrator_cmpt(&adc_calibrator) && ctl_is_adc_calibrator_result_valid(&adc_calibrator))
    {
        if (index_adc_calibrator == 0)
        {
            adc_i_load.bias += ctl_get_adc_calibrator_result(&adc_calibrator);
            index_adc_calibrator++;
            ctl_clear_adc_calibrator(&adc_calibrator);
            ctl_enable_adc_calibrator(&adc_calibrator);
        }
        else if (index_adc_calibrator == 1)
        {
            adc_i_L.bias += ctl_get_adc_calibrator_result(&adc_calibrator);
            index_adc_calibrator++;
            ctl_clear_adc_calibrator(&adc_calibrator);
            ctl_enable_adc_calibrator(&adc_calibrator);
        }
        else if (index_adc_calibrator == 2)
        {
            adc_v_out.bias += ctl_get_adc_calibrator_result(&adc_calibrator);
            index_adc_calibrator++;
            ctl_clear_adc_calibrator(&adc_calibrator);
            ctl_enable_adc_calibrator(&adc_calibrator);
        }
        else if (index_adc_calibrator == 3)
        {
            adc_v_in.bias += ctl_get_adc_calibrator_result(&adc_calibrator);
            index_adc_calibrator++;
            ctl_clear_adc_calibrator(&adc_calibrator);
            ctl_enable_adc_calibrator(&adc_calibrator);
        }
        else
        {
            flag_enable_adc_calibrator = 0; // 校准结束
            clear_all_controllers();
        }
    }
    return 0;
}

void clear_all_controllers(void)
{
    ctl_clear_dcdc_core(&dcdc_core);
}

void ctl_enable_pwm(void)
{
    ctl_fast_enable_output();
}

void ctl_disable_pwm(void)
{
    ctl_fast_disable_output();
}
