
//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should define your own controller objects,
// and initilize them.
//
// User should implement a ctl loop function, this
// function would be called every main loop.
//
// User should implement a state machine if you are using
// Controller Nanon framework.
//

#include <gmp_core.h>

#include <ctrl_settings.h>

#include "ctl_main.h"

#include <xplt.peripheral.h>

// PMSM controller
pmsm_smo_controller_t pmsm_ctrl;

//
adc_bias_calibrator_t adc_calibrator;
fast_gt flag_enable_adc_calibrator = 0;
fast_gt index_adc_calibrator = 0;

// enable motor running
volatile fast_gt flag_enable_system = 0;

// CTL initialize routine
void ctl_init()
{
    // setup ADC calibrate
//    ctl_filter_IIR2_setup_t adc_calibrator_filter;
//    adc_calibrator_filter.filter_type = FILTER_IIR2_TYPE_LOWPASS;
//    adc_calibrator_filter.fc = 20;
//    adc_calibrator_filter.fs = CONTROLLER_FREQUENCY;
//    adc_calibrator_filter.gain = 1;
//    adc_calibrator_filter.q = 0.707f;
    // ctl_init_adc_bias_calibrator(&adc_calibrator, &adc_calibrator_filter);

    flag_enable_system = 0;

    // set pmsm_ctrl parameters
    pmsm_smo_controller_init_t pmsm_ctrl_init;

    pmsm_ctrl_init.fs = CONTROLLER_FREQUENCY;

    // current pid controller parameters
    pmsm_ctrl_init.current_pid_gain = (parameter_gt)(MOTOR_PARAM_LS * MTR_CTRL_CURRENT_LOOP_BW * 2 * CTL_PARAM_CONST_PI *
                                                     MTR_CTRL_VOLTAGE_BASE / MTR_CTRL_CURRENT_BASE);
    pmsm_ctrl_init.current_Ti = (parameter_gt)(MOTOR_PARAM_LS / MOTOR_PARAM_RS);
    pmsm_ctrl_init.current_Td = 0;
    pmsm_ctrl_init.voltage_limit_min = float2ctrl(-1.0);
    pmsm_ctrl_init.voltage_limit_max = float2ctrl(1.0);

    // speed pid controller parameters
    pmsm_ctrl_init.spd_ctrl_div = SPD_CONTROLLER_PWM_DIVISION;

    pmsm_ctrl_init.spd_pid_gain = 0.04f;
    pmsm_ctrl_init.spd_Ti = 1.0f / 1000;

    // pmsm_ctrl_init.spd_pid_gain = (parameter_gt)(3.5);
    // pmsm_ctrl_init.spd_Ti = (parameter_gt)(4.0f / MTR_CTRL_SPEED_LOOP_BW);

    pmsm_ctrl_init.spd_Td = 0;
    pmsm_ctrl_init.current_limit_min = float2ctrl(-0.45);
    pmsm_ctrl_init.current_limit_max = float2ctrl(0.45);

    // accelerator parameters
    pmsm_ctrl_init.acc_limit_min = -150.0f;
    pmsm_ctrl_init.acc_limit_max = 150.0f;

    // Motor parameters
    pmsm_ctrl_init.Ld = (parameter_gt)(MOTOR_PARAM_LS);
    pmsm_ctrl_init.Lq = (parameter_gt)(MOTOR_PARAM_LS);
    pmsm_ctrl_init.Rs = (parameter_gt)(MOTOR_PARAM_RS);
    pmsm_ctrl_init.pole_pairs = MOTOR_PARAM_POLE_PAIRS;
    pmsm_ctrl_init.u_base = (parameter_gt)(MTR_CTRL_VOLTAGE_BASE);
    pmsm_ctrl_init.i_base = (parameter_gt)(MTR_CTRL_CURRENT_BASE);

    // SMO controller parameters
    pmsm_ctrl_init.speed_base_rpm = (parameter_gt)(MOTOR_PARAM_MAX_SPEED);
    pmsm_ctrl_init.smo_fc_e = 30.0;
    pmsm_ctrl_init.smo_fc_omega = 50.0;

    pmsm_ctrl_init.smo_k_slide = float2ctrl(1);
    pmsm_ctrl_init.smo_kp = float2ctrl(4);
    pmsm_ctrl_init.smo_Ti = float2ctrl(0.0075);
    pmsm_ctrl_init.smo_Td = 0;

    // VF/IF slope controller
    pmsm_ctrl_init.ramp_target_freq = 100.0f;
    pmsm_ctrl_init.ramp_target_freq_slope = 500.0f;

    // init the PMSM controller
    ctl_init_pmsm_smo_bare_controller(&pmsm_ctrl, &pmsm_ctrl_init);

    // BUG TI cannot print out sizeof() result if no type is specified.
    gmp_base_print(TEXT_STRING("PMSM SERVO struct has been inited, size :%d\r\n"), (int)sizeof(pmsm_ctrl_init));

#if (BUILD_LEVEL == 1)

    ctl_pmsm_smo_ctrl_voltage_mode(&pmsm_ctrl);
    ctl_set_pmsm_smo_ctrl_vdq_ff(&pmsm_ctrl, float2ctrl(0.2), float2ctrl(0.2));

#elif (BUILD_LEVEL == 2)

    ctl_pmsm_smo_ctrl_current_mode(&pmsm_ctrl);
    ctl_set_pmsm_smo_ctrl_idq_ff(&pmsm_ctrl, float2ctrl(0), float2ctrl(0.01));

#elif (BUILD_LEVEL == 3)

    ctl_pmsm_smo_ctrl_current_mode(&pmsm_ctrl);
    ctl_set_pmsm_smo_ctrl_idq_ff(&pmsm_ctrl, float2ctrl(0.0), float2ctrl(0.01));

    ctl_enable_pmsm_smo(&pmsm_ctrl);

#endif // BUILD_LEVEL

    // if in simulation mode, enable system
#if !defined SPECIFY_PC_ENVIRONMENT
    // stop here and wait for user start the motor controller
    while (flag_enable_system == 0)
    {
    }
#endif // SPECIFY_PC_ENVIRONMENT

    ctl_enable_output();

    // Debug mode online the controller
    ctl_enable_pmsm_smo_ctrl(&pmsm_ctrl);
}

//////////////////////////////////////////////////////////////////////////
// endless loop function here

uint16_t sgen_out = 0;

void ctl_mainloop(void)
{
    int spd_target = gmp_base_get_system_tick() / 100;

    // ctl_set_pmsm_smo_ctrl_speed(&pmsm_ctrl, float2ctrl(0.1) * spd_target - float2ctrl(1.0));

#if (BUILD_LEVEL == 3)
    // if build level == 3 switch to SMO controller
    if (gmp_base_get_system_tick() >= 600)
    {
        ctl_switch_pmsm_smo_ctrl_using_smo(&pmsm_ctrl);
    }
#endif // BUILD_LEVEL

    if (gmp_base_get_system_tick() >= 1000)
    {
        ctl_set_pmsm_smo_ctrl_speed(&pmsm_ctrl, float2ctrl(0.3));
    }

    if (gmp_base_get_system_tick() >= 1400)
    {
        ctl_set_pmsm_smo_ctrl_speed(&pmsm_ctrl, float2ctrl(0.8));
    }

    //
    if (flag_enable_adc_calibrator)
    {
        if (ctl_is_adc_calibrator_cmpt(&adc_calibrator))
        {
            // set_adc_bias_via_channel(index_adc_calibrator, ctl_get_adc_calibrator_result(&adc_calibrator));
            index_adc_calibrator += 1;
            if (index_adc_calibrator > MTR_ADC_IDC)
                flag_enable_adc_calibrator = 0;
        }
    }
    return;
}

#ifdef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

void ctl_fmif_monitor_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
}

// return value:
// 1 change to next progress
// 0 keep the same state
fast_gt ctl_fmif_sm_pending_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 0;
}

// return value:
// 1 change to next progress
// 0 keep the same state
fast_gt ctl_fmif_sm_calibrate_routine(ctl_object_nano_t *pctl_obj)
{
    return ctl_cb_pmsm_servo_frmework_current_calibrate(&pmsm_servo);
}

fast_gt ctl_fmif_sm_ready_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 0;
}

// Main relay close, power on the main circuit
fast_gt ctl_fmif_sm_runup_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 1;
}

fast_gt ctl_fmif_sm_online_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 0;
}

fast_gt ctl_fmif_sm_fault_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 0;
}

#endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
