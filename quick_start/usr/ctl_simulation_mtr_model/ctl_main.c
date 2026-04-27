
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
pmsm_bare_controller_t pmsm_ctrl;

// speed encoder
spd_calculator_t spd_enc;

// PMSM const frequency controller
ctl_const_f_controller const_f;

//
adc_bias_calibrator_t adc_calibrator;
fast_gt flag_enable_adc_calibrator = 1;
fast_gt index_adc_calibrator = 0;

// enable motor running
fast_gt falg_enable_system = 0;

// CTL initialize routine
void ctl_init()
{
    // setup ADC calibrate
    ctl_filter_IIR2_setup_t adc_calibrator_filter;
    adc_calibrator_filter.filter_type = FILTER_IIR2_TYPE_LOWPASS;
    adc_calibrator_filter.fc = 20;
    adc_calibrator_filter.fs = CONTROLLER_FREQUENCY;
    adc_calibrator_filter.gain = 1;
    adc_calibrator_filter.q = 0.707f;
    // ctl_init_adc_bias_calibrator(&adc_calibrator, &adc_calibrator_filter);

    falg_enable_system = 0;

    // create a speed observer by position encoder
    ctl_init_spd_calculator(
        // attach position with speed encoder
        &spd_enc, pmsm_ctrl.mtr_interface.position,
        // set spd calculator parameters
        CONTROLLER_FREQUENCY, 5, MOTOR_PARAM_MAX_SPEED, 1, 150);

    ctl_setup_const_f_controller(&const_f, 20, CONTROLLER_FREQUENCY);

    // attach a speed encoder object with motor controller
    ctl_attach_mtr_velocity(&pmsm_ctrl.mtr_interface, &spd_enc.encif);

    // set pmsm_ctrl parameters
    pmsm_bare_controller_init_t pmsm_ctrl_init;

    pmsm_ctrl_init.fs = CONTROLLER_FREQUENCY;

    // current pid controller parameters
    pmsm_ctrl_init.current_pid_gain = 2.15f;
    // pmsm_ctrl_init.current_Ti = 1.0f / 500;
    pmsm_ctrl_init.current_Ti = 1.0f / 500;
    pmsm_ctrl_init.current_Td = 0;
    pmsm_ctrl_init.voltage_limit_min = float2ctrl(-0.45);
    pmsm_ctrl_init.voltage_limit_max = float2ctrl(0.45);

    // speed pid controller parameters
    pmsm_ctrl_init.spd_ctrl_div = SPD_CONTROLLER_PWM_DIVISION;
    pmsm_ctrl_init.spd_pid_gain = 3.5f;
    pmsm_ctrl_init.spd_Ti = 1.0f / 100;
    pmsm_ctrl_init.spd_Td = 0;
    pmsm_ctrl_init.current_limit_min = float2ctrl(-0.6);
    pmsm_ctrl_init.current_limit_max = float2ctrl(0.6);

    // accelerator parameters
    pmsm_ctrl_init.acc_limit_min = -150.0f;
    pmsm_ctrl_init.acc_limit_max = 150.0f;

    // init the PMSM controller
    ctl_init_pmsm_bare_controller(&pmsm_ctrl, &pmsm_ctrl_init);

    // BUG TI cannot print out sizeof() result if no type is specified.
    gmp_base_print(TEXT_STRING("PMSM SERVO struct has been inited, size :%d\r\n"), (int)sizeof(pmsm_ctrl_init));

#if (BUILD_LEVEL == 1)
    ctl_attach_mtr_position(&pmsm_ctrl.mtr_interface, &const_f.enc);
    ctl_pmsm_ctrl_voltage_mode(&pmsm_ctrl);
    ctl_set_pmsm_ctrl_vdq_ff(&pmsm_ctrl, float2ctrl(0.2), float2ctrl(0.2));

#elif (BUILD_LEVEL == 2)
    ctl_attach_mtr_position(&pmsm_ctrl.mtr_interface, &const_f.enc);
    ctl_pmsm_ctrl_current_mode(&pmsm_ctrl);
    ctl_set_pmsm_ctrl_idq_ff(&pmsm_ctrl, float2ctrl(0.3), float2ctrl(0.1));

#elif (BUILD_LEVEL == 3)

    ctl_pmsm_ctrl_current_mode(&pmsm_ctrl);
    ctl_set_pmsm_ctrl_idq_ff(&pmsm_ctrl, float2ctrl(0.1), float2ctrl(0.05));

#elif (BUILD_LEVEL == 4)

    ctl_pmsm_ctrl_velocity_mode(&pmsm_ctrl);
    ctl_set_pmsm_ctrl_speed(&pmsm_ctrl, float2ctrl(0.25));
#endif // BUILD_LEVEL

    // Debug mode online the controller
    ctl_enable_pmsm_ctrl(&pmsm_ctrl);

    // stop here and wait for user start the motor controller
    while (falg_enable_system == 0)
    {
    }

    ctl_enable_output();
}

//////////////////////////////////////////////////////////////////////////
// endless loop function here

void ctl_mainloop(void)
{
    int spd_target = gmp_base_get_ctrl_tick() / 100;

    ctl_set_pmsm_ctrl_speed(&pmsm_ctrl, float2ctrl(0.1) * spd_target - float2ctrl(1.0));

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
