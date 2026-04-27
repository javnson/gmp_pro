
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

// ACM Sensored controller
acm_sensored_bare_controller_t acm_ctrl;

#ifdef PMSM_CTRL_USING_QEP_ENCODER
// Auto - turn encoder
pos_autoturn_encoder_t pos_enc;
#endif // PMSM_CTRL_USING_QEP_ENCODER

// speed encoder
//spd_calculator_t spd_enc;

//#if defined OPENLOOP_CONST_FREQUENCY
//
//// PMSM const frequency controller
//ctl_const_f_controller const_f;
//
//#else // OPENLOOP_CONST_FREQUENCY
//
//// PMSM const frequency slope controller
//ctl_slope_f_controller slope_f;
//
//#endif // OPENLOOP_CONST_FREQUENCY

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
    ctl_filter_IIR2_setup_t adc_calibrator_filter;
    adc_calibrator_filter.filter_type = FILTER_IIR2_TYPE_LOWPASS;
    adc_calibrator_filter.fc = 20;
    adc_calibrator_filter.fs = CONTROLLER_FREQUENCY;
    adc_calibrator_filter.gain = 1;
    adc_calibrator_filter.q = 0.707f;
    ctl_init_adc_calibrator(&adc_calibrator, &adc_calibrator_filter);

#ifdef PMSM_CTRL_USING_QEP_ENCODER
    // init Auto - turn encoder
    ctl_init_autoturn_pos_encoder(&pos_enc, MOTOR_PARAM_POLE_PAIRS, MTR_ENCODER_LINES);
    // Set encoder offset
    ctl_set_autoturn_pos_encoder_offset(&pos_enc, MTR_ENCODER_OFFSET);
#endif // PMSM_CTRL_USING_QEP_ENCODER

    flag_enable_system = 0;

    // create a speed observer by position encoder
//    ctl_init_spd_calculator(
//        // attach position with speed encoder
//        &spd_enc, acm_ctrl.mtr_interface.position,
//        // set spd calculator parameters
//        CONTROLLER_FREQUENCY, 5, MOTOR_PARAM_MAX_SPEED, 1, 150);

//#if defined OPENLOOP_CONST_FREQUENCY
//    ctl_init_const_f_controller(&const_f, 20, CONTROLLER_FREQUENCY);
//#else  // OPENLOOP_CONST_FREQUENCY
//    // frequency target 20 Hz, frequency slope 40 Hz/s
//    ctl_init_const_slope_f_controller(&slope_f, 20.0f, 40.0f, CONTROLLER_FREQUENCY);
//#endif // OPENLOOP_CONST_FREQUENCY

    // attach a speed encoder object with motor controller
    //ctl_attach_mtr_velocity(&acm_ctrl.mtr_interface, &spd_enc.encif);

    ctl_attach_acm_sensored_bare_rotor_postion(&acm_ctrl, &pos_enc.encif);

    // set acm_ctrl parameters
    acm_sensored_bare_controller_init_t acm_ctrl_init;

    acm_ctrl_init.fs = CONTROLLER_FREQUENCY;

    // current pid controller parameters
//    acm_ctrl_init.current_pid_gain = (parameter_gt)((MOTOR_PARAM_L1S + MOTOR_PARAM_LM) * MTR_CTRL_CURRENT_LOOP_BW * 2 * PI *
//                                                    MTR_CTRL_VOLTAGE_BASE / MTR_CTRL_CURRENT_BASE);
//    acm_ctrl_init.current_Ti = (parameter_gt)((MOTOR_PARAM_L1S + MOTOR_PARAM_LM) / MOTOR_PARAM_RS);

    acm_ctrl_init.current_pid_gain = (parameter_gt)(0.9);
    acm_ctrl_init.current_Ti = (parameter_gt)(1.0/20);

    acm_ctrl_init.current_Td = 0;
    acm_ctrl_init.voltage_limit_min = float2ctrl(-0.3);
    acm_ctrl_init.voltage_limit_max = float2ctrl(0.3);

    // speed pid controller parameters
    acm_ctrl_init.spd_ctrl_div = SPD_CONTROLLER_PWM_DIVISION;
    acm_ctrl_init.spd_pid_gain = (parameter_gt)(3.0);
    acm_ctrl_init.spd_Ti = (parameter_gt)(4.0f / MTR_CTRL_SPEED_LOOP_BW);
    acm_ctrl_init.spd_Td = 0;
    acm_ctrl_init.current_limit_min = float2ctrl(-0.45);
    acm_ctrl_init.current_limit_max = float2ctrl(0.45);

    // accelerator parameters
    acm_ctrl_init.acc_limit_min = -150.0f;
    acm_ctrl_init.acc_limit_max = 150.0f;

    // ACM flux speed calculator
    acm_ctrl_init.Lr = MOTOR_PARAM_LM + MOTOR_PARAM_L1R;
    acm_ctrl_init.Rr = MOTOR_PARAM_RR;
    acm_ctrl_init.pole_pairs = MOTOR_PARAM_POLE_PAIRS;
    acm_ctrl_init.base_freq = MOTOR_PARAM_RATED_FREQUENCY;
    acm_ctrl_init.base_spd = MOTOR_PARAM_MAX_SPEED;

    // startup speed acc
//    acm_ctrl_init.target_freq = 5.0;
    acm_ctrl_init.target_freq_slope = 2.5;

    // init the PMSM controller
    ctl_init_acm_sensored_bare_controller(&acm_ctrl, &acm_ctrl_init);

    // BUG TI cannot print out sizeof() result if no type is specified.
    gmp_base_print(TEXT_STRING("PMSM SERVO struct has been inited, size :%d\r\n"), (int)sizeof(acm_ctrl_init));

#if (BUILD_LEVEL == 1)
//#if defined OPENLOOP_CONST_FREQUENCY
//    ctl_attach_mtr_position(&acm_ctrl.mtr_interface, &const_f.enc);
//#else  // OPENLOOP_CONST_FREQUENCY
//    ctl_attach_mtr_position(&acm_ctrl.mtr_interface, &slope_f.enc);
//#endif // OPENLOOP_CONST_FREQUENCY

    ctl_acm_sensored_ctrl_voltage_mode(&acm_ctrl);
    ctl_set_acm_sensored_ctrl_vdq_ff(&acm_ctrl, float2ctrl(0.05), float2ctrl(0.05));

    ctl_set_acm_sensored_ctrl_speed(&acm_ctrl, float2ctrl(0.2));

#elif (BUILD_LEVEL == 2)
//#if defined OPENLOOP_CONST_FREQUENCY
//    ctl_attach_mtr_position(&acm_ctrl.mtr_interface, &const_f.enc);
//#else  // OPENLOOP_CONST_FREQUENCY
//    ctl_attach_mtr_position(&acm_ctrl.mtr_interface, &slope_f.enc);
//#endif // OPENLOOP_CONST_FREQUENCY
    ctl_acm_sensored_ctrl_current_mode(&acm_ctrl);
    ctl_set_acm_sensored_ctrl_idq_ff(&acm_ctrl, float2ctrl(0.05), float2ctrl(0.2));

    ctl_set_acm_sensored_ctrl_speed(&acm_ctrl, float2ctrl(0.2));

#elif (BUILD_LEVEL == 3)

    ctl_acm_sensored_ctrl_current_mode(&acm_ctrl);
    ctl_set_acm_sensored_ctrl_idq_ff(&acm_ctrl, float2ctrl(0.2), float2ctrl(0.2));

    ctl_set_acm_sensored_ctrl_speed(&acm_ctrl, float2ctrl(0.3));

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
    ctl_enable_acm_sensored_ctrl(&acm_ctrl);

#if defined SPECIFY_ENABLE_ADC_CALIBRATE
    // Enable ADC calibrate
    flag_enable_adc_calibrator = 1;
    index_adc_calibrator = 0;

    // Select ADC calibrate
    ctl_disable_acm_sensored_ctrl_output(&acm_ctrl);
    ctl_enable_adc_calibrator(&adc_calibrator);
#endif // SPECIFY_ENABLE_ADC_CALIBRATE
}

//////////////////////////////////////////////////////////////////////////
// endless loop function here

uint16_t sgen_out = 0;

void ctl_mainloop(void)
{
    // Get current System Tick
    time_gt current_tick = gmp_base_get_ctrl_tick();

    //
    // Judge if PWM is enabled
    //
    if(acm_ctrl.flag_enable_output)
    {
        ctl_enable_output();
    }
    else
    {
        ctl_disable_output();
    }

    //
    // ADC Auto calibrate
    //
    if (flag_enable_adc_calibrator)
    {
        if (ctl_is_adc_calibrator_cmpt(&adc_calibrator) && ctl_is_adc_calibrator_result_valid(&adc_calibrator))
        {
            // set_adc_bias_via_channel(index_adc_calibrator, ctl_get_adc_calibrator_result(&adc_calibrator));

            if (index_adc_calibrator == 3) // dc bus calibrate
            {
                idc.bias = idc.bias + ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), idc.gain);

                flag_enable_adc_calibrator = 0;

                // enable pmsm controller
                ctl_enable_acm_sensored_ctrl_output(&acm_ctrl);
            }
            // index_adc_calibrator == 2 ~ 0, for Iabc
            else
            {
                // iabc get result
                iabc.bias[index_adc_calibrator] =
                    iabc.bias[index_adc_calibrator] +
                    ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), iabc.gain[index_adc_calibrator]);

                // move to next position
                index_adc_calibrator += 1;

                // clear calibrator
                ctl_clear_adc_calibrator(&adc_calibrator);

                // enable calibrator to next position
                ctl_enable_adc_calibrator(&adc_calibrator);
            }

            if (index_adc_calibrator > MTR_ADC_IDC)
                flag_enable_adc_calibrator = 0;
        }
    }

    //
    // Speed Loop switch logic
    // Current Loop -> Speed Loop
    //

    if(acm_ctrl.speed_set - ctl_get_mtr_velocity(&acm_ctrl.mtr_interface) < ctl_mul(float2ctrl(0.1),acm_ctrl.speed_set) )
    {
        ctl_acm_sensored_ctrl_velocity_mode(&acm_ctrl);
    }


    //
    // Add User Control Process Logic here
    //

    return;
}

