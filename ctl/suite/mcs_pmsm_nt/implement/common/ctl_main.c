
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

#include <core/pm/function_scheduler.h>

//=================================================================================================
// global controller variables

// state machine
cia402_sm_t cia402_sm;
ctl_mtr_protect_t protection;

// modulator: SPWM modulator / SVPWM modulator / NPC modulator
#if defined USING_NPC_MODULATOR
npc_modulator_t spwm;
#else
spwm_modulator_t spwm;
#endif // USING_NPC_MODULATOR

// controller body: Current controller, Command dispatcher, motion controller
mtr_current_ctrl_t mtr_ctrl;
mtr_current_init_t mtr_ctrl_init;
vel_pos_ctrl_t motion_ctrl;

// Observer: SMO, FO, Speed measurement.
ctl_slope_f_pu_controller rg;
pos_autoturn_encoder_t pos_enc;
spd_calculator_t spd_enc;

#ifdef ENABLE_SMO

ctl_smo_init_t smo_init;
pmsm_smo_t smo;

#endif // ENABLE_SMO

// additional controller: harmonic management

//
volatile fast_gt flag_system_running = 0;
volatile fast_gt flag_error = 0;

// adc calibrator flags
adc_bias_calibrator_t adc_calibrator;
volatile fast_gt flag_enable_adc_calibrator = 1;
//volatile fast_gt flag_enable_adc_calibrator = 0;
volatile fast_gt index_adc_calibrator = 0;

//=================================================================================================
// CTL initialize routine

void ctl_init()
{
    //
    // stop here and wait for user start the motor controller
    //
    ctl_fast_disable_output();

    //
    // motor current controller init objects
    //
    mtr_ctrl_init.fs = CONTROLLER_FREQUENCY;
    mtr_ctrl_init.v_base = CTRL_VOLTAGE_BASE;
    mtr_ctrl_init.i_base = CTRL_CURRENT_BASE;

    mtr_ctrl_init.v_bus = CTRL_VOLTAGE_BASE;
    mtr_ctrl_init.v_phase_limit = MOTOR_PARAM_RATED_VOLTAGE;

    mtr_ctrl_init.freq_base = MOTOR_PARAM_RATED_FREQUENCY;
    mtr_ctrl_init.spd_base = MOTOR_PARAM_MAX_SPEED / 1000;
    mtr_ctrl_init.pole_pairs = MOTOR_PARAM_POLE_PAIRS;

    mtr_ctrl_init.mtr_Ld = MOTOR_PARAM_LS;
    mtr_ctrl_init.mtr_Lq = MOTOR_PARAM_LS;
    mtr_ctrl_init.mtr_Rs = MOTOR_PARAM_RS;

    ctl_auto_tuning_mtr_current_ctrl(&mtr_ctrl_init);
    ctl_init_mtr_current_ctrl(&mtr_ctrl, &mtr_ctrl_init);

    //
    // init SPWM modulator
    //
#if defined USING_NPC_MODULATOR
    ctl_init_npc_modulator(&spwm, CTRL_PWM_CMP_MAX, CTRL_PWM_DEADBAND_CMP, &mtr_ctrl.iuvw, float2ctrl(0.02),
                           float2ctrl(0.005));
#else
    ctl_init_spwm_modulator(&spwm, CTRL_PWM_CMP_MAX, CTRL_PWM_DEADBAND_CMP, &mtr_ctrl.iuvw, float2ctrl(0.02),
                            float2ctrl(0.005));
#endif // USING_NPC_MODULATOR

    //
    // angle signal generator
    //
    ctl_init_const_slope_f_pu_controller(
        // ramp angle generator
        &rg,
        // target frequency (Hz), target frequency slope (Hz/s)
        20.0f, 20.0f,
        // rated krpm, pole pairs
        MOTOR_PARAM_MAX_SPEED/1000.0f, mtr_ctrl_init.pole_pairs,
        // ISR frequency
        CONTROLLER_FREQUENCY);

    //
    // motion controller
    //
    ctl_init_vel_pos_ctrl(
        // controller object
        &motion_ctrl,
        // spd_kp, pos_kp, spd_ki, pos_ki
        1.0f, 1.0f, 1.0f, 1.0f,
        // spd_lim, cur_lim
        1.0f, 0.3f,
        // spd_div, pos_div, controller freq
        CTRL_SPD_DIV, CTRL_POS_DIV, CONTROLLER_FREQUENCY);

    //
    // Encoder Init
    //
    ctl_init_autoturn_pos_encoder(&pos_enc, mtr_ctrl_init.pole_pairs, CTRL_POS_ENC_FS);
    ctl_set_autoturn_pos_encoder_mech_offset(&pos_enc, float2ctrl(CTRL_POS_ENC_BIAS));

    ctl_init_spd_calculator(&spd_enc, &pos_enc.encif, CONTROLLER_FREQUENCY, CTRL_SPD_DIV, MOTOR_PARAM_MAX_SPEED, 20.0f);

#ifdef ENABLE_SMO

    //
    // Observer Init
    //
    ctl_auto_tuning_pmsm_smo(&smo_init, &mtr_ctrl_init);
    ctl_init_pmsm_smo(&smo, &smo_init);

#endif // ENABLE_SMO

// attach motor current controller with input port
#if BUILD_LEVEL <= 2
    ctl_attach_mtr_current_ctrl_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &rg.enc, &spd_enc.encif);
#else  // BUILD_LEVEL
    ctl_attach_mtr_current_ctrl_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &pos_enc.encif, &spd_enc.encif);
#endif // BUILD_LEVEL

    ctl_attach_vel_pos_ctrl(&motion_ctrl, &pos_enc.encif, &spd_enc.encif);

#if BUILD_LEVEL == 1
    // Voltage open loop
    ctl_disable_mtr_current_ctrl(&mtr_ctrl);
    ctl_set_mtr_current_ctrl_vdq_ref(&mtr_ctrl, 0.2, 0.2);

#elif BUILD_LEVEL == 2
    // Basic current close loop, IF
    ctl_enable_mtr_current_ctrl(&mtr_ctrl);
    ctl_set_mtr_current_ctrl_ref(&mtr_ctrl, float2ctrl(0.1), float2ctrl(0.1));

#elif BUILD_LEVEL == 3
    // Basic current close loop, inverter
    ctl_enable_mtr_current_ctrl(&mtr_ctrl);
    ctl_set_mtr_current_ctrl_ref(&mtr_ctrl, float2ctrl(0.1), float2ctrl(0.1));

#elif BUILD_LEVEL == 4
    // Basic Speed close loop
    ctl_enable_mtr_current_ctrl(&mtr_ctrl);
    ctl_enable_velocity_ctrl(&motion_ctrl);
    ctl_set_target_velocity(&motion_ctrl, 0.1);

#endif // BUILD_LEVEL

    //
    // init and config CiA402 standard state machine
    //
    init_cia402_state_machine(&cia402_sm);
    cia402_sm.minimum_transit_delay[3] = 100;

#if defined SPECIFY_PC_ENVIRONMENT
    cia402_sm.flag_enable_control_word = 0;
    cia402_sm.current_cmd = CIA402_CMD_ENABLE_OPERATION;
#endif // SPECIFY_PC_ENVIRONMENT

    //
    // init and config Motor Protection module
    //
    ctl_init_mtr_protect(&protection, CONTROLLER_FREQUENCY);
    ctl_attach_mtr_protect_port(&protection, &mtr_ctrl.udc, (ctl_vector2_t*)&mtr_ctrl.idq0, &mtr_ctrl.idq_ref, NULL,
                                NULL);

    //
    // init ADC Calibrator
    //
    ctl_init_adc_calibrator(&adc_calibrator, 20, 0.707f, CONTROLLER_FREQUENCY);

    if (flag_enable_adc_calibrator)
    {
        ctl_enable_adc_calibrator(&adc_calibrator);
    }
}

//=================================================================================================
// CTL endless loop routine

void ctl_mainloop(void)
{
    cia402_dispatch(&cia402_sm);

    return;
}

//=================================================================================================
// CiA402 default callback routine

gmp_task_status_t tsk_protect(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

#ifdef ENABLE_MOTOR_FAULT_PROTECTION
    if (ctl_dispatch_mtr_protect_slow(&protection))
    {
        cia402_fault_request(&cia402_sm);
    }
#endif // ENABLE_MOTOR_FAULT_PROTECTION

    return GMP_TASK_DONE;
}

void ctl_enable_pwm()
{
    ctl_fast_enable_output();
}

void ctl_disable_pwm()
{
    ctl_fast_disable_output();
}

void clear_all_controllers()
{
    ctl_clear_mtr_current_ctrl(&mtr_ctrl);
    ctl_clear_vel_pos_ctrl(&motion_ctrl);
    ctl_clear_slope_f_pu(&rg);

#if defined USING_NPC_MODULATOR
    ctl_clear_npc_modulator(&spwm);
#else
    ctl_clear_spwm_modulator(&spwm);
#endif // USING_NPC_MODULATOR
}

fast_gt ctl_exec_adc_calibration(void)
{
    //
    // 1. ADC Auto calibrate
    //
    if (flag_enable_adc_calibrator)
    {
        if (ctl_is_adc_calibrator_cmpt(&adc_calibrator) && ctl_is_adc_calibrator_result_valid(&adc_calibrator))
        {

            // index_adc_calibrator == 7, for Ibus
            if (index_adc_calibrator == 7)
            {
                // vbus get result
                idc.bias = idc.bias + ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), idc.gain);

                // move to next position
                index_adc_calibrator += 1;

                // adc calibrate process done.
                flag_enable_adc_calibrator = 0;

                // clear INV controller
                clear_all_controllers();

                // ADC Calibrator complete here.
                //ctl_enable_gfl_inv(&inv_ctrl);
            }

            // index_adc_calibrator == 6, for Vbus
            else if (index_adc_calibrator == 6)
            {
                // vbus get result
                //udc.bias = udc.bias + ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), udc.gain);

                // move to next position
                index_adc_calibrator += 1;

                // clear calibrator
                ctl_clear_adc_calibrator(&adc_calibrator);

                // enable calibrator to next position
                ctl_enable_adc_calibrator(&adc_calibrator);
            }

            // index_adc_calibrator == 5 ~ 3, for Vuvw
            else if (index_adc_calibrator <= 5 && index_adc_calibrator >= 3)
            {
                // vuvw get result
                uuvw.bias[index_adc_calibrator - 9] =
                    uuvw.bias[index_adc_calibrator - 9] +
                    ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), uuvw.gain[index_adc_calibrator - 9]);

                // move to next position
                index_adc_calibrator += 1;

                // clear calibrator
                ctl_clear_adc_calibrator(&adc_calibrator);

                // enable calibrator to next position
                ctl_enable_adc_calibrator(&adc_calibrator);
            }

            // index_adc_calibrator == 2 ~ 0, for Iuvw
            else if (index_adc_calibrator <= 2)
            {
                // iuvw get result
                iuvw.bias[index_adc_calibrator] =
                    iuvw.bias[index_adc_calibrator] +
                    ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), iuvw.gain[index_adc_calibrator]);

                // move to next position
                index_adc_calibrator += 1;

                // clear calibrator
                ctl_clear_adc_calibrator(&adc_calibrator);

                // enable calibrator to next position
                ctl_enable_adc_calibrator(&adc_calibrator);
            }

            // over-range protection
            if (index_adc_calibrator > 13)
                flag_enable_adc_calibrator = 0;
        }

        // ADC calibrate is not complete
        return 0;
    }

    // skip calibrate routine
    return 1;
}
