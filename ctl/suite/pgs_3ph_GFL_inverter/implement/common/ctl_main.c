
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

//=================================================================================================
// global controller variables

// state machine
cia402_sm_t cia402_sm;

// modulator: SPWM modulator / SVPWM modulator / NPC modulator
#if defined USING_NPC_MODULATOR
npc_modulator_t spwm;
#else
spwm_modulator_t spwm;
#endif // USING_NPC_MODULATOR


// controller body: Current controller, Power controller / Voltage controller
gfl_pq_ctrl_t pq_ctrl;
gfl_inv_ctrl_init_t gfl_init;
gfl_inv_ctrl_t inv_ctrl;

// Observer: PLL

// additional controller: harmonic management, negative current controller

//
volatile fast_gt flag_system_running = 0;
volatile fast_gt flag_error = 0;

// adc calibrator flags
adc_bias_calibrator_t adc_calibrator;
volatile fast_gt flag_enable_adc_calibrator = 1;
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
    // GFL inverter init objects
    //
    gfl_init.fs = CONTROLLER_FREQUENCY;
    gfl_init.v_base = CTRL_VOLTAGE_BASE;
    gfl_init.v_grid = 0.33f;
    gfl_init.i_base = CTRL_CURRENT_BASE;
    gfl_init.freq_base = 50.0f;

    gfl_init.grid_filter_L = 1.5e-3f;
    gfl_init.grid_filter_C = 5.0e-6f;

    ctl_auto_tuning_gfl_inv(&gfl_init);
    ctl_init_gfl_inv(&inv_ctrl, &gfl_init);

    //
    // init SPWM modulator
    //
#if defined USING_NPC_MODULATOR
    ctl_init_npc_modulator(&spwm, CTRL_PWM_CMP_MAX, CTRL_PWM_DEADBAND_CMP, &inv_ctrl.adc_iabc->value, float2ctrl(0.02),
                           float2ctrl(0.005));
#else
    ctl_init_spwm_modulator(&spwm, CTRL_PWM_CMP_MAX, CTRL_PWM_DEADBAND_CMP, &inv_ctrl.adc_iabc->value, float2ctrl(0.02),
                            float2ctrl(0.005));
#endif // USING_NPC_MODULATOR

    //
    // Power controller
    //
    ctl_init_gfl_pq(&pq_ctrl, 0.75f, 0.001f, 0.75f, 0.001f, 1.0f, CONTROLLER_FREQUENCY);
    ctl_attach_gfl_pq_to_core(&pq_ctrl, &inv_ctrl);

#if BUILD_LEVEL == 1
    // Voltage open loop, inverter
    ctl_set_gfl_inv_openloop_mode(&inv_ctrl);
    ctl_set_gfl_inv_voltage_openloop(&inv_ctrl, float2ctrl(0.6), float2ctrl(0.6));

#elif BUILD_LEVEL == 2 || BUILD_LEVEL == 3
    // Basic current close loop, inverter
    ctl_set_gfl_inv_current_mode(&inv_ctrl);
    ctl_set_gfl_inv_current(&inv_ctrl, float2ctrl(0.1), float2ctrl(0.1));

#elif BUILD_LEVEL == 4
    // current close loop with feed forward, inverter
    ctl_set_gfl_inv_current_mode(&inv_ctrl);
    ctl_set_gfl_inv_current(&inv_ctrl, float2ctrl(0.6), float2ctrl(0.6));

    ctl_enable_gfl_inv_decouple(&inv_ctrl);
    ctl_enable_gfl_inv_active_damp(&inv_ctrl);
    ctl_enable_gfl_inv_lead_compensator(&inv_ctrl);

#endif // BUILD_LEVEL

    //
    // init and config CiA402 stdandard state machine
    //
    init_cia402_state_machine(&cia402_sm);
    cia402_sm.minimum_transit_delay[3] = 100;

#if defined SPECIFY_PC_ENVIRONMENT
    cia402_sm.flag_enable_control_word = 0;
    cia402_sm.current_cmd = CIA402_CMD_ENABLE_OPERATION;
#endif // SPECIFY_PC_ENVIRONMENT

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

void ctl_enable_pwm()
{
    ctl_fast_enable_output();
}

void ctl_disable_pwm()
{
    ctl_fast_disable_output();
}

fast_gt ctl_check_pll_locked(void)
{
    return (ctl_abs(ctl_get_gfl_pll_error(&inv_ctrl)) < CTRL_SPLL_EPSILON);
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

            // index_adc_calibrator == 13, for Ibus
            if (index_adc_calibrator == 13)
            {
                // vbus get result
                idc.bias = idc.bias + ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), idc.gain);

                // move to next position
                index_adc_calibrator += 1;

                // adc calibrate process done.
                flag_enable_adc_calibrator = 0;

                // clear INV controller
                ctl_clear_gfl_inv_with_PLL(&inv_ctrl);

                // ADC Calibrator complete here.
                ctl_enable_gfl_inv(&inv_ctrl);
            }

            // index_adc_calibrator == 12, for Vbus
            else if (index_adc_calibrator == 12)
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

            // index_adc_calibrator == 11 ~ 9, for Vuvw
            else if (index_adc_calibrator <= 11 && index_adc_calibrator >= 9)
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

            // index_adc_calibrator == 8 ~ 6, for Vabc
            else if (index_adc_calibrator <= 8 && index_adc_calibrator >= 6)
            {
                // vabc get result
                vabc.bias[index_adc_calibrator - 6] =
                    vabc.bias[index_adc_calibrator - 6] +
                    ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), vabc.gain[index_adc_calibrator - 6]);

                // move to next position
                index_adc_calibrator += 1;

                // clear calibrator
                ctl_clear_adc_calibrator(&adc_calibrator);

                // enable calibrator to next position
                ctl_enable_adc_calibrator(&adc_calibrator);
            }

            // index_adc_calibrator == 5 ~ 3, for Iabc
            else if (index_adc_calibrator <= 5 && index_adc_calibrator >= 3)
            {

                // iabc get result
                iabc.bias[index_adc_calibrator - 3] =
                    iabc.bias[index_adc_calibrator - 3] +
                    ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), iabc.gain[index_adc_calibrator - 3]);

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
