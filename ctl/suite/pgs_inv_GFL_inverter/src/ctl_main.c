
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

#include "ctl_settings_defaults.h"

#include "ctl_main.h"

#include <xplt.peripheral.h>

#include <core/dev/pil_core.h>

//=================================================================================================
// global controller variables

// System framework
cia402_sm_t cia402_sm;

// Control Law Core
// Current controller, Power controller / Voltage controller
gfl_pq_ctrl_t pq_ctrl;
gfl_pq_droop_init_t pq_droop_init;
gfl_pq_droop_ctrl_t pq_droop_ctrl;
ctrl_gt gfl_pll_frequency_hz;
inv_neg_ctrl_init_t gfl_neg_init;
inv_neg_ctrl_t neg_current_ctrl;
inv_voltage_ctrl_init_t gfl_voltage_init;
inv_voltage_ctrl_t gfl_voltage_ctrl;
inv_zero_ctrl_init_t gfl_zero_init;
inv_zero_ctrl_t gfl_zero_ctrl;
gfl_inv_ctrl_init_t gfl_init;
gfl_inv_ctrl_t inv_ctrl;

// Input channel

// Output channel: SPWM modulator / SVPWM modulator / NPC modulator
#if defined USING_NPC_MODULATOR
npc_modulator_t spwm;
#else
spwm_modulator_t spwm;
#endif // USING_NPC_MODULATOR
#if defined USING_3D_SVPWM
ctl_vector4_t pwm_3d_duty;
pwm_gt pwm_3d_out[4];
#endif

// Protection module

// ADC Calibrator
adc_bias_calibrator_t adc_calibrator;
#if defined SPECIFY_ENABLE_ADC_CALIBRATE
volatile fast_gt flag_enable_adc_calibrator = 1;
#else
volatile fast_gt flag_enable_adc_calibrator = 0;
#endif
volatile fast_gt index_adc_calibrator = 0;
uint32_t pq_loop_tick = 0;

// User commands

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
    gfl_init.v_grid = GFL_GRID_VOLTAGE_PU;
    gfl_init.i_base = CTRL_CURRENT_BASE;
    gfl_init.freq_base = GFL_GRID_FREQUENCY_HZ;

    gfl_init.grid_filter_L = GFL_GRID_FILTER_INDUCTANCE_H;
    gfl_init.grid_filter_C = GFL_GRID_FILTER_CAPACITANCE_F;

    ctl_auto_tuning_gfl_inv(&gfl_init);
    ctl_init_gfl_inv(&inv_ctrl, &gfl_init);

    ctl_auto_tuning_neg_inv(&gfl_neg_init, &gfl_init);
    ctl_init_neg_inv(&neg_current_ctrl, &gfl_neg_init);
    ctl_attach_neg_inv_to_gfl(&neg_current_ctrl, &inv_ctrl);

    ctl_auto_tuning_voltage_inv(&gfl_voltage_init, &gfl_init);
    gfl_voltage_init.voltage_loop_bw = GFL_VOLTAGE_LOOP_BW_HZ;
    gfl_voltage_init.voltage_loop_zero = GFL_VOLTAGE_LOOP_ZERO_HZ;
    gfl_voltage_init.current_circle_limit = GFL_VOLTAGE_CIRCLE_LIMIT_PU;
    gfl_voltage_init.current_square_limit = GFL_VOLTAGE_SQUARE_LIMIT_PU;

#if defined GFL_ENABLE_VOLTAGE_CIRCLE_LIMIT
    gfl_voltage_init.flag_enable_circle_limit = 1;
#else
    gfl_voltage_init.flag_enable_circle_limit = 0;
#endif

#if defined GFL_ENABLE_VOLTAGE_SQUARE_LIMIT
    gfl_voltage_init.flag_enable_square_limit = 1;
#else
    gfl_voltage_init.flag_enable_square_limit = 0;
#endif

    ctl_init_voltage_inv(&gfl_voltage_ctrl, &gfl_voltage_init);
    ctl_attach_voltage_inv_to_gfl(&gfl_voltage_ctrl, &inv_ctrl);

    ctl_auto_tuning_zero_inv(&gfl_zero_init, &gfl_init);
    gfl_zero_init.kp = GFL_ZERO_QPR_KP;
    gfl_zero_init.kr = GFL_ZERO_QPR_KR;
    gfl_zero_init.freq_cut = GFL_ZERO_QPR_CUTOFF_HZ;
    gfl_zero_init.output_limit_max = GFL_ZERO_VOLTAGE_LIMIT_PU;
    gfl_zero_init.output_limit_min = -GFL_ZERO_VOLTAGE_LIMIT_PU;
    ctl_init_zero_inv(&gfl_zero_ctrl, &gfl_zero_init);
    ctl_attach_zero_inv(&gfl_zero_ctrl, &inv_ctrl.iab0.dat[phase_0], &gfl_zero_ctrl.v0_out);

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
    ctl_init_gfl_pq(&pq_ctrl, GFL_PQ_ACTIVE_KP, GFL_PQ_ACTIVE_KI, GFL_PQ_REACTIVE_KP, GFL_PQ_REACTIVE_KI,
                    GFL_PQ_CURRENT_LIMIT_PU, GFL_PQ_LOOP_FREQUENCY_HZ);
    ctl_attach_gfl_pq_to_core(&pq_ctrl, &inv_ctrl);
    ctl_set_gfl_pq_ref(&pq_ctrl, float2ctrl(GFL_ACTIVE_POWER_REF_PU), float2ctrl(GFL_REACTIVE_POWER_REF_PU));

    pq_droop_init.fs = GFL_PQ_LOOP_FREQUENCY_HZ;
    pq_droop_init.lpf_hz = GFL_PQ_DROOP_LPF_HZ;
    pq_droop_init.frequency_nominal_hz = GFL_GRID_FREQUENCY_HZ;
    pq_droop_init.voltage_nominal = GFL_GRID_VOLTAGE_PU;
    pq_droop_init.p_gain_pu_per_hz = GFL_PQ_DROOP_P_GAIN_PU_PER_HZ;
    pq_droop_init.q_gain_pu_per_v_pu = GFL_PQ_DROOP_Q_GAIN_PU_PER_V_PU;
    pq_droop_init.p_min = GFL_PQ_DROOP_P_MIN_PU;
    pq_droop_init.p_max = GFL_PQ_DROOP_P_MAX_PU;
    pq_droop_init.q_min = GFL_PQ_DROOP_Q_MIN_PU;
    pq_droop_init.q_max = GFL_PQ_DROOP_Q_MAX_PU;
    ctl_init_gfl_pq_droop(&pq_droop_ctrl, &pq_droop_init);
    gfl_pll_frequency_hz = float2ctrl(GFL_GRID_FREQUENCY_HZ);
    ctl_attach_gfl_pq_droop(&pq_droop_ctrl, &gfl_pll_frequency_hz,
                            &inv_ctrl.vdq);
    ctl_set_gfl_pq_droop_base(
        &pq_droop_ctrl, float2ctrl(GFL_ACTIVE_POWER_REF_PU),
        float2ctrl(GFL_REACTIVE_POWER_REF_PU));
#if defined GFL_ENABLE_PQ_DROOP
    ctl_enable_gfl_pq_droop(&pq_droop_ctrl);
#endif
    pq_loop_tick = 0;

#if BUILD_LEVEL == 1
    // Voltage open loop, inverter
    ctl_set_gfl_inv_openloop_mode(&inv_ctrl);
    ctl_set_gfl_inv_voltage_openloop(&inv_ctrl, float2ctrl(GFL_OPEN_LOOP_VD_PU), float2ctrl(GFL_OPEN_LOOP_VQ_PU));

#elif BUILD_LEVEL == 2
    // Basic current close loop, inverter
    ctl_set_gfl_inv_current_mode(&inv_ctrl);
    ctl_set_gfl_inv_current(&inv_ctrl, float2ctrl(GFL_CURRENT_LEVEL2_ID_PU), float2ctrl(GFL_CURRENT_LEVEL2_IQ_PU));

#elif BUILD_LEVEL == 3
    // Basic current close loop, inverter
    ctl_set_gfl_inv_current_mode(&inv_ctrl);
    ctl_set_gfl_inv_current(&inv_ctrl, float2ctrl(GFL_CURRENT_LEVEL3_ID_PU), float2ctrl(GFL_CURRENT_LEVEL3_IQ_PU));

    ctl_enable_neg_current_inv(&neg_current_ctrl);

    ctl_enable_gfl_inv_pll(&inv_ctrl);
    ctl_set_gfl_inv_grid_connect(&inv_ctrl);

#elif BUILD_LEVEL == 4
    // current close loop with feed forward, inverter
    ctl_set_gfl_inv_current_mode(&inv_ctrl);
    ctl_set_gfl_inv_current(&inv_ctrl, float2ctrl(GFL_CURRENT_LEVEL4_ID_PU), float2ctrl(GFL_CURRENT_LEVEL4_IQ_PU));

    ctl_enable_neg_current_inv(&neg_current_ctrl);

    ctl_enable_gfl_inv_pll(&inv_ctrl);
    ctl_set_gfl_inv_grid_connect(&inv_ctrl);

    inv_ctrl.flag_enable_decouple = 1;
    inv_ctrl.flag_enable_active_damping = 1;
    inv_ctrl.flag_enable_lead_compensator = 1;

#elif BUILD_LEVEL == 5
    // Cascaded P/Q power loop -> d/q current loop, grid connected.
    ctl_set_gfl_inv_current_mode(&inv_ctrl);
    ctl_set_gfl_inv_current(&inv_ctrl, 0, 0);

    ctl_enable_neg_current_inv(&neg_current_ctrl);
    ctl_enable_gfl_inv_pll(&inv_ctrl);
    ctl_set_gfl_inv_grid_connect(&inv_ctrl);
    inv_ctrl.flag_enable_decouple = 1;
    inv_ctrl.flag_enable_active_damping = 1;
    inv_ctrl.flag_enable_lead_compensator = 1;
    ctl_enable_gfl_pq_ctrl(&pq_ctrl);

#elif BUILD_LEVEL == 6
    // Stand-alone LC-filter capacitor-voltage loop -> d/q current loop.
    ctl_set_gfl_inv_current_mode(&inv_ctrl);
    ctl_set_gfl_inv_current(&inv_ctrl, 0, 0);
    ctl_set_voltage_inv_reference(&gfl_voltage_ctrl, float2ctrl(GFL_STANDALONE_VD_PU),
                                  float2ctrl(GFL_STANDALONE_VQ_PU));
    ctl_enable_voltage_inv(&gfl_voltage_ctrl);
#if defined GFL_ENABLE_VOLTAGE_DECOUPLE
    ctl_enable_voltage_inv_decouple(&gfl_voltage_ctrl);
#else
    ctl_disable_voltage_inv_decouple(&gfl_voltage_ctrl);
#endif

#if defined USING_3D_SVPWM
    // Suppress negative-sequence capacitor voltage and zero-sequence current.
    ctl_enable_neg_voltage_inv(&neg_current_ctrl);
    ctl_enable_zero_inv(&gfl_zero_ctrl);
#endif

#endif // BUILD_LEVEL

#if defined USING_3D_SVPWM && (BUILD_LEVEL >= 3) && (BUILD_LEVEL <= 5)
    ctl_enable_zero_inv(&gfl_zero_ctrl);
#endif

    //
    // init and config CiA402 standard state machine
    //
    init_cia402_state_machine(&cia402_sm);
    cia402_sm.minimum_transit_delay[3] = GFL_CIA402_OPERATION_ENABLE_DELAY_MS;

#if defined SPECIFY_PC_ENVIRONMENT
    cia402_sm.flag_enable_control_word = 0;
    cia402_sm.current_cmd = CIA402_CMD_ENABLE_OPERATION;
#endif // SPECIFY_PC_ENVIRONMENT

#if (BUILD_LEVEL >= 3) && (BUILD_LEVEL <= 5)

    // NOTICE:
    // if grid connect is request disable switch delay from CIA402_SM_SWITCH_ON_DISABLED to CIA402_SM_SWITCHED_ON
    // or a longer judgment time can lead to failure to connect to the grid.
    cia402_sm.minimum_transit_delay[CIA402_SM_READY_TO_SWITCH_ON] = 0;
    cia402_sm.minimum_transit_delay[CIA402_SM_SWITCHED_ON] = 0;

#endif // BUILD_LEVEL

    //
    // init ADC Calibrator
    //
    ctl_init_adc_calibrator(&adc_calibrator, GFL_ADC_CALIBRATOR_FC_HZ, GFL_ADC_CALIBRATOR_Q, CONTROLLER_FREQUENCY);

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

void gmp_pil_sim_step(const gmp_sim_rx_buf_t* rx, gmp_sim_tx_buf_t* tx)
{
#if defined ENABLE_GMP_DL_PIL_SIM
    ctl_input_callback_pil(rx);

    ctl_dispatch();

    ctl_output_callback_pil(tx);
#endif // defined ENABLE_GMP_DL_PIL_SIM
}

#if defined ENABLE_GMP_DL_PIL_SIM
time_gt gmp_base_get_ctrl_tick(void)
{
    return inv_ctrl.isr_tick / ((uint32_t)CONTROLLER_FREQUENCY / 1000);
}
#endif // defined ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
// CiA402 default callback routine

void ctl_enable_pwm()
{
    /*
     * CiA402 clears dynamic controller state while output is disabled. Restore
     * commissioning references at the enable edge so BUILD_LEVEL 2-4 current
     * commands are not lost during the startup state transitions.
     */
#if BUILD_LEVEL == 1
    ctl_set_gfl_inv_voltage_openloop(&inv_ctrl, float2ctrl(GFL_OPEN_LOOP_VD_PU),
                                     float2ctrl(GFL_OPEN_LOOP_VQ_PU));
#elif BUILD_LEVEL == 2
    ctl_set_gfl_inv_current(&inv_ctrl, float2ctrl(GFL_CURRENT_LEVEL2_ID_PU),
                            float2ctrl(GFL_CURRENT_LEVEL2_IQ_PU));
#elif BUILD_LEVEL == 3
    ctl_set_gfl_inv_current(&inv_ctrl, float2ctrl(GFL_CURRENT_LEVEL3_ID_PU),
                            float2ctrl(GFL_CURRENT_LEVEL3_IQ_PU));
#elif BUILD_LEVEL == 4
    ctl_set_gfl_inv_current(&inv_ctrl, float2ctrl(GFL_CURRENT_LEVEL4_ID_PU),
                            float2ctrl(GFL_CURRENT_LEVEL4_IQ_PU));
#elif BUILD_LEVEL == 5
    ctl_set_gfl_inv_current(&inv_ctrl, 0, 0);
#endif

    ctl_fast_enable_output();
}

void ctl_disable_pwm()
{
    ctl_fast_disable_output();

    // clear controller here
    ctl_clear_gfl_inv(&inv_ctrl);
    ctl_clear_neg_inv(&neg_current_ctrl);
    ctl_clear_voltage_inv(&gfl_voltage_ctrl);
    ctl_clear_zero_inv(&gfl_zero_ctrl);
    ctl_clear_gfl_pq(&pq_ctrl);
    ctl_clear_gfl_pq_droop(&pq_droop_ctrl);
    gfl_pll_frequency_hz = float2ctrl(GFL_GRID_FREQUENCY_HZ);
    pq_loop_tick = 0;
}

fast_gt ctl_check_pll_locked(void)
{
    ctrl_gt pll_erro = ctl_abs(ctl_get_gfl_pll_error(&inv_ctrl));

    // grid connected, judge if PLL is ready.
    if (ctl_is_gfl_grid_connected(&inv_ctrl))
    {
        if (pll_erro < CTRL_SPLL_EPSILON)
            return 1;
        else
            return 0;
    }

    // not connect to gird
    else
        return 1;
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
                //ctl_enable_gfl_inv(&inv_ctrl);
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

#if !defined SPECIFY_PC_ENVIRONMENT
/** @brief Provide GFL current references and feedback to the platform Scope. */
void user_get_scope_channels(ctrl_gt channels[4])
{
    channels[0] = inv_ctrl.idq_set.dat[phase_d];
    channels[1] = inv_ctrl.idq_set.dat[phase_q];
    channels[2] = inv_ctrl.idq.dat[phase_d];
    channels[3] = inv_ctrl.idq.dat[phase_q];
}
#endif
