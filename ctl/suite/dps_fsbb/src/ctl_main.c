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
volatile uint16_t g_fsbb_faults = FSBB_FAULT_NONE;
volatile fast_gt g_fsbb_output_enabled = 0;

// User commands
ctrl_gt g_v_out_ref_user = float2ctrl(FSBB_DEFAULT_OUTPUT_VOLTAGE / CTRL_VOLTAGE_BASE);
ctrl_gt g_i_limit_user = float2ctrl(FSBB_DEFAULT_CURRENT_LIMIT / CTRL_CURRENT_BASE);
ctrl_gt v_req = float2ctrl(0.0f);

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

    fsbb_init.slope_v_pu_s = FSBB_VOLTAGE_RAMP_PU_S;
    fsbb_init.slope_i_pu_s = FSBB_CURRENT_RAMP_PU_S;

    fsbb_init.i_out_max = float2ctrl(FSBB_OUTPUT_CURRENT_LIM / CTRL_CURRENT_BASE);
    fsbb_init.i_out_min = float2ctrl(0.0f);
    fsbb_init.v_cmd_max = float2ctrl(FSBB_OUTPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE);
    fsbb_init.v_cmd_min = float2ctrl(0.0f);

    fsbb_init.fc_current_loop = FSBB_CURRENT_LOOP_BANDWIDTH;
    fsbb_init.fc_voltage_loop = FSBB_VOLTAGE_LOOP_BANDWIDTH;

    ctl_dcdc_core_init_t core_init = {0};
    ctl_dcdc_blueprint_fsbb_cascade(&core_init, &fsbb_init);

    // init FSBB controller core
    ctl_init_dcdc_core(&dcdc_core, &core_init);
    ctl_set_dcdc_core_limits(&dcdc_core,
                             float2ctrl(FSBB_OUTPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE),
                             float2ctrl(0.0f));

    // attach FSBB with ADC peripheral
    ctl_attach_dcdc_core(&dcdc_core, &adc_v_in.control_port, &adc_v_out.control_port, &adc_i_L.control_port,
                         &adc_i_load.control_port);

    v_req = float2ctrl(FSBB_OPEN_LOOP_VOLTAGE_COMMAND / CTRL_VOLTAGE_BASE);

    //
    // init FSBB PWM modulator
    //
    ctl_init_fsbb_modulator(&fsbb_mod, CTRL_PWM_CMP_MAX, float2ctrl(FSBB_DUTY_MAX), float2ctrl(FSBB_DUTY_MIN),
                            float2ctrl(FSBB_TRANSITION_RATIO_LOW), float2ctrl(FSBB_TRANSITION_RATIO_HIGH));

    //
    // init and config CiA402 standard state machine
    //
    init_cia402_state_machine(&cia402_sm);
    cia402_sm.minimum_transit_delay[3] = 100; // Stabilization delay before enabling operation.

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
#if defined SPECIFY_ENABLE_ADC_CALIBRATE
    flag_enable_adc_calibrator = 1;
    index_adc_calibrator = 0;
    ctl_enable_adc_calibrator(&adc_calibrator);
#endif

    //
    // incremental compilation configuration
    //
}

//=================================================================================================
// CTL endless loop routine

void ctl_mainloop(void)
{
    // The SIL target preloads ENABLE_OPERATION as its current command, so it
    // follows the same wait/calibration/enable sequence as embedded targets.
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
    return gmp_base_get_system_tick();
}
#endif // defined ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
// Controller Tasks

gmp_task_status_t tsk_protect(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    if (g_fsbb_faults != FSBB_FAULT_NONE)
        cia402_fault_request(&cia402_sm);
    return GMP_TASK_DONE;
}

//=================================================================================================
// CiA402 default callback routine

//
// ADC Auto calibrate routine
//
fast_gt ctl_exec_adc_calibration(void)
{
#if !defined SPECIFY_ENABLE_ADC_CALIBRATE
    return 1;
#else
    if (!flag_enable_adc_calibrator)
        return 1;

    if (ctl_is_adc_calibrator_cmpt(&adc_calibrator) && ctl_is_adc_calibrator_result_valid(&adc_calibrator))
    {
        if (index_adc_calibrator == 0)
        {
            adc_i_L.bias += ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), adc_i_L.gain);
            index_adc_calibrator++;
#if defined FSBB_ENABLE_IOUT_SAMPLE
            ctl_clear_adc_calibrator(&adc_calibrator);
            ctl_enable_adc_calibrator(&adc_calibrator);
        }
        else if (index_adc_calibrator == 1)
        {
            adc_i_load.bias += ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), adc_i_load.gain);
            index_adc_calibrator++;
#endif
        }

#if defined FSBB_ENABLE_IOUT_SAMPLE
        if (index_adc_calibrator >= 2)
#else
        if (index_adc_calibrator >= 1)
#endif
        {
            flag_enable_adc_calibrator = 0;
            ctl_clear_adc_calibrator(&adc_calibrator);
            clear_all_controllers();
            return 1;
        }
    }
    return 0;
#endif
}

void clear_all_controllers(void)
{
    ctl_clear_dcdc_core(&dcdc_core);
}

void ctl_enable_pwm(void)
{
    if (g_fsbb_faults == FSBB_FAULT_NONE)
    {
        ctl_fast_enable_output();
        /* ctl_fast_enable_output() clears all ramps and controllers. Replace
           the pre-enable compare values in the same outgoing SIL frame so
           the power stage starts from the configured zero-command duty. */
        ctl_step_fsbb_modulator(&fsbb_mod, float2ctrl(0.0f), adc_v_in.control_port.value);
        g_fsbb_output_enabled = 1;
    }
}

void ctl_disable_pwm(void)
{
    ctl_fast_disable_output();
    g_fsbb_output_enabled = 0;
}
