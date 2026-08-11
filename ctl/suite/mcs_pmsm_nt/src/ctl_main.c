
//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should define your own controller objects,
// and initilize them.
//
// User should implement a ctl loop function, this
// function would be called every main loop.
//

#include <gmp_core.h>

#include "ctl_settings_defaults.h"

// Backward compatibility for project settings that still use the historical
// misspelled PIL switch. New SDPE projects use ENABLE_GMP_DL_PIL_SIM.
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif

#include "ctl_main.h"

#include <xplt.peripheral.h>

#include <core/pm/function_scheduler.h>

//=================================================================================================
// global controller variables

// System framework
cia402_sm_t cia402_sm;

// Control Law Core

// controller body: Current controller, Command dispatcher, motion controller
mc_foc_core_t mtr_ctrl;
mc_foc_init_t mtr_ctrl_init;

ctl_mech_ctrl_t mech_ctrl;
ctl_mech_init_t mech_init;

// Observer: SMO, FO, Speed measurement.
ctl_slope_f_pu_controller rg;
pos_autoturn_encoder_t pos_enc;
spd_calculator_t spd_enc;

#ifdef ENABLE_SMO

ctl_pmsm_esmo_init_t smo_init;
ctl_pmsm_esmo_t smo;

#endif // ENABLE_SMO

// Input channel

// Output channel
// modulator: SPWM modulator / SVPWM modulator / NPC modulator
#if defined USING_NPC_MODULATOR
npc_modulator_t spwm;
#else
spwm_modulator_t spwm;
#endif // USING_NPC_MODULATOR

// Protection module
ctl_mtr_protect_t protection;

// ADC Calibrator
adc_bias_calibrator_t adc_calibrator;
#if defined ENABLE_GMP_DL_PIL_SIM
/** Simulated ADC channels are already calibrated by the plant model. */
volatile fast_gt flag_enable_adc_calibrator = 0;
#elif defined SPECIFY_ENABLE_ADC_CALIBRATE
volatile fast_gt flag_enable_adc_calibrator = 1;
#else
volatile fast_gt flag_enable_adc_calibrator = 0;
#endif
volatile fast_gt index_adc_calibrator = 0;

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
    // motor current controller init objects
    //
    mtr_ctrl_init.fs = CONTROLLER_FREQUENCY;
    mtr_ctrl_init.v_base = CTRL_VOLTAGE_BASE;
    mtr_ctrl_init.i_base = CTRL_CURRENT_BASE;

    // Physical DC-link voltage used by the bus-voltage compensator.
    mtr_ctrl_init.v_bus = CTRL_DCBUS_VOLTAGE;
    mtr_ctrl_init.v_phase_limit = MCS_MAX_CIR_SATURATION_VOLTAGE_V;

    mtr_ctrl_init.freq_base = MOTOR_PARAM_RATED_FREQUENCY;
    mtr_ctrl_init.spd_base = CTRL_SPEED_RPM_BASE / 1000;
    mtr_ctrl_init.pole_pairs = MOTOR_PARAM_POLE_PAIRS;

    mtr_ctrl_init.mtr_Ld = MOTOR_PARAM_LS;
    mtr_ctrl_init.mtr_Lq = MOTOR_PARAM_LS;
    mtr_ctrl_init.mtr_Rs = MOTOR_PARAM_RS;

    ctl_auto_tuning_foc_core(&mtr_ctrl_init);
    ctl_init_foc_core(&mtr_ctrl, &mtr_ctrl_init);

    ctl_set_foc_core_saturation(&mtr_ctrl, MCS_MAX_RECT_SATURATION_VOLTAGE_V / CTRL_VOLTAGE_BASE,
                                MCS_MAX_CIR_SATURATION_VOLTAGE_V / CTRL_VOLTAGE_BASE);

    //
    // init SPWM modulator
    //
#if defined USING_NPC_MODULATOR
    ctl_init_npc_modulator(&spwm, CTRL_PWM_CMP_MAX, CTRL_PWM_DEADBAND_CMP, &mtr_ctrl.iuvw,
                           float2ctrl(MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A / CTRL_CURRENT_BASE),
                           float2ctrl(MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A / CTRL_CURRENT_BASE));
#else
    ctl_init_spwm_modulator(&spwm, CTRL_PWM_CMP_MAX, CTRL_PWM_DEADBAND_CMP, &mtr_ctrl.iuvw,
                            float2ctrl(MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A / CTRL_CURRENT_BASE),
                            float2ctrl(MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A / CTRL_CURRENT_BASE));
#endif // USING_NPC_MODULATOR

    //
    // angle signal generator
    //
    ctl_init_const_slope_f_pu_controller(
        // ramp angle generator
        &rg,
        // target frequency (Hz), target frequency slope (Hz/s)
        MCS_OPEN_LOOP_FREQ_HZ, MCS_OPEN_LOOP_FREQ_SLOPE_HZ_S,
        // rated krpm, pole pairs
        CTRL_SPEED_RPM_BASE / 1000.0f, mtr_ctrl_init.pole_pairs,
        // ISR frequency
        CONTROLLER_FREQUENCY);

    //
    // mechanical controller
    //
    mech_init.fs = CONTROLLER_FREQUENCY;

    mech_init.pos_kp = MCS_MECH_POSITION_KP_PU;
    mech_init.pos_ki = MCS_MECH_POSITION_KI_PU_S;

    mech_init.vel_kp = MCS_MECH_VELOCITY_KP_PU;
    mech_init.vel_ki = MCS_MECH_VELOCITY_KI_PU_S;

    mech_init.speed_limit = MCS_MECH_SPEED_LIMIT_RPM / CTRL_SPEED_RPM_BASE;
    mech_init.speed_slope_limit = MCS_MECH_SPEED_SLOPE_RPM_S / CTRL_SPEED_RPM_BASE;
    mech_init.cur_limit = MCS_MECH_CURRENT_LIMIT_A / CTRL_CURRENT_BASE;

    mech_init.mech_division = CTRL_MECH_DIV;

    ctl_init_mech_ctrl(&mech_ctrl, &mech_init);

    //
    // Encoder Init
    //
    ctl_init_autoturn_pos_encoder(&pos_enc, mtr_ctrl_init.pole_pairs, CTRL_POS_ENC_FS);
    ctl_set_autoturn_pos_encoder_mech_offset(&pos_enc, float2ctrl(CTRL_POS_ENC_BIAS));

    ctl_init_spd_calculator(&spd_enc, &pos_enc.encif, CONTROLLER_FREQUENCY, CTRL_MECH_DIV, CTRL_SPEED_RPM_BASE,
                            MCS_ENCODER_SPEED_FILTER_FC_HZ);

#ifdef ENABLE_SMO

    //
    // Observer Init
    //
    ctl_autotune_esmo_init_from_mtr(&smo_init, &mtr_ctrl_init, MOTOR_PARAM_FLUX);
    ctl_init_pmsm_esmo(&smo, &smo_init);

#endif // ENABLE_SMO

// attach motor current controller with input port
#if BUILD_LEVEL <= 2
    ctl_attach_foc_core_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &rg.enc, &spd_enc.encif);
#else  // BUILD_LEVEL
    ctl_attach_foc_core_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &pos_enc.encif, &spd_enc.encif);
#endif // BUILD_LEVEL

    ctl_attach_mech_ctrl(&mech_ctrl, &pos_enc.encif, &spd_enc.encif);

    //
    // incremental compilation configuration
    //

#if BUILD_LEVEL == 1
    // Voltage open loop
    ctl_disable_foc_core_current_ctrl(&mtr_ctrl);
    ctl_set_foc_core_vdq_ref(&mtr_ctrl, 0.0, 0.0);

#elif BUILD_LEVEL == 2
    // Basic current close loop, IF
    ctl_enable_foc_core_current_ctrl(&mtr_ctrl);
    ctl_set_foc_core_idq_ref(&mtr_ctrl, float2ctrl(MCS_COMMISSIONING_ID_REF_A / CTRL_CURRENT_BASE),
                             float2ctrl(MCS_COMMISSIONING_IQ_REF_A / CTRL_CURRENT_BASE));

#elif BUILD_LEVEL == 3
    // Basic current close loop, inverter
    ctl_enable_foc_core_current_ctrl(&mtr_ctrl);
    ctl_set_foc_core_idq_ref(&mtr_ctrl, float2ctrl(MCS_COMMISSIONING_ID_REF_A / CTRL_CURRENT_BASE),
                             float2ctrl(MCS_COMMISSIONING_IQ_REF_A / CTRL_CURRENT_BASE));

#elif BUILD_LEVEL == 4
    // Basic Speed close loop
    ctl_enable_foc_core_current_ctrl(&mtr_ctrl);
    ctl_set_mech_ctrl_mode(&mech_ctrl, MECH_MODE_VELOCITY);
    ctl_set_mech_target_velocity(&mech_ctrl,
                                 float2ctrl(MCS_COMMISSIONING_SPEED_REF_RPM / CTRL_SPEED_RPM_BASE));

#endif // BUILD_LEVEL

    //
    // init and config CiA402 standard state machine
    //
    init_cia402_state_machine(&cia402_sm);
#if defined ENABLE_GMP_DL_PIL_SIM
    /** Start every virtual power-state transition immediately. */
    cia402_sm.minimum_transit_delay[0] = 0;
    cia402_sm.minimum_transit_delay[1] = 0;
    cia402_sm.minimum_transit_delay[2] = 0;
    cia402_sm.minimum_transit_delay[3] = 0;
#else
    cia402_sm.minimum_transit_delay[3] = MCS_CIA402_OPERATION_ENABLE_DELAY_MS;
#endif

#if defined SPECIFY_PC_ENVIRONMENT || defined ENABLE_GMP_DL_PIL_SIM
    /** PIL follows the SIL auto-enable behavior while physical PWM stays tripped. */
    cia402_sm.flag_enable_control_word = 0;
    cia402_sm.current_cmd = CIA402_CMD_ENABLE_OPERATION;
#endif // defined SPECIFY_PC_ENVIRONMENT || defined ENABLE_GMP_DL_PIL_SIM

    //
    // init and config Motor Protection module
    //
    ctl_init_mtr_protect(&protection, CONTROLLER_FREQUENCY);

    ctl_set_mtr_protect_ov(&protection, float2ctrl(MCS_MAX_DC_BUS_VOLTAGE_V/CTRL_DCBUS_VOLTAGE));
    ctl_set_mtr_protect_uv(&protection, float2ctrl(MCS_MIN_DC_BUS_VOLTAGE_V/CTRL_DCBUS_VOLTAGE));
    ctl_set_mtr_protect_oc(&protection, float2ctrl(MCS_MAX_SHUTDOWN_CURRENT_A/CTRL_CURRENT_BASE));

    ctl_attach_mtr_protect_port(&protection, &mtr_ctrl.udc, (ctl_vector2_t*)&mtr_ctrl.idq0, &mtr_ctrl.idq_ref, NULL,
                                NULL);
    ctl_set_mtr_protect_mask(&protection, MTR_PROT_DEVIATION);

    //
    // init ADC Calibrator
    //
    ctl_init_adc_calibrator(&adc_calibrator, MCS_ADC_CALIBRATOR_FC_HZ, MCS_ADC_CALIBRATOR_Q,
                            CONTROLLER_FREQUENCY);

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

#if defined ENABLE_GMP_DL_PIL_SIM
/** @brief Apply one standard PMSM SIL/PIL input frame to controller ports. */
static void ctl_apply_pil_input(const gmp_sim_rx_buf_t* rx)
{
    uuvw_src[phase_U] = rx->adc_result[1];
    uuvw_src[phase_V] = rx->adc_result[2];
    uuvw_src[phase_W] = rx->adc_result[3];
    iuvw_src[phase_U] = rx->adc_result[4];
    iuvw_src[phase_V] = rx->adc_result[5];
    iuvw_src[phase_W] = rx->adc_result[6];
    udc_src = rx->adc_result[0];
    ctl_step_autoturn_pos_encoder(&pos_enc, rx->digital_input);
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

/** @brief Export one controller result using the established PMSM SIL ABI. */
static void ctl_collect_pil_output(gmp_sim_tx_buf_t* tx)
{
    tx->pwm_cmp[0] = spwm.pwm_out[phase_U];
    tx->pwm_cmp[1] = spwm.pwm_out[phase_V];
    tx->pwm_cmp[2] = spwm.pwm_out[phase_W];
    tx->monitor[0] = mtr_ctrl.iuvw.dat[phase_A];
    tx->monitor[1] = mtr_ctrl.iuvw.dat[phase_B];
}

#endif // defined ENABLE_GMP_DL_PIL_SIM

/** @brief Execute one controller step requested by the Data Link PIL service. */
void gmp_pil_sim_step(const gmp_sim_rx_buf_t* rx, gmp_sim_tx_buf_t* tx)
{
#if defined ENABLE_GMP_DL_PIL_SIM
    ctl_apply_pil_input(rx);

    ctl_dispatch();

    ctl_collect_pil_output(tx);
#else
    GMP_UNUSED_VAR(rx);
    GMP_UNUSED_VAR(tx);
#endif // defined ENABLE_GMP_DL_PIL_SIM
}

#if defined ENABLE_GMP_DL_PIL_SIM
time_gt gmp_base_get_ctrl_tick(void)
{
    return mtr_ctrl.isr_tick / ((uint32_t)CONTROLLER_FREQUENCY / 1000);
}
#endif // defined ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
// Controller Tasks

gmp_task_status_t tsk_protect(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    uint32_t error_mask = ctl_get_mtr_protect_mask(&protection);

    if (mtr_ctrl.flag_enable_current_ctrl)
    {
        error_mask = error_mask & ~MTR_PROT_DEVIATION;
    }
    else
    {
        error_mask = error_mask | MTR_PROT_DEVIATION;
    }

    ctl_set_mtr_protect_mask(&protection, error_mask);

#ifdef ENABLE_MOTOR_FAULT_PROTECTION
    if (ctl_dispatch_mtr_protect_slow(&protection))
    {
        cia402_fault_request(&cia402_sm);
    }
#endif // ENABLE_MOTOR_FAULT_PROTECTION

    return GMP_TASK_DONE;
}

//=================================================================================================
// CiA402 default callback routine

void ctl_enable_pwm()
{
#if defined ENABLE_GMP_DL_PIL_SIM
    clear_all_controllers();
#else
    ctl_fast_enable_output();
#endif
}

void ctl_disable_pwm()
{
    ctl_fast_disable_output();
}

void clear_all_controllers()
{
    ctl_clear_foc_core(&mtr_ctrl);
    ctl_clear_mech_ctrl(&mech_ctrl);
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
    // ADC Auto calibrate
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
                uuvw.bias[index_adc_calibrator - 3] =
                    uuvw.bias[index_adc_calibrator - 3] +
                    ctl_div(ctl_get_adc_calibrator_result(&adc_calibrator), uuvw.gain[index_adc_calibrator - 3]);

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
/** @brief Provide current-loop signals to the platform Scope. */
void user_get_scope_channels(ctrl_gt channels[4])
{
    channels[0] = spwm.vabc_out.dat[phase_A];
    channels[1] = spwm.vabc_out.dat[phase_B];
    channels[2] = spwm.vabc_out.dat[phase_C];
    channels[3] = mtr_ctrl.idq0.dat[phase_q];
}
#endif
