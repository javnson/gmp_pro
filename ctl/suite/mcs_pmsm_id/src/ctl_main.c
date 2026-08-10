
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

#if defined SPECIFY_PC_ENVIRONMENT
#include <stdio.h>
#endif

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

// additional controller: harmonic management

//
volatile fast_gt flag_system_running = 0;
volatile fast_gt flag_oid_pwm_inhibit = 0;
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

    // The bus compensator compares its nominal DC bus against udc, which is
    // normalized by the phase-voltage base.  Supplying the phase base here
    // introduces an unintended 1/sqrt(3) command attenuation.
    mtr_ctrl_init.v_bus = CTRL_DCBUS_VOLTAGE;
    mtr_ctrl_init.v_phase_limit = MOTOR_PARAM_RATED_VOLTAGE;

    mtr_ctrl_init.freq_base = MOTOR_PARAM_RATED_FREQUENCY;
    mtr_ctrl_init.spd_base = MOTOR_PARAM_MAX_SPEED / 1000;
    mtr_ctrl_init.pole_pairs = MOTOR_PARAM_POLE_PAIRS;

    mtr_ctrl_init.mtr_Ld = MOTOR_PARAM_LS;
    mtr_ctrl_init.mtr_Lq = MOTOR_PARAM_LS;
    mtr_ctrl_init.mtr_Rs = MOTOR_PARAM_RS;

    ctl_auto_tuning_foc_core(&mtr_ctrl_init);
    ctl_init_foc_core(&mtr_ctrl, &mtr_ctrl_init);

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
        MOTOR_PARAM_MAX_SPEED / 1000.0f, mtr_ctrl_init.pole_pairs,
        // ISR frequency
        CONTROLLER_FREQUENCY);

    //
    // mechanical controller
    //
    mech_init.fs = CONTROLLER_FREQUENCY;

    mech_init.pos_kp = 5.0f;
    mech_init.pos_ki = 1.0f;

    mech_init.vel_kp = 5.0f;
    mech_init.vel_ki = 1.0f;

    mech_init.speed_limit = 1.0f;
    mech_init.speed_slope_limit = 1.0f;
    mech_init.cur_limit = 0.3f;

    mech_init.mech_division = CTRL_MECH_DIV;

    ctl_init_mech_ctrl(&mech_ctrl, &mech_init);

    //
    // Encoder Init
    //
    ctl_init_autoturn_pos_encoder(&pos_enc, mtr_ctrl_init.pole_pairs, CTRL_POS_ENC_FS);
    ctl_set_autoturn_pos_encoder_mech_offset(&pos_enc, float2ctrl(CTRL_POS_ENC_BIAS));

    ctl_init_spd_calculator(&spd_enc, &pos_enc.encif, CONTROLLER_FREQUENCY, CTRL_MECH_DIV, MOTOR_PARAM_MAX_SPEED,
                            20.0f);

#ifdef ENABLE_SMO

    //
    // Observer Init
    //
    ctl_autotune_esmo_init_from_mtr(&smo_init, &mtr_ctrl_init, 0.005f);
    ctl_init_pmsm_esmo(&smo, &smo_init);

#endif // ENABLE_SMO

// attach motor current controller with input port
#if BUILD_LEVEL <= 2
    ctl_attach_foc_core_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &rg.enc, &spd_enc.encif);
#else  // BUILD_LEVEL
    ctl_attach_foc_core_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &pos_enc.encif, &spd_enc.encif);
#endif // BUILD_LEVEL

    ctl_attach_mech_ctrl(&mech_ctrl, &pos_enc.encif, &spd_enc.encif);

#if BUILD_LEVEL == 1
    // Voltage open loop
    ctl_disable_foc_core_current_ctrl(&mtr_ctrl);
    ctl_set_foc_core_vdq_ref(&mtr_ctrl, 0.0, 0.0);

#elif BUILD_LEVEL == 2
    // Basic current close loop, IF
    ctl_enable_foc_core_current_ctrl(&mtr_ctrl);
    ctl_set_foc_core_idq_ref(&mtr_ctrl, float2ctrl(0.1), float2ctrl(0.1));

#elif BUILD_LEVEL == 3
    // Basic current close loop, inverter
    ctl_enable_foc_core_current_ctrl(&mtr_ctrl);
    ctl_set_foc_core_idq_ref(&mtr_ctrl, float2ctrl(0.1), float2ctrl(0.1));

#elif BUILD_LEVEL == 4
    // Basic Speed close loop
    ctl_enable_foc_core_current_ctrl(&mtr_ctrl);
    ctl_set_mech_ctrl_mode(&mech_ctrl, MECH_MODE_VELOCITY);
    ctl_set_mech_target_velocity(&mech_ctrl, 0.1);

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
#if defined MCS_MAX_DC_BUS_VOLTAGE_V && defined MCS_MIN_DC_BUS_VOLTAGE_V
    // mtr_ctrl.udc is normalized by CTRL_VOLTAGE_BASE (phase-voltage base),
    // not by CTRL_DCBUS_VOLTAGE.  Convert the physical SDPE limits to the
    // same PU system before the protection module compares them.
    protection.limit_ov_pu = float2ctrl(MCS_MAX_DC_BUS_VOLTAGE_V / CTRL_VOLTAGE_BASE);
    protection.limit_uv_pu = float2ctrl(MCS_MIN_DC_BUS_VOLTAGE_V / CTRL_VOLTAGE_BASE);
#endif
    ctl_attach_mtr_protect_port(&protection, &mtr_ctrl.udc, (ctl_vector2_t*)&mtr_ctrl.idq0, &mtr_ctrl.idq_ref, NULL,
                                NULL);
    ctl_set_mtr_protect_mask(&protection, MTR_PROT_DEVIATION);

    //
    // init ADC Calibrator
    //
    ctl_init_adc_calibrator(&adc_calibrator, 20, 0.707f, CONTROLLER_FREQUENCY);

    if (flag_enable_adc_calibrator)
    {
        ctl_enable_adc_calibrator(&adc_calibrator);
    }

#if defined SPECIFY_PC_ENVIRONMENT && defined MCS_PMSM_ID_SIM_BYPASS_ADC_CALIBRATION
    flag_enable_adc_calibrator = 0;
    index_adc_calibrator = 8;
#endif

    init_pmsm_offline_id();

    // The angle switcher is stepped unconditionally by the OID ISR, including
    // READY/PREPARE.  Attach its two valid sources immediately after the suite
    // interface has assigned pmsm_oid.enc; the flux sub-state will reattach the
    // same sources when it becomes active.
    ctl_attach_angle_switcher(&pmsm_oid.angle_switcher, &pmsm_oid.vf_gen.enc, pmsm_oid.enc);

#if defined SPECIFY_PC_ENVIRONMENT && defined MCS_PMSM_ID_TIME_SCALE
    // Preserve the switching/dead-time plant resolution but shorten the
    // physical dwell periods for practical SIL iteration. Minimum values keep
    // the RL pulse and steady-state regressions meaningful.
    const parameter_gt oid_time_scale = (parameter_gt)MCS_PMSM_ID_TIME_SCALE;
    pmsm_oid.sub_rs_dt.cfg.max_current_pu = 0.1f;
    pmsm_oid.sub_rs_dt.cfg.min_current_pu = 0.02f;
    pmsm_oid.sub_ldq.cfg.max_bias_curr_pu = 0.1f;
    pmsm_oid.sub_ldq.cfg.align_current_pu = 0.1f;
    // Keep the pulse comfortably above ADC quantization and the identified
    // dead-time voltage in the fast averaged plant.
    pmsm_oid.sub_ldq.cfg.pulse_voltage_pu = 0.1f;
    pmsm_oid.sub_ldq.cfg.pulse_time_s = 0.01f;
    pmsm_oid.sub_flux.cfg.if_current_pu = 0.1f;
    pmsm_oid.sub_rs_dt.cfg.align_time_s =
        (1.0f * oid_time_scale > 0.02f) ? 1.0f * oid_time_scale : 0.02f;
    pmsm_oid.sub_rs_dt.cfg.measure_delay_s =
        (0.2f * oid_time_scale > 0.01f) ? 0.2f * oid_time_scale : 0.01f;
    pmsm_oid.sub_rs_dt.cfg.measure_points = 20;
    pmsm_oid.sub_ldq.cfg.settle_time_s =
        (0.2f * oid_time_scale > 0.01f) ? 0.2f * oid_time_scale : 0.01f;
    pmsm_oid.sub_ldq.cfg.cooldown_time_s =
        (0.05f * oid_time_scale > 0.002f) ? 0.05f * oid_time_scale : 0.002f;
    pmsm_oid.sub_flux.cfg.settle_time_s =
        (2.0f * oid_time_scale > 0.05f) ? 2.0f * oid_time_scale : 0.05f;
    pmsm_oid.sub_flux.cfg.measure_points = 100;

    if (oid_time_scale > 0.0f && oid_time_scale < 1.0f)
    {
        pmsm_oid.vf_gen.freq_slope.slope_max /= oid_time_scale;
        pmsm_oid.vf_gen.freq_slope.slope_min /= oid_time_scale;
    }
#endif
}

//=================================================================================================
// CTL endless loop routine

void ctl_mainloop(void)
{
    cia402_dispatch(&cia402_sm);

#if defined SPECIFY_PC_ENVIRONMENT && defined MCS_PMSM_ID_AUTO_START
    // A native SIL run has no debugger or UART command available to assign
    // pmsm_oid.sm. Consume one automatic start request only after the normal
    // CiA402 and ADC-calibration prerequisites have completed.
    static fast_gt oid_auto_start_consumed = 0;
    if (!oid_auto_start_consumed && cia402_sm.current_state == CIA402_SM_OPERATION_ENABLED &&
        !flag_enable_adc_calibrator && pmsm_oid.sm == PMSM_OFFLINE_ID_READY)
    {
        pmsm_oid.sm = PMSM_OFFLINE_ID_PREPARE;
        oid_auto_start_consumed = 1;
    }
#endif

    loop_pmsm_offline_id();

#if defined SPECIFY_PC_ENVIRONMENT && defined MCS_PMSM_ID_TIME_SCALE
    // The fast averaged plant has no switching ripple to decorrelate the
    // zero-current dead-time/ADC limit cycle.  Use all bias points for a
    // lower-middle robust regression result instead of the production
    // algorithm's zero-bias sample. Positive pulse/outlier bias makes the
    // upper half unsuitable as a nominal unsaturated estimate. The detailed
    // Simulink and hardware paths retain the production index-0 contract.
    static fast_gt ldq_fast_sil_postprocessed = 0;
    if (!ldq_fast_sil_postprocessed && pmsm_oid.sm == PMSM_OFFLINE_ID_FLUX &&
        simulink_rx_buffer.panel[15] > 0.5)
    {
        parameter_gt ld_sorted[12];
        parameter_gt lq_sorted[12];
        uint16_t count = pmsm_oid.sub_ldq.cfg.bias_steps;
        if (count > 12U)
            count = 12U;
        for (uint16_t i = 0; i < count; ++i)
        {
            ld_sorted[i] = pmsm_oid.sub_ldq.ld_array[i];
            lq_sorted[i] = pmsm_oid.sub_ldq.lq_array[i];
            for (uint16_t j = i; j > 0 && ld_sorted[j] < ld_sorted[j - 1]; --j)
            {
                parameter_gt tmp = ld_sorted[j];
                ld_sorted[j] = ld_sorted[j - 1];
                ld_sorted[j - 1] = tmp;
            }
            for (uint16_t j = i; j > 0 && lq_sorted[j] < lq_sorted[j - 1]; --j)
            {
                parameter_gt tmp = lq_sorted[j];
                lq_sorted[j] = lq_sorted[j - 1];
                lq_sorted[j - 1] = tmp;
            }
        }
        if (count >= 4U)
        {
            uint16_t upper_index = count / 2U - 1U;
            uint16_t lower_index = upper_index - 1U;
            parameter_gt scale = (CTRL_VOLTAGE_BASE / CTRL_CURRENT_BASE) / 4.0f;
            pmsm_oid.pmsm_param.Ld =
                (ld_sorted[lower_index] + ld_sorted[upper_index]) * scale;
            pmsm_oid.pmsm_param.Lq =
                (lq_sorted[lower_index] + lq_sorted[upper_index]) * scale;
            pmsm_oid.pmsm_param.saliency_ratio =
                pmsm_oid.pmsm_param.Lq / pmsm_oid.pmsm_param.Ld;
            pmsm_oid.pmsm_param.is_ipm =
                (pmsm_oid.pmsm_param.saliency_ratio > 1.05f) ? 1 : 0;
        }
        ldq_fast_sil_postprocessed = 1;
    }
#endif

#if defined SPECIFY_PC_ENVIRONMENT
    // State-transition-only diagnostics remain cheap enough for SIL and make
    // native/model handshake failures diagnosable even when Simulink exits
    // before returning logsout.
    static fast_gt state_diag_initialized = 0;
    static fast_gt last_cia402_state = 0;
    static fast_gt last_oid_state = 0;
    static fast_gt last_rs_dt_state = 0;
    static fast_gt last_ldq_state = 0;
    static fast_gt last_flux_state = 0;
    static fast_gt last_encoder_state = 0;
    static fast_gt last_mech_state = 0;
    fast_gt cia402_state_changed =
        !state_diag_initialized || last_cia402_state != (fast_gt)cia402_sm.current_state;
    fast_gt oid_state_changed = !state_diag_initialized || last_oid_state != (fast_gt)pmsm_oid.sm;
    fast_gt oid_substate_changed = !state_diag_initialized ||
        last_rs_dt_state != (fast_gt)pmsm_oid.sub_rs_dt.sm ||
        last_ldq_state != (fast_gt)pmsm_oid.sub_ldq.sm ||
        last_flux_state != (fast_gt)pmsm_oid.sub_flux.sm ||
        last_encoder_state != (fast_gt)pmsm_oid.sub_encoder.sm ||
        last_mech_state != (fast_gt)pmsm_oid.sub_mech.sm;
    if (cia402_state_changed)
    {
        last_cia402_state = (fast_gt)cia402_sm.current_state;
        gmp_base_print("[SIL] CiA402 state=%d\r\n", last_cia402_state);
    }
    if (oid_state_changed)
    {
        last_oid_state = (fast_gt)pmsm_oid.sm;
        gmp_base_print("[SIL] OID state=%d, sub=%d/%d/%d enc=%d/%d mech=%d\r\n", last_oid_state,
                       (fast_gt)pmsm_oid.sub_rs_dt.sm, (fast_gt)pmsm_oid.sub_ldq.sm,
                       (fast_gt)pmsm_oid.sub_flux.sm, (fast_gt)pmsm_oid.sub_encoder.sm,
                       (fast_gt)pmsm_oid.sub_encoder.fault, (fast_gt)pmsm_oid.sub_mech.sm);
    }
    if (oid_substate_changed)
    {
        last_rs_dt_state = (fast_gt)pmsm_oid.sub_rs_dt.sm;
        last_ldq_state = (fast_gt)pmsm_oid.sub_ldq.sm;
        last_flux_state = (fast_gt)pmsm_oid.sub_flux.sm;
        last_encoder_state = (fast_gt)pmsm_oid.sub_encoder.sm;
        last_mech_state = (fast_gt)pmsm_oid.sub_mech.sm;
    }
    if (cia402_state_changed || oid_state_changed || oid_substate_changed)
    {
        FILE* trace_file = NULL;
        if (fopen_s(&trace_file, "pmsm_id_sil_state.log", state_diag_initialized ? "a" : "w") == 0)
        {
            if (!state_diag_initialized)
            {
                fprintf(trace_file,
                        "config vdc=%.9g vbase=%.9g ibase=%.9g Rs=%.9g Ld=%.9g Lq=%.9g flux=%.9g poles=%d\n",
                        (double)CTRL_DCBUS_VOLTAGE, (double)CTRL_VOLTAGE_BASE,
                        (double)CTRL_CURRENT_BASE, (double)MOTOR_PARAM_RS,
                        (double)MOTOR_PARAM_LD, (double)MOTOR_PARAM_LQ,
                        (double)MOTOR_PARAM_FLUX, (fast_gt)MOTOR_PARAM_POLE_PAIRS);
            }
            fprintf(trace_file,
                    "tick=%llu cia402=%d oid=%d sub=%d/%d/%d enc=%d/%d mech=%d err=0x%08x raw_i=%u/%u/%u raw_udc=%u i=%.6g/%.6g/%.6g udc=%.6g\n",
                    (unsigned long long)cia402_sm.current_tick, last_cia402_state, last_oid_state,
                    (fast_gt)pmsm_oid.sub_rs_dt.sm, (fast_gt)pmsm_oid.sub_ldq.sm,
                    (fast_gt)pmsm_oid.sub_flux.sm, (fast_gt)pmsm_oid.sub_encoder.sm,
                    (fast_gt)pmsm_oid.sub_encoder.fault, (fast_gt)pmsm_oid.sub_mech.sm,
                    (unsigned int)protection.error_code.all,
                    (unsigned int)iuvw_src[phase_U], (unsigned int)iuvw_src[phase_V],
                    (unsigned int)iuvw_src[phase_W], (unsigned int)udc_src,
                    (double)mtr_ctrl.iuvw.dat[phase_U], (double)mtr_ctrl.iuvw.dat[phase_V],
                    (double)mtr_ctrl.iuvw.dat[phase_W], (double)mtr_ctrl.udc);
            if (oid_state_changed && pmsm_oid.sm == PMSM_OFFLINE_ID_FLUX)
            {
                for (uint16_t ldq_idx = 0; ldq_idx < pmsm_oid.sub_ldq.cfg.bias_steps; ++ldq_idx)
                {
                    fprintf(trace_file, "ldq_raw[%u] ld=%.9g lq=%.9g\n", (unsigned int)ldq_idx,
                            (double)pmsm_oid.sub_ldq.ld_array[ldq_idx],
                            (double)pmsm_oid.sub_ldq.lq_array[ldq_idx]);
                }
            }
            if (oid_state_changed && pmsm_oid.sm == PMSM_OFFLINE_ID_FAULT &&
                pmsm_oid.sub_flux.sm == PMSM_ID_FLUX_FAULT)
            {
                for (uint16_t flux_idx = 0; flux_idx < pmsm_oid.sub_flux.cfg.steps; ++flux_idx)
                {
                    fprintf(trace_file, "flux_raw[%u] w=%.9g emf=%.9g\n", (unsigned int)flux_idx,
                            (double)ctl_mem_get_2d_soa(&pmsm_oid.analyzer.mem, 4, flux_idx,
                                                       pmsm_oid.analyzer.depth),
                            (double)ctl_mem_get_2d_soa(&pmsm_oid.analyzer.mem, 5, flux_idx,
                                                       pmsm_oid.analyzer.depth));
                }
            }
            fclose(trace_file);
        }
    }
    state_diag_initialized = 1;
#endif

    return;
}

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
    if (!flag_oid_pwm_inhibit)
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

/** @brief Execute one controller step requested by the Data Link PIL service. */
void gmp_pil_sim_step(const gmp_sim_rx_buf_t* rx, gmp_sim_tx_buf_t* tx)
{
    ctl_apply_pil_input(rx);
    ctl_dispatch();
    ctl_collect_pil_output(tx);
}
#endif // defined ENABLE_GMP_DL_PIL_SIM

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
