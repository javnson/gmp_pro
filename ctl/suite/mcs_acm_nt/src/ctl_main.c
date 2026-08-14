/**
 * @file ctl_main.c
 * @brief ACIM suite object ownership, initialization, and background lifecycle.
 * @warning Rotor shaft angle/speed and rotor-flux field angle/synchronous speed
 * are different signals. The current core is attached only to the field pair.
 */

#include <gmp_core.h>
#include <core/dev/datalink/pil_core.h>
#include "ctl_settings_defaults.h"
#include "ctl_main.h"

//=================================================================================================
// Global controller objects

cia402_sm_t cia402_sm;
im_ifoc_ctrl_t mtr_ctrl;
ctl_mech_ctrl_t mech_ctrl;
static ctl_mech_init_t mech_init;
ctl_consultant_im_t acim_motor;
ctl_consultant_pu_im_t acim_pu;
ctl_im_pos_calc_t acim_pos_calc;
ctl_im_fo_t acim_fo;
ctl_slope_f_pu_controller rg;
pos_autoturn_encoder_t pos_enc;
spd_calculator_t spd_enc;
spwm_modulator_t spwm;
ctl_mtr_protect_t protection;
ctrl_gt acim_sync_speed_pu;
ctrl_gt acim_magnetizing_current_pu;
ctrl_gt acim_open_loop_vq_per_freq_pu;
uint32_t acim_magnetizing_ticks;
velocity_ift acim_if_field_spd;
ctl_sensorless_handover_t acim_handover;
fast_gt acim_sensorless_handover;
fast_gt acim_sensorless_observer_released;
fast_gt acim_sensorless_fault_latched;
uint32_t acim_sensorless_loss_ticks;

//=================================================================================================
// Controller initialization

void ctl_init(void)
{
    const parameter_gt ls = (parameter_gt)(MOTOR_PARAM_L1S + MOTOR_PARAM_LM);
    const parameter_gt lr = (parameter_gt)(MOTOR_PARAM_L1R + MOTOR_PARAM_LM);
    const parameter_gt omega_base = (parameter_gt)(CTL_PARAM_CONST_2PI * MOTOR_PARAM_RATED_FREQUENCY);

    ctl_fast_disable_output();
    ctl_consultant_im_init(&acim_motor, MOTOR_PARAM_POLE_PAIRS, (parameter_gt)MOTOR_PARAM_RS,
                           (parameter_gt)MOTOR_PARAM_RR,
                           ls, lr, (parameter_gt)MOTOR_PARAM_LM);
    ctl_consultant_pu_im_init(&acim_pu, (parameter_gt)CTRL_VOLTAGE_BASE,
                              (parameter_gt)CTRL_CURRENT_BASE, omega_base,
                              MOTOR_PARAM_POLE_PAIRS, 1.0f);

    ctl_autotune_and_init_im_ifoc_consultant(&mtr_ctrl, &acim_motor, &acim_pu, CONTROLLER_FREQUENCY,
                                             CTRL_DCBUS_VOLTAGE, MCS_MAX_CIR_SATURATION_VOLTAGE_V, 0.0f);
    ctl_set_im_ifoc_saturation(&mtr_ctrl, MCS_MAX_RECT_SATURATION_VOLTAGE_V / CTRL_VOLTAGE_BASE,
                               MCS_MAX_CIR_SATURATION_VOLTAGE_V / CTRL_VOLTAGE_BASE);
#ifndef ENABLE_ACIM_DECOUPLING
    ctl_disable_im_ifoc_decouple(&mtr_ctrl);
#endif

    ctl_init_spwm_modulator(&spwm, CTRL_PWM_CMP_MAX, CTRL_PWM_DEADBAND_CMP, &mtr_ctrl.iuvw,
                            real2ctrl(MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A / CTRL_CURRENT_BASE),
                            real2ctrl(MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A / CTRL_CURRENT_BASE));
#ifdef ENABLE_PWM_DEADTIME_COMPENSATION
    ctl_enable_spwm_deadtime_compensation(&spwm);
#endif
    ctl_init_const_slope_f_pu_controller(&rg, MCS_OPEN_LOOP_FREQ_HZ, MCS_OPEN_LOOP_FREQ_SLOPE_HZ_S,
                                         CTRL_SPEED_RPM_BASE / 1000.0f, MOTOR_PARAM_POLE_PAIRS,
                                         CONTROLLER_FREQUENCY);
    acim_open_loop_vq_per_freq_pu = real2ctrl(
        (MCS_OPEN_LOOP_VQ_REF_V / CTRL_VOLTAGE_BASE) /
        (MCS_OPEN_LOOP_FREQ_HZ / MOTOR_PARAM_RATED_FREQUENCY));
    ctl_init_autoturn_pos_encoder(&pos_enc, MOTOR_PARAM_POLE_PAIRS, CTRL_POS_ENC_FS);
    ctl_set_autoturn_pos_encoder_mech_offset(&pos_enc, real2ctrl(CTRL_POS_ENC_BIAS));
    ctl_init_spd_calculator(&spd_enc, &pos_enc.encif, CONTROLLER_FREQUENCY, CTRL_MECH_DIV,
                            CTRL_SPEED_RPM_BASE, MCS_ENCODER_SPEED_FILTER_FC_HZ);

    ctl_init_im_pos_calc_consultant(&acim_pos_calc, &acim_motor, &acim_pu, CONTROLLER_FREQUENCY);
    ctl_init_im_fo_consultant(&acim_fo, &acim_motor, &acim_pu, CONTROLLER_FREQUENCY,
                              MCS_FO_COMP_BW_HZ, MCS_FO_ATO_BW_HZ, MCS_FO_FAULT_TIME_MS);
    ctl_set_im_fo_voltage_model_leak(&acim_fo, MCS_FO_VM_LEAK_HZ, CONTROLLER_FREQUENCY);

    ctl_init_sensorless_handover(
        &acim_handover, MCS_FO_HANDOVER_TRANSITION_MS / 1000.0f, CONTROLLER_FREQUENCY,
        real2ctrl(MCS_SENSORLESS_STARTUP_ID_REF_A / CTRL_CURRENT_BASE),
        real2ctrl(MCS_SENSORLESS_HANDOVER_ID_REF_A / CTRL_CURRENT_BASE),
        real2ctrl(MCS_COMMISSIONING_ID_REF_A / CTRL_CURRENT_BASE),
        real2ctrl(MCS_STARTUP_ID_FADE_START_HZ / MOTOR_PARAM_RATED_FREQUENCY),
        real2ctrl(MCS_STARTUP_ID_FADE_END_HZ / MOTOR_PARAM_RATED_FREQUENCY));
    ctl_attach_sensorless_handover(&acim_handover, &rg.enc, &acim_if_field_spd,
                                   &acim_fo.pos_out, &acim_fo.sync_spd_out);
    ctl_configure_sensorless_handover_speed_qualification(
        &acim_handover, MCS_FO_HANDOVER_SPEED_ERR_PU,
        MCS_FO_HANDOVER_SPEED_EXIT_ERR_PU,
        MCS_FO_HANDOVER_DEBOUNCE_MS / 1000.0f, CONTROLLER_FREQUENCY);

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

#if BUILD_LEVEL <= 2
    ctl_disable_im_ifoc_decouple(&mtr_ctrl);
    ctl_attach_im_ifoc_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &rg.enc, NULL);
    ctl_attach_mech_ctrl(&mech_ctrl, &pos_enc.encif, &spd_enc.encif);
#elif MCS_ACIM_FEEDBACK_MODE == MCS_ACIM_FEEDBACK_SENSORED
    ctl_enable_im_pos_calc(&acim_pos_calc);
    ctl_attach_im_ifoc_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port,
                            &acim_pos_calc.enc_out, &acim_pos_calc.sync_spd_out);
    ctl_attach_mech_ctrl(&mech_ctrl, &pos_enc.encif, &spd_enc.encif);
#else
    ctl_enable_im_fo(&acim_fo);
    ctl_attach_im_ifoc_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port,
                            ctl_get_sensorless_handover_position(&acim_handover),
                            ctl_get_sensorless_handover_speed(&acim_handover));
    ctl_attach_mech_ctrl(&mech_ctrl, &acim_fo.rotor_pos_out, &acim_fo.spd_out);
#endif

#if BUILD_LEVEL == 1
    ctl_disable_im_ifoc(&mtr_ctrl);
    ctl_disable_im_ifoc_vdq_ff(&mtr_ctrl);
    ctl_set_im_ifoc_vdq_ref(&mtr_ctrl, CTL_CTRL_CONST_ZERO, CTL_CTRL_CONST_ZERO);
#elif BUILD_LEVEL == 2
    ctl_enable_im_ifoc(&mtr_ctrl);
    ctl_disable_im_ifoc_vdq_ff(&mtr_ctrl);
    ctl_set_im_ifoc_ref(&mtr_ctrl, real2ctrl(MCS_COMMISSIONING_ID_REF_A / CTRL_CURRENT_BASE),
                        real2ctrl(MCS_COMMISSIONING_IQ_REF_A / CTRL_CURRENT_BASE));
#else
    ctl_enable_im_ifoc(&mtr_ctrl);
#endif

#if BUILD_LEVEL == 4
    ctl_set_mech_ctrl_mode(&mech_ctrl, MECH_MODE_VELOCITY);
    ctl_set_mech_target_velocity(&mech_ctrl, real2ctrl(MCS_COMMISSIONING_SPEED_REF_RPM / CTRL_SPEED_RPM_BASE));
#endif

    init_cia402_state_machine(&cia402_sm);
#if defined SPECIFY_PC_ENVIRONMENT
    cia402_sm.minimum_transit_delay[0] = 0;
    cia402_sm.minimum_transit_delay[1] = 0;
    cia402_sm.minimum_transit_delay[2] = 0;
    cia402_sm.minimum_transit_delay[3] = MCS_CIA402_OPERATION_ENABLE_DELAY_MS;
    cia402_sm.flag_enable_control_word = 0;
    cia402_sm.current_cmd = CIA402_CMD_ENABLE_OPERATION;
#endif

    ctl_init_mtr_protect(&protection, CONTROLLER_FREQUENCY);
    ctl_set_mtr_protect_ov(&protection, real2ctrl(MCS_MAX_DC_BUS_VOLTAGE_V / CTRL_DCBUS_VOLTAGE));
    ctl_set_mtr_protect_uv(&protection, real2ctrl(MCS_MIN_DC_BUS_VOLTAGE_V / CTRL_DCBUS_VOLTAGE));
    ctl_set_mtr_protect_oc(&protection, real2ctrl(MCS_MAX_SHUTDOWN_CURRENT_A / CTRL_CURRENT_BASE));
    ctl_attach_mtr_protect_port(&protection, &mtr_ctrl.udc, (ctl_vector2_t*)&mtr_ctrl.idq0,
                                &mtr_ctrl.idq_ref, NULL, NULL);
    clear_all_controllers();
}

//=================================================================================================
// Background dispatch and reset lifecycle

void ctl_mainloop(void) { cia402_dispatch(&cia402_sm); }

void clear_all_controllers(void)
{
    ctl_clear_im_ifoc(&mtr_ctrl);
    ctl_clear_mech_ctrl(&mech_ctrl);
    ctl_clear_slope_f_pu(&rg);
    ctl_clear_im_pos_calc(&acim_pos_calc);
    ctl_clear_im_fo(&acim_fo);
    ctl_clear_spwm_modulator(&spwm);
    acim_sync_speed_pu = CTL_CTRL_CONST_ZERO;
    acim_magnetizing_current_pu = CTL_CTRL_CONST_ZERO;
    acim_magnetizing_ticks = 0;
    acim_if_field_spd.speed = CTL_CTRL_CONST_ZERO;
    ctl_clear_sensorless_handover(&acim_handover);
    acim_sensorless_handover = 0;
    acim_sensorless_observer_released = 0;
    acim_sensorless_fault_latched = 0;
    acim_sensorless_loss_ticks = 0;
}

void ctl_enable_pwm(void) { clear_all_controllers(); ctl_fast_enable_output(); }
void ctl_disable_pwm(void) { ctl_fast_disable_output(); }
fast_gt ctl_exec_adc_calibration(void) { return 1; }

//=================================================================================================
// PIL transport hook

#if defined ENABLE_GMP_DL_PIL_SIM
/** @brief Apply one SDPE-mapped ACIM PIL sample to the controller ports. */
static void ctl_apply_pil_input(const gmp_sim_rx_buf_t* rx)
{
    uuvw_src[phase_U] = rx->adc_result[GMP_PIL_RX_ADC_UU_INDEX];
    uuvw_src[phase_V] = rx->adc_result[GMP_PIL_RX_ADC_UV_INDEX];
    uuvw_src[phase_W] = rx->adc_result[GMP_PIL_RX_ADC_UW_INDEX];
    iuvw_src[phase_U] = rx->adc_result[GMP_PIL_RX_ADC_IU_INDEX];
    iuvw_src[phase_V] = rx->adc_result[GMP_PIL_RX_ADC_IV_INDEX];
    iuvw_src[phase_W] = rx->adc_result[GMP_PIL_RX_ADC_IW_INDEX];
    udc_src = rx->adc_result[GMP_PIL_RX_ADC_UDC_INDEX];

    ctl_step_autoturn_pos_encoder(&pos_enc, rx->digital_input);
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

/** @brief Export ACIM control results through the shared SIL/PIL ABI. */
static void ctl_collect_pil_output(gmp_sim_tx_buf_t* tx)
{
    tx->pwm_cmp[GMP_PIL_TX_PWM_U_INDEX] = spwm.pwm_out[phase_U];
    tx->pwm_cmp[GMP_PIL_TX_PWM_V_INDEX] = spwm.pwm_out[phase_V];
    tx->pwm_cmp[GMP_PIL_TX_PWM_W_INDEX] = spwm.pwm_out[phase_W];
    tx->monitor[GMP_PIL_TX_MONITOR_IU_INDEX] = mtr_ctrl.iuvw.dat[phase_U];
    tx->monitor[GMP_PIL_TX_MONITOR_IV_INDEX] = mtr_ctrl.iuvw.dat[phase_V];
    tx->monitor[GMP_PIL_TX_MONITOR_ID_INDEX] = mtr_ctrl.idq0.dat[phase_d];
    tx->monitor[GMP_PIL_TX_MONITOR_IQ_INDEX] = mtr_ctrl.idq0.dat[phase_q];
    tx->monitor[GMP_PIL_TX_MONITOR_POSITION_INDEX] = mtr_ctrl.field_pos_if->elec_position;
    tx->monitor[GMP_PIL_TX_MONITOR_SPEED_INDEX] = spd_enc.encif.speed;
}
#endif

void gmp_pil_sim_step(const gmp_sim_rx_buf_t* rx, gmp_sim_tx_buf_t* tx)
{
#if defined ENABLE_GMP_DL_PIL_SIM
    ctl_apply_pil_input(rx);
    ctl_dispatch();
    ctl_collect_pil_output(tx);
#else
    GMP_UNUSED_VAR(rx);
    GMP_UNUSED_VAR(tx);
#endif
}

#if defined ENABLE_GMP_DL_PIL_SIM
time_gt gmp_base_get_ctrl_tick(void)
{
    return mtr_ctrl.isr_tick / ((uint32_t)CONTROLLER_FREQUENCY / 1000U);
}
#endif

#if !defined SPECIFY_PC_ENVIRONMENT
/** @brief Expose field-oriented ACIM commissioning signals to Data Link Scope. */
void user_get_scope_channels(ctrl_gt channels[4])
{
    channels[0] = mtr_ctrl.idq_ref.dat[phase_d];
    channels[1] = mtr_ctrl.idq0.dat[phase_d];
    channels[2] = mtr_ctrl.idq_ref.dat[phase_q];
    channels[3] = mtr_ctrl.idq0.dat[phase_q];
}
#endif
