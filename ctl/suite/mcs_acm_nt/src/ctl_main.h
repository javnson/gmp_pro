/**
 * @file ctl_main.h
 * @brief BUILD_LEVEL fast-loop composition for the AC induction-motor suite.
 *
 * @warning Every angle in this file is turns-per-unit: 1 pu = 360 degrees =
 * 2*pi rad. The encoder supplies shaft information to the mechanical loop;
 * Park/iPark always uses the independently calculated rotor-flux field angle.
 */

#ifndef _FILE_MCS_ACIM_NT_CTL_MAIN_H_
#define _FILE_MCS_ACIM_NT_CTL_MAIN_H_

#include <xplt.peripheral.h>
#include <ctl/component/interface/spwm_modulator.h>
#include <ctl/component/motor_control/basic/mtr_protection.h>
#include <ctl/component/motor_control/basic/vf_generator.h>
#include <ctl/component/motor_control/current_loop/imfoc_core.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/component/motor_control/interface/sensorless_handover.h>
#include <ctl/component/motor_control/mechanical_loop/basic_mech_ctrl.h>
#include <ctl/component/motor_control/observer/acim_fo.h>
#include <ctl/component/motor_control/observer/acim_pos_calc.h>
#include <ctl/framework/cia402_state_machine.h>

#ifdef __cplusplus
extern "C" {
#endif

extern cia402_sm_t cia402_sm;
extern im_ifoc_ctrl_t mtr_ctrl;
extern ctl_mech_ctrl_t mech_ctrl;
extern ctl_consultant_im_t acim_motor;
extern ctl_consultant_pu_im_t acim_pu;
extern ctl_im_pos_calc_t acim_pos_calc;
extern ctl_im_fo_t acim_fo;
extern ctl_slope_f_pu_controller rg;
extern pos_autoturn_encoder_t pos_enc;
extern spd_calculator_t spd_enc;
extern spwm_modulator_t spwm;
extern ctl_mtr_protect_t protection;
extern ctrl_gt acim_sync_speed_pu;
extern ctrl_gt acim_magnetizing_current_pu;
extern ctrl_gt acim_open_loop_vq_per_freq_pu;
extern uint32_t acim_magnetizing_ticks;
extern velocity_ift acim_if_field_spd;
extern ctl_sensorless_handover_t acim_handover;
extern fast_gt acim_sensorless_handover;
extern fast_gt acim_sensorless_observer_released;
extern fast_gt acim_sensorless_fault_latched;
extern uint32_t acim_sensorless_loss_ticks;

void clear_all_controllers(void);

//=================================================================================================
// High-frequency controller dispatch

GMP_STATIC_INLINE void ctl_dispatch(void)
{
    ctl_step_slope_f_pu(&rg);
    ctl_step_spd_calc(&spd_enc);

#if BUILD_LEVEL == 1
    /* ACIM open-loop commissioning uses a V/f ramp. A fixed voltage at zero
     * frequency would over-flux the machine and can trip over-current before
     * phase order and PWM polarity have been checked. */
    ctl_set_im_ifoc_vdq_ref(&mtr_ctrl,
                            real2ctrl(MCS_OPEN_LOOP_VD_REF_V / CTRL_VOLTAGE_BASE),
                            ctl_mul(acim_open_loop_vq_per_freq_pu, rg.current_freq_pu));
#endif

#if BUILD_LEVEL >= 3
#if MCS_ACIM_FEEDBACK_MODE == MCS_ACIM_FEEDBACK_SENSORED
    (void)ctl_step_im_pos_calc(&acim_pos_calc, mtr_ctrl.idq0.dat[phase_d],
                               mtr_ctrl.idq0.dat[phase_q], spd_enc.encif.speed);
    acim_sync_speed_pu = acim_pos_calc.sync_spd_out.speed;
    acim_magnetizing_current_pu = acim_pos_calc.i_md_pu;
#else
    acim_if_field_spd.speed = rg.current_freq_pu;

    if (acim_sensorless_handover && !acim_sensorless_fault_latched)
    {
        ctrl_gt run_speed_err = acim_fo.sync_spd_out.speed - rg.current_freq_pu;
        if (run_speed_err < CTL_CTRL_CONST_ZERO) run_speed_err = -run_speed_err;

        if (!acim_fo.flag_observer_locked ||
            (run_speed_err > real2ctrl(MCS_FO_LOSS_SPEED_ERR_PU)))
        {
            if (acim_sensorless_loss_ticks <
                (uint32_t)(MCS_FO_LOSS_DEBOUNCE_MS * CONTROLLER_FREQUENCY / 1000.0f))
                ++acim_sensorless_loss_ticks;
            else
            {
                /* Latch a failed handover until the next controller clear so
                 * angle ownership can never chatter within one enable cycle. */
                acim_sensorless_handover = 0;
                acim_sensorless_observer_released = 0;
                acim_sensorless_fault_latched = 1;
                acim_sensorless_loss_ticks = 0;
                ctl_force_sensorless_handover(&acim_handover, 0);
                ctl_set_im_fo_compensation_bw(&acim_fo, MCS_FO_COMP_BW_HZ,
                                               acim_pu.W_base, CONTROLLER_FREQUENCY);
            }
        }
        else
        {
            acim_sensorless_loss_ticks = 0;
        }
    }

    /* The voltage model is not observable at standstill.  Startup therefore
     * has three explicit states: forced PLL initialization, free PLL
     * acquisition while I/F still owns the current-loop frame, and qualified
     * ownership transfer to the observer. */
    if (!acim_sensorless_handover)
    {
        ctrl_gt angle_err = acim_fo.pos_out.elec_position - rg.enc.elec_position;
        ctrl_gt acquire_freq_pu = real2ctrl(MCS_FO_ACQUIRE_FREQ_HZ / MOTOR_PARAM_RATED_FREQUENCY);
        ctrl_gt handover_freq_pu = real2ctrl(MCS_FO_HANDOVER_FREQ_HZ / MOTOR_PARAM_RATED_FREQUENCY);

        if (angle_err > CTL_CTRL_CONST_1_OVER_2) angle_err -= CTL_CTRL_CONST_1;
        if (angle_err < -CTL_CTRL_CONST_1_OVER_2) angle_err += CTL_CTRL_CONST_1;
        if (angle_err < CTL_CTRL_CONST_ZERO) angle_err = -angle_err;

        if (acim_sensorless_fault_latched || (rg.current_freq_pu < acquire_freq_pu))
        {
            /* Below the observable region, initialize rather than evaluate the
             * PLL.  The actual release occurs after the observer step below. */
            acim_sensorless_observer_released = 0;
            ctl_cancel_sensorless_handover_request(&acim_handover);
        }
        else
        {
            if (!acim_sensorless_observer_released)
            {
                /* Load one physically consistent integrator initial condition,
                 * then allow the voltage model and PLL to evolve freely. */
                ctl_seed_im_fo_voltage_model(&acim_fo);
            }
            acim_sensorless_observer_released = 1;
        }

        if (!acim_sensorless_fault_latched && acim_sensorless_observer_released &&
            acim_fo.flag_observer_locked &&
            (rg.current_freq_pu >= handover_freq_pu) &&
            (angle_err <= real2ctrl(MCS_FO_HANDOVER_ANGLE_ERR_PU)))
        {
            /* Speed matching, hysteresis and debounce are owned by the
             * reusable handover module. The suite contributes observer lock,
             * observability frequency and wrapped-angle qualification only. */
            if (!acim_handover.angle.request_pending)
                ctl_request_sensorless_handover(&acim_handover, 1);
        }
        else
        {
            ctl_cancel_sensorless_handover_request(&acim_handover);
        }
    }

    {
        angle_switch_state_e previous_state = acim_handover.angle.state;
        (void)ctl_step_sensorless_handover(&acim_handover, rg.current_freq_pu);
        acim_sensorless_handover =
            (acim_handover.angle.state != ANGLE_SWITCH_IDLE_A);
        if ((previous_state == ANGLE_SWITCH_IDLE_A) && acim_sensorless_handover)
        {
            ctl_set_im_fo_compensation_bw(&acim_fo, MCS_FO_RUN_COMP_BW_HZ,
                                           acim_pu.W_base, CONTROLLER_FREQUENCY);
            ctl_enable_im_fo_compensation(&acim_fo);
        }
    }

    acim_sync_speed_pu = acim_handover.angle.out_spd.speed;
    acim_magnetizing_current_pu =
        ctl_mul(acim_fo.psi_r_cm_mag, real2ctrl(acim_pu.L_s_base / MOTOR_PARAM_LM));
#endif
    ctl_set_im_ifoc_magnetizing_current(&mtr_ctrl, acim_magnetizing_current_pu);

    if (acim_magnetizing_ticks < (uint32_t)(MCS_MAGNETIZING_TIME_MS * CONTROLLER_FREQUENCY / 1000.0f))
    {
        ++acim_magnetizing_ticks;
        ctl_clear_mech_ctrl(&mech_ctrl);
        ctl_set_im_ifoc_ref(&mtr_ctrl,
#if MCS_ACIM_FEEDBACK_MODE == MCS_ACIM_FEEDBACK_SENSORLESS
                            real2ctrl(MCS_SENSORLESS_STARTUP_ID_REF_A / CTRL_CURRENT_BASE),
#else
                            real2ctrl(MCS_COMMISSIONING_ID_REF_A / CTRL_CURRENT_BASE),
#endif
                            CTL_CTRL_CONST_ZERO);
    }
    else
    {
#if BUILD_LEVEL == 4
        ctl_step_mech_ctrl(&mech_ctrl);
        ctl_set_im_ifoc_ref(&mtr_ctrl,
#if MCS_ACIM_FEEDBACK_MODE == MCS_ACIM_FEEDBACK_SENSORLESS
                            acim_handover.id_ref_out,
#else
                            real2ctrl(MCS_COMMISSIONING_ID_REF_A / CTRL_CURRENT_BASE),
#endif
                            ctl_get_mech_cmd(&mech_ctrl));
#else
        ctl_set_im_ifoc_ref(&mtr_ctrl,
#if MCS_ACIM_FEEDBACK_MODE == MCS_ACIM_FEEDBACK_SENSORLESS
                            acim_handover.id_ref_out,
#else
                            real2ctrl(MCS_COMMISSIONING_ID_REF_A / CTRL_CURRENT_BASE),
#endif
                            real2ctrl(MCS_COMMISSIONING_IQ_REF_A / CTRL_CURRENT_BASE));
#endif
    }
#endif

    ctl_step_im_ifoc(&mtr_ctrl);

#if (BUILD_LEVEL >= 3) && (MCS_ACIM_FEEDBACK_MODE == MCS_ACIM_FEEDBACK_SENSORLESS)
    {
        ctrl_gt observer_v_alpha;
        ctrl_gt observer_v_beta;
#if MCS_FO_VOLTAGE_SOURCE == MCS_FO_VOLTAGE_FROM_MEASUREMENT
        ctl_vector3_t measured_uab0;
        /* Direct phase-voltage feedback contains physical dead time, device
         * drop and DC-bus ripple. Its ADC offset/noise must be identified. */
        ctl_ct_clarke(&uuvw.control_port.value, &measured_uab0);
        observer_v_alpha = measured_uab0.dat[phase_alpha];
        observer_v_beta = measured_uab0.dat[phase_beta];
#else
        /* This is the current controller command before the modulator changes
         * compare values for dead-time compensation. `udc` is normalized by
         * CTRL_VOLTAGE_BASE; the centered-duty bridge contributes 1/2. */
        ctrl_gt voltage_scale =
            ctl_mul(real2ctrl(MCS_FO_COMMAND_VOLTAGE_SCALE), mtr_ctrl.udc);
        observer_v_alpha = ctl_mul(voltage_scale, mtr_ctrl.vab0.dat[phase_alpha]);
        observer_v_beta = ctl_mul(voltage_scale, mtr_ctrl.vab0.dat[phase_beta]);
#endif
        ctl_step_im_fo_with_field_angle(
            &acim_fo, observer_v_alpha, observer_v_beta,
            mtr_ctrl.iab0.dat[phase_alpha], mtr_ctrl.iab0.dat[phase_beta],
            acim_handover.angle.out_enc.elec_position);
        if (!acim_sensorless_observer_released)
        {
            /* Force only in the unobservable region.  Once released, the PLL
             * must run freely so the handover checks measure real acquisition
             * error rather than the value written by the startup ramp. */
            ctl_sync_ato_pll(&acim_fo.ato_pll, rg.enc.elec_position, rg.current_freq_pu);
            acim_fo.pos_out.elec_position = rg.enc.elec_position;
            acim_fo.sync_spd_out.speed = rg.current_freq_pu;
        }
    }
#endif

#ifdef ENABLE_MOTOR_FAULT_PROTECTION
    if (ctl_step_mtr_protect_fast(&protection)) cia402_fault_request(&cia402_sm);
#endif

    ctl_vector3_copy(&spwm.vab0_out, &mtr_ctrl.vab0);
    ctl_step_svpwm_modulator(&spwm);
}

#ifdef __cplusplus
}
#endif
#endif
