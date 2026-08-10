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
extern rotation_ift acim_field_pos_if;
extern velocity_ift acim_field_spd_if;
extern fast_gt acim_sensorless_handover;
extern fast_gt acim_sensorless_observer_released;
extern fast_gt acim_sensorless_fault_latched;
extern uint32_t acim_sensorless_handover_ticks;
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
                            float2ctrl(MCS_OPEN_LOOP_VD_REF_V / CTRL_VOLTAGE_BASE),
                            ctl_mul(acim_open_loop_vq_per_freq_pu, rg.current_freq_pu));
#endif

#if BUILD_LEVEL >= 3
#if MCS_ACIM_FEEDBACK_MODE == MCS_ACIM_FEEDBACK_SENSORED
    (void)ctl_step_im_pos_calc(&acim_pos_calc, mtr_ctrl.idq0.dat[phase_d],
                               mtr_ctrl.idq0.dat[phase_q], spd_enc.encif.speed);
    acim_sync_speed_pu = acim_pos_calc.sync_spd_out.speed;
    acim_magnetizing_current_pu = acim_pos_calc.i_md_pu;
#else
    if (acim_sensorless_handover)
    {
        ctrl_gt run_speed_err = acim_fo.sync_spd_out.speed - rg.current_freq_pu;
        if (run_speed_err < float2ctrl(0.0f)) run_speed_err = -run_speed_err;

        if (!acim_fo.flag_observer_locked ||
            (run_speed_err > float2ctrl(MCS_FO_LOSS_SPEED_ERR_PU)))
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
                acim_sensorless_handover_ticks = 0;
                acim_sensorless_loss_ticks = 0;
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
        ctrl_gt speed_err = acim_fo.sync_spd_out.speed - rg.current_freq_pu;
        ctrl_gt acquire_freq_pu = float2ctrl(MCS_FO_ACQUIRE_FREQ_HZ / MOTOR_PARAM_RATED_FREQUENCY);
        ctrl_gt handover_freq_pu = float2ctrl(MCS_FO_HANDOVER_FREQ_HZ / MOTOR_PARAM_RATED_FREQUENCY);

        if (angle_err > CTL_CTRL_CONST_1_OVER_2) angle_err -= CTL_CTRL_CONST_1;
        if (angle_err < -CTL_CTRL_CONST_1_OVER_2) angle_err += CTL_CTRL_CONST_1;
        if (angle_err < float2ctrl(0.0f)) angle_err = -angle_err;
        if (speed_err < float2ctrl(0.0f)) speed_err = -speed_err;

        acim_field_pos_if.elec_position = rg.enc.elec_position;
        acim_field_spd_if.speed = rg.current_freq_pu;

        if (acim_sensorless_fault_latched || (rg.current_freq_pu < acquire_freq_pu))
        {
            /* Below the observable region, initialize rather than evaluate the
             * PLL.  The actual release occurs after the observer step below. */
            acim_sensorless_observer_released = 0;
            acim_sensorless_handover_ticks = 0;
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
            (angle_err <= float2ctrl(MCS_FO_HANDOVER_ANGLE_ERR_PU)) &&
            (speed_err <= float2ctrl(MCS_FO_HANDOVER_SPEED_ERR_PU)))
        {
            if (acim_sensorless_handover_ticks <
                (uint32_t)(MCS_FO_HANDOVER_DEBOUNCE_MS * CONTROLLER_FREQUENCY / 1000.0f))
                ++acim_sensorless_handover_ticks;
            else
            {
                acim_sensorless_handover = 1;
                ctl_set_im_fo_compensation_bw(&acim_fo, MCS_FO_RUN_COMP_BW_HZ,
                                               acim_pu.W_base, CONTROLLER_FREQUENCY);
                ctl_enable_im_fo_compensation(&acim_fo);
            }
        }
        else
        {
            acim_sensorless_handover_ticks = 0;
        }
    }
    if (acim_sensorless_handover)
    {
        acim_field_pos_if.elec_position = acim_fo.pos_out.elec_position;
        acim_field_spd_if.speed = acim_fo.sync_spd_out.speed;
    }

    acim_sync_speed_pu = acim_field_spd_if.speed;
    acim_magnetizing_current_pu =
        ctl_mul(acim_fo.psi_r_cm_mag, float2ctrl(acim_pu.L_s_base / MOTOR_PARAM_LM));
#endif
    ctl_set_im_ifoc_magnetizing_current(&mtr_ctrl, acim_magnetizing_current_pu);

    if (acim_magnetizing_ticks < (uint32_t)(MCS_MAGNETIZING_TIME_MS * CONTROLLER_FREQUENCY / 1000.0f))
    {
        ++acim_magnetizing_ticks;
        ctl_clear_mech_ctrl(&mech_ctrl);
        ctl_set_im_ifoc_ref(&mtr_ctrl, float2ctrl(MCS_COMMISSIONING_ID_REF_A / CTRL_CURRENT_BASE),
                            float2ctrl(0.0f));
    }
    else
    {
#if BUILD_LEVEL == 4
        ctl_step_mech_ctrl(&mech_ctrl);
        ctl_set_im_ifoc_ref(&mtr_ctrl, float2ctrl(MCS_COMMISSIONING_ID_REF_A / CTRL_CURRENT_BASE),
                            ctl_get_mech_cmd(&mech_ctrl));
#else
        ctl_set_im_ifoc_ref(&mtr_ctrl, float2ctrl(MCS_COMMISSIONING_ID_REF_A / CTRL_CURRENT_BASE),
                            float2ctrl(MCS_COMMISSIONING_IQ_REF_A / CTRL_CURRENT_BASE));
#endif
    }
#endif

    ctl_step_im_ifoc(&mtr_ctrl);

#if (BUILD_LEVEL >= 3) && (MCS_ACIM_FEEDBACK_MODE == MCS_ACIM_FEEDBACK_SENSORLESS)
    {
        /* `mtr_ctrl.udc` is normalized by CTRL_VOLTAGE_BASE.  The remaining
         * command-to-phase factor is target/modulator specific.  The supplied
         * centered-duty bridge uses 1/2; hardware dead-time and device-drop
         * compensation may require a separately identified value. */
        ctrl_gt voltage_scale =
            ctl_mul(float2ctrl(MCS_FO_COMMAND_VOLTAGE_SCALE), mtr_ctrl.udc);
        ctl_step_im_fo_with_field_angle(
            &acim_fo, ctl_mul(voltage_scale, mtr_ctrl.vab0.dat[phase_alpha]),
            ctl_mul(voltage_scale, mtr_ctrl.vab0.dat[phase_beta]),
            mtr_ctrl.iab0.dat[phase_alpha], mtr_ctrl.iab0.dat[phase_beta],
            acim_field_pos_if.elec_position);
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
