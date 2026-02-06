/**
 * @file ctl_main.cpp
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <xplt.peripheral.h>

//=================================================================================================
// include Necessary control modules

#include <ctl/component/interface/adc_channel.h>
#include <ctl/component/interface/pwm_channel.h>

#include <ctl/component/motor_control/basic/encoder.h>

#include <ctl/component/motor_control/basic/vf_generator.h>

#include <ctl/component/motor_control/current_loop/motor_current_ctrl.h>

#include <ctl/component/motor_control/motion/vel_pos_loop.h>

//#include <ctl/component/digital_power/three_phase/three_phase_GFL.h>

#include <ctl/component/interface/pwm_modulator.h>

#include <ctl/framework/cia402_state_machine.h>

#ifndef _FILE_CTL_MAIN_H_
#define _FILE_CTL_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// controller modules with extern

extern volatile fast_gt flag_system_running;

extern adc_bias_calibrator_t adc_calibrator;
extern volatile fast_gt flag_enable_adc_calibrator;
extern volatile fast_gt index_adc_calibrator;

// state machine
extern cia402_sm_t cia402_sm;

// modulator: SPWM modulator / SVPWM modulator / NPC modulator
#if defined USING_NPC_MODULATOR
extern npc_modulator_t spwm;
#else
extern spwm_modulator_t spwm;
#endif // USING_NPC_MODULATOR

// controller body: Current controller, Command dispatcher, motion controller
extern mtr_current_ctrl_t mtr_ctrl;
extern vel_pos_ctrl_t motion_ctrl;

// Observer: SMO, FO, Speed measurement.
extern ctl_slope_f_pu_controller rg;
extern pos_autoturn_encoder_t pos_enc;
extern spd_calculator_t spd_enc;

// additional controller: harmonic management

//=================================================================================================
// function prototype
void clear_all_controllers();

//=================================================================================================
// controller process

// periodic callback function things.
GMP_STATIC_INLINE void ctl_dispatch(void)
{
    // ADC calibrator routine
    if (flag_enable_adc_calibrator)
    {
        if (index_adc_calibrator == 7)
            ctl_step_adc_calibrator(&adc_calibrator, idc.control_port.value);
        else if (index_adc_calibrator == 6)
            ctl_step_adc_calibrator(&adc_calibrator, udc.control_port.value);
        else if (index_adc_calibrator <= 5 && index_adc_calibrator >= 3)
            ctl_step_adc_calibrator(&adc_calibrator, uuvw.control_port.value.dat[index_adc_calibrator - 3]);
        else if (index_adc_calibrator <= 2)
            ctl_step_adc_calibrator(&adc_calibrator, iuvw.control_port.value.dat[index_adc_calibrator]);
    }

    // normal controller routine
    else
    {        
        // ramp generator
        ctl_step_slope_f_pu(&rg);

        // Step auto turn pos encoder
        ctl_step_autoturn_pos_encoder(&pos_enc, EQEP_getPosition(EQEP_Encoder_BASE));

        ctl_step_spd_calc(&spd_enc);

#if BUILD_LEVEL > 3
        // motion controller
        ctl_step_vel_pos_ctrl(&motion_ctrl);

        // current command dispatch
        ctl_set_mtr_current_ctrl_ref(&mtr_ctrl, 0, ctl_get_vel_pos_cmd(&motion_ctrl));
        
#endif

        // motor current controller
        ctl_step_current_controller(&mtr_ctrl);

        // mix all output
        spwm.vab0_out.dat[phase_U] = mtr_ctrl.vab0.dat[phase_U];
        spwm.vab0_out.dat[phase_V] = mtr_ctrl.vab0.dat[phase_V];
        spwm.vab0_out.dat[phase_W] = mtr_ctrl.vab0.dat[phase_W];

        // modulation
#if defined USING_NPC_MODULATOR
        ctl_step_npc_modulator(&spwm);
#else
        ctl_step_spwm_modulator(&spwm);
#endif // USING_NPC_MODULATOR

    }
}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_
