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

#include <ctl/component/digital_power/three_phase/three_phase_GFL.h>
#include <ctl/component/digital_power/three_phase/three_phase_additional.h>

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

// controller body: Current controller, Power controller / Voltage controller
extern gfl_inv_ctrl_init_t gfl_init;
extern gfl_inv_ctrl_t inv_ctrl;
extern gfl_pq_ctrl_t pq_ctrl;
extern inv_neg_ctrl_init_t gfl_neg_init;
extern inv_neg_ctrl_t neg_current_ctrl;

// Observer: PLL

extern ctl_triangle_wave_generator_t tri_wave;

// additional controller: harmonic management, negative current controller

//

//=================================================================================================
// controller process

// periodic callback function things.
GMP_STATIC_INLINE void ctl_dispatch(void)
{
    // ADC calibrator routine
    if (flag_enable_adc_calibrator)
    {
        if (index_adc_calibrator == 13)
            ctl_step_adc_calibrator(&adc_calibrator, idc.control_port.value);
        else if (index_adc_calibrator == 12)
            ctl_step_adc_calibrator(&adc_calibrator, udc.control_port.value);
        else if (index_adc_calibrator <= 11 && index_adc_calibrator >= 9)
            ctl_step_adc_calibrator(&adc_calibrator, uuvw.control_port.value.dat[index_adc_calibrator - 9]);
        else if (index_adc_calibrator <= 8 && index_adc_calibrator >= 6)
            ctl_step_adc_calibrator(&adc_calibrator, vabc.control_port.value.dat[index_adc_calibrator - 6]);
        else if (index_adc_calibrator <= 5 && index_adc_calibrator >= 3)
            ctl_step_adc_calibrator(&adc_calibrator, iabc.control_port.value.dat[index_adc_calibrator - 3]);
        else if (index_adc_calibrator <= 2)
            ctl_step_adc_calibrator(&adc_calibrator, iuvw.control_port.value.dat[index_adc_calibrator]);
    }

    // normal controller routine
    else
    {
        ctl_step_triangle_wave_generator(&tri_wave);
        inv_ctrl.idq_set.dat[phase_d] = tri_wave.output;

        // run controller body
        ctl_step_gfl_inv_ctrl(&inv_ctrl);
        ctl_step_neg_inv_ctrl(&neg_current_ctrl);

        ctl_step_gfl_pq(&pq_ctrl);

        if (pq_ctrl.flag_enable)
        {
            ctl_set_gfl_inv_current(&inv_ctrl, pq_ctrl.idq_set_out.dat[phase_d], pq_ctrl.idq_set_out.dat[phase_q]);
        }

        // mix all output
        spwm.vab0_out.dat[phase_A] = inv_ctrl.vab0_out.dat[phase_A] + neg_current_ctrl.vab_out.dat[phase_A];
        spwm.vab0_out.dat[phase_B] = inv_ctrl.vab0_out.dat[phase_B] + neg_current_ctrl.vab_out.dat[phase_B];
        spwm.vab0_out.dat[phase_0] = inv_ctrl.vab0_out.dat[phase_0];

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
