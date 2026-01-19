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

// const F & VF controller
#include <ctl/component/motor_control/basic/vf_generator.h>

// speed encoder
//#include <ctl\component\motor_control\basic\encoder.h>


#include <ctl/component/interface/pwm_channel.h>

#include <ctl/component/interface/adc_channel.h>

#include <xplt.peripheral.h>

#include <ctl/component/intrinsic/discrete/signal_generator.h>

#include <ctl/component/digital_power/three_phase/three_phase_GFL.h>

#include <ctl/component/interface/pwm_modulator.h>



#include <ctl/framework/cia402_state_machine.h>


#ifndef _FILE_CTL_MAIN_H_
#define _FILE_CTL_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//extern volatile fast_gt flag_system_enable;
extern volatile fast_gt flag_system_running;

extern adc_bias_calibrator_t adc_calibrator;
extern volatile fast_gt flag_enable_adc_calibrator;
extern volatile fast_gt index_adc_calibrator;

// state machine
extern cia402_sm_t cia402_sm;

// modulator: SPWM modulator / SVPWM modulator / NPC modulator
extern spwm_modulator_t spwm;

// controller body: Current controller, Power controller / Voltage controller
extern gfl_inv_ctrl_init_t gfl_init;
extern gfl_inv_ctrl_t inv_ctrl;
extern gfl_pq_ctrl_t pq_ctrl;

// Observer: PLL


// additional controller: harmonic management, negative current controller

//


// periodic callback function things.
GMP_STATIC_INLINE
void ctl_dispatch(void)
{

#if defined SPECIFY_ENABLE_ADC_CALIBRATE
    if (flag_enable_adc_calibrator)
    {
        if (index_adc_calibrator == 13)
            ctl_step_adc_calibrator(&adc_calibrator,idc.control_port.value);
        else if (index_adc_calibrator == 12)
            ctl_step_adc_calibrator(&adc_calibrator,udc.control_port.value);
        else if (index_adc_calibrator <= 11 && index_adc_calibrator >= 9)
            ctl_step_adc_calibrator(&adc_calibrator,uuvw.control_port.value.dat[index_adc_calibrator - 9]);
        else if (index_adc_calibrator <= 8 && index_adc_calibrator >= 6)
            ctl_step_adc_calibrator(&adc_calibrator,vabc.control_port.value.dat[index_adc_calibrator - 6]);
        else if (index_adc_calibrator <= 5 && index_adc_calibrator >= 3)
            ctl_step_adc_calibrator(&adc_calibrator,iabc.control_port.value.dat[index_adc_calibrator - 3]);
        else if (index_adc_calibrator <= 2)
            ctl_step_adc_calibrator(&adc_calibrator,iuvw.control_port.value.dat[index_adc_calibrator]);
    }
    else
    {
#endif // SPECIFY_ENABLE_ADC_CALIBRATE


    // run controller body
    ctl_step_gfl_inv_ctrl(&inv_ctrl);

    ctl_step_gfl_pq(&pq_ctrl);
    if(pq_ctrl.flag_enable)
    {
        ctl_set_gfl_inv_current(&inv_ctrl, pq_ctrl.idq_set_out.dat[phase_d],pq_ctrl.idq_set_out.dat[phase_q] );
    }

    // mix all output
    spwm.vab0_out.dat[phase_U] = inv_ctrl.vab0_out.dat[phase_U];
    spwm.vab0_out.dat[phase_V] = inv_ctrl.vab0_out.dat[phase_V];
    spwm.vab0_out.dat[phase_W] = inv_ctrl.vab0_out.dat[phase_W];

    // modulation
    ctl_step_spwm_modulator(&spwm);

#if defined SPECIFY_ENABLE_ADC_CALIBRATE
    }
#endif //SPECIFY_ENABLE_ADC_CALIBRATE
}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_
