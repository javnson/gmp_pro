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
#include <ctl\component\motor_control\basic\encoder.h>

// motor controller
#include <ctl/suite/mcs_pmsm/pmsm_ctrl.h>

#include <ctl/component/interface/pwm_channel.h>

#include <ctl/component/interface/adc_channel.h>

#include <ctl\component\digital_power\basic/boost.h>

#include <xplt.peripheral.h>

#ifndef _FILE_CTL_MAIN_H_
#define _FILE_CTL_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//// speed encoder
// extern spd_calculator_t spd_enc;
//
//// PMSM servo objects
//// extern pmsm_fm_t pmsm;
//
// #if defined OPENLOOP_CONST_FREQUENCY
//
//// PMSM const frequency controller
// extern ctl_const_f_controller const_f;
//
// #else // OPENLOOP_CONST_FREQUENCY
//
//// PMSM const frequency slope controller
// extern ctl_slope_f_controller slope_f;
//
// #endif // OPENLOOP_CONST_FREQUENCY
//
// #ifdef PMSM_CTRL_USING_QEP_ENCODER
//// Auto - turn encoder
// extern pos_autoturn_encoder_t pos_enc;
// #endif // PMSM_CTRL_USING_QEP_ENCODER
//
//// PMSM controller
// extern pmsm_bare_controller_t pmsm_ctrl;

extern adc_bias_calibrator_t adc_calibrator;
extern fast_gt flag_enable_adc_calibrator;
extern fast_gt index_adc_calibrator;


// Boost Controller Suite
extern boost_ctrl_t boost_ctrl;

//extern pid_regular_t current_pid, voltage_pid;
//
//extern ctrl_gt pwm_out_pu;
//
//extern ptr_adc_channel_t uin;
//extern ptr_adc_channel_t uout;
//extern ptr_adc_channel_t idc;
//
//typedef enum _tag_adc_index
//{
//    MTR_ADC_UA = 0,
//    MTR_ADC_UB,
//    MTR_ADC_UC,
//    MTR_ADC_IA,
//    MTR_ADC_IB,
//    MTR_ADC_IC,
//    MTR_ADC_UDC,
//    MTR_ADC_IDC
//} adc_index_t;

// periodic callback function things.
GMP_STATIC_INLINE
void ctl_dispatch(void)
{
    //ctrl_gt current_ref = ctl_step_pid_ser(&voltage_pid, float2ctrl(0.8) - uout.control_port.value);
    //pwm_out_pu = float2ctrl(1) - ctl_step_pid_ser(&current_pid, current_ref - idc.control_port.value);


    ////pwm_out_pu = float2ctrl(1) - ctl_step_pid_ser(&current_pid, idc.control_port.value - current_ref);
    /// 
    
    ctl_step_boost_ctrl(&boost_ctrl);

}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_
