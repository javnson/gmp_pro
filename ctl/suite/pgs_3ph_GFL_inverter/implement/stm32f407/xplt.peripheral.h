
//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all necessary GMP config macro in this file.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#ifndef _FILE_XPLT_PERIPHERAL_H_
#define _FILE_XPLT_PERIPHERAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <gmp_core.h>

// controller settings
#include "ctrl_settings.h"

// select ADC PTR interface
#include <ctl/component/interface/adc_ptr_channel.h>

// select default encoder
#include <ctl/component/motor_control/basic/encoder.h>

// select SIL package
#include <ctl/component/motor_control/basic/motor_universal_interface.h>

#include <ctl/component/motor_control/basic/std_sil_motor_interface.h>

// encoder
#include <ext/encoder/as5048/as5048a.h>


	// STM32 Peripheral
	extern ADC_HandleTypeDef hadc1;
	extern TIM_HandleTypeDef htim1;
	extern TIM_HandleTypeDef htim3;
	
	extern DAC_HandleTypeDef hdac;

//// ADC Sequence
//enum ADC1_ITEMS
//{
//    MOTOR_IB = 0,
//    MOTOR_UC,
//    MOTOR_UB,
//    ADC1_SEQ_SIZE
//};
//
//enum ADC2_ITEMS
//{
//    MOTOR_IA = 0,
//    MOTOR_IC,
//    MOTOR_UA,
//    MOTOR_UDC,
//    ADC2_SEQ_SIZE
//};

//// ADC DMA buffer
//extern uint32_t adc1_res[ADC1_SEQ_SIZE];
//extern uint32_t adc2_res[ADC2_SEQ_SIZE];
//
//// SIL standard port for Motor control
//extern tri_ptr_adc_channel_t uabc;
//extern tri_ptr_adc_channel_t iabc;
//
//extern ptr_adc_channel_t udc;
//extern ptr_adc_channel_t idc;
//
//// extern pos_autoturn_encoder_t pos_enc;
//extern pwm_tri_channel_t pwm_out;
//
//// raw data
//extern adc_gt uabc_raw[3];
//extern adc_gt iabc_raw[3];
//extern adc_gt udc_raw;
//extern adc_gt idc_raw;
//
//// Encoder Interface
//extern ext_as5048a_encoder_t pos_enc;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PERIPHERAL_H_
