
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

//=================================================================================================
// definitions of peripheral

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern DAC_HandleTypeDef hdac1;

extern FDCAN_HandleTypeDef hfdcan1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

// ADC Sequence
enum ADC1_ITEMS
{
    MOTOR_IB = 0,
    MOTOR_UC,
    MOTOR_UB,
    ADC1_SEQ_SIZE
};

enum ADC2_ITEMS
{
    MOTOR_IA = 0,
    MOTOR_IC,
    MOTOR_UA,
    MOTOR_UDC,
    ADC2_SEQ_SIZE
};

// inverter side voltage feedback
extern tri_ptr_adc_channel_t uuvw;
extern adc_gt uuvw_src[3];

// inverter side current feedback
extern tri_ptr_adc_channel_t iuvw;
extern adc_gt iuvw_src[3];

// grid side voltage feedback
extern tri_ptr_adc_channel_t vabc;
extern adc_gt vabc_src[3];

// grid side current feedback
extern tri_ptr_adc_channel_t iabc;
extern adc_gt iabc_src[3];

// DC bus current & voltage feedback
extern ptr_adc_channel_t udc;
extern adc_gt udc_src;
extern ptr_adc_channel_t idc;
extern adc_gt idc_src;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PERIPHERAL_H_
