//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all declarations of controller objects in this file.
//
// User should implement the Main ISR of the controller tasks.
//
// User should ensure that all the controller codes here is platform-independent.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#include <xplt.peripheral.h>

#include "main.h"

#ifndef _FILE_CTL_INTERFACE_H_
#define _FILE_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus



//=================================================================================================
// Controller interface

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback(void)
{
    // copy source ADC data
    vabc_src[phase_A] = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_1);
    vabc_src[phase_B] = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_2);
    vabc_src[phase_C] = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_3);

    iabc_src[phase_A] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);
    iabc_src[phase_B] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2);
    iabc_src[phase_C] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3);
		
//    uuvw_src[phase_U] = ADC_readResult(INV_UU_RESULT_BASE, INV_UU);
//    uuvw_src[phase_V] = ADC_readResult(INV_UV_RESULT_BASE, INV_UV);
//    uuvw_src[phase_W] = ADC_readResult(INV_UW_RESULT_BASE, INV_UW);

//    iuvw_src[phase_U] = ADC_readResult(INV_IU_RESULT_BASE, INV_IU);
//    iuvw_src[phase_V] = ADC_readResult(INV_IV_RESULT_BASE, INV_IV);
//    iuvw_src[phase_W] = ADC_readResult(INV_IW_RESULT_BASE, INV_IW);

    udc_src = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_4);
    idc_src = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_4);

    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iabc);
    ctl_step_tri_ptr_adc_channel(&vabc);
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback(void)
{
    // Write ePWM peripheral CMP
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, spwm.pwm_out[phase_U]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, spwm.pwm_out[phase_V]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, spwm.pwm_out[phase_W]);


    // Monitor Port
#if BUILD_LEVEL == 1

    // grid current and inverter current		
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, iabc.control_port.value.dat[phase_C] * 2048 + 2048);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, iuvw.control_port.value.dat[phase_C] * 2048 + 2048);


#endif // BUILD_LEVEL
}

// Compare output enable reg mask CCER (CH1/CH1N, CH2/CH2N, CH3/CH3N)
#define TIM_CCER_MASK  (TIM_CCER_CC1E | TIM_CCER_CC1NE | \
                        TIM_CCER_CC2E | TIM_CCER_CC2NE | \
                        TIM_CCER_CC3E | TIM_CCER_CC3NE)
												
// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE void ctl_fast_enable_output()
{
    // Clear any Trip Zone flag
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
		
		htim1.Instance->CCER |= TIM_CCER_MASK;

    ctl_enable_gfl_inv(&inv_ctrl);

    // PWM enable
		HAL_GPIO_WritePin(PWM_DISABLE_GPIO_Port, PWM_DISABLE_Pin, GPIO_PIN_SET);
		
//    GPIO_WritePin(PWM_ENABLE_PORT, 1);

//    GPIO_WritePin(PWM_RESET_PORT, 0);

//    GPIO_WritePin(CONTROLLER_LED, 0);
}

// Disable Output
GMP_STATIC_INLINE void ctl_fast_disable_output()
{
    // Disables the PWM device
    htim1.Instance->CCER &= ~TIM_CCER_MASK;
				
    ctl_disable_gfl_inv(&inv_ctrl);

    // PWM disable
		HAL_GPIO_WritePin(PWM_DISABLE_GPIO_Port, PWM_DISABLE_Pin, GPIO_PIN_RESET);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
