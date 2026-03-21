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

#ifndef _FILE_CTL_INTERFACE_H_
#define _FILE_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//////////////////////////////////////////////////////////////////////////
// device related functions
// Controller interface
//

// peripheral handles

extern ADC_HandleTypeDef hadc1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

extern DAC_HandleTypeDef hdac1;

// Input Callback
GMP_STATIC_INLINE
void ctl_input_callback(void)
{

    // copy ADC data to raw buffer
//    udc_raw = adc2_res[MOTOR_UDC];

//    uabc_raw[phase_U] = adc2_res[MOTOR_UA];
//    uabc_raw[phase_V] = adc1_res[MOTOR_UB];
//    uabc_raw[phase_W] = adc1_res[MOTOR_UC];

//    iabc_raw[phase_U] = adc2_res[MOTOR_UA];
//    iabc_raw[phase_V] = adc1_res[MOTOR_UB];
//    iabc_raw[phase_W] = adc1_res[MOTOR_UC];
		
		// copy ADC injected data to raw buffer
		iuvw_src[phase_A] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);
		iuvw_src[phase_B] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2);
		iuvw_src[phase_C] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3);
		
		udc_src = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_4);
	
    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);

    // invoke position encoder routine.
    ctl_step_autoturn_pos_encoder(&pos_enc, __HAL_TIM_GET_COUNTER(&htim3));
    // ctl_step_as5048a_pos_encoder(&pos_enc);
}

// Output Callback
GMP_STATIC_INLINE
void ctl_output_callback(void)
{
    // write to compare
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, spwm.pwm_out[phase_U]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, spwm.pwm_out[phase_V]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, spwm.pwm_out[phase_W]);
	
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048 + 2048.0f * spwm.vabc_out.dat[phase_U]);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2048 + 2048.0f * mtr_ctrl.iab0.dat[phase_alpha]);
}

// Compare output enable reg mask CCER (CH1/CH1N, CH2/CH2N, CH3/CH3N)
#define TIM_CCER_MASK  (TIM_CCER_CC1E | TIM_CCER_CC1NE | \
                        TIM_CCER_CC2E | TIM_CCER_CC2NE | \
                        TIM_CCER_CC3E | TIM_CCER_CC3NE)

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE
void ctl_fast_enable_output()
{
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
		
		htim1.Instance->CCER |= TIM_CCER_MASK;
		
		// Enable Gate driver
		HAL_GPIO_WritePin(PWM_DISABLE_GPIO_Port, PWM_DISABLE_Pin, GPIO_PIN_SET);
		
}

// Disable Output
GMP_STATIC_INLINE
void ctl_fast_disable_output()
{
//		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
//		
//    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
//    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
//    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
		
		htim1.Instance->CCER &= ~TIM_CCER_MASK;
		
	  // Recover Timer
//		__HAL_TIM_ENABLE(&htim1);
		
		// Disable Gate Driver
		HAL_GPIO_WritePin(PWM_DISABLE_GPIO_Port, PWM_DISABLE_Pin, GPIO_PIN_RESET);
		
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
