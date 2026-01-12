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

#include <ctl/component/motor_control/basic/std_sil_motor_interface.h>

#include <xplt.peripheral.h>

#include <ext/encoder/as5048/as5048a.h>

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

// extern ptr_adc_channel_t uc;
// extern ptr_adc_channel_t uin;
// extern ptr_adc_channel_t il;
//
//
// extern pwm_channel_t pwm_out;

extern ctrl_gt sinv_pwm_pu[2];
extern pwm_channel_t sinv_pwm_out[2];

extern ptr_adc_channel_t sinv_il;
extern ptr_adc_channel_t sinv_ig;
extern ptr_adc_channel_t sinv_uc;

extern ptr_adc_channel_t sinv_udc;

extern ctrl_gt modulate_target;

extern qpr_ctrl_t qpr_test;

// Functions without controller nano framework.
#ifndef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

// Input Callback
GMP_STATIC_INLINE
void ctl_input_callback(void)
{

    //
    // deal with all the ADC data
    //

    ctl_step_ptr_adc_channel(&sinv_il);
    ctl_step_ptr_adc_channel(&sinv_ig);
    ctl_step_ptr_adc_channel(&sinv_uc);
    ctl_step_ptr_adc_channel(&sinv_udc);
		
		HAL_GPIO_TogglePin(Mz_CS_GPIO_Port, Mz_CS_Pin);

}

// Output Callback
GMP_STATIC_INLINE
void ctl_output_callback(void)
{
    // ctl_calc_pwm_tri_channel(&pwm_out);

    //
    // Step I: Run PWM calculate
    //

    //

    //
    // Step II: Write PWM compare register
    //

    // write to compare
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ctl_calc_pwm_channel(&sinv_pwm_out[0], sinv_pwm_pu[0]));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ctl_calc_pwm_channel(&sinv_pwm_out[1], sinv_pwm_pu[1]));
		
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (ctl_mul(spll.phasor.dat[0], sinv_current_ref) + 1) * 3276/2);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (sinv_ig.control_port.value + 1) * 3276/2);
}

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE
void ctl_enable_output()
{
	ctl_clear_qpr_controller(&sinv_qpr_base);
	ctl_clear_pr_controller(&sinv_pr_base);
	ctl_clear_pid(&current_pid);
	ctl_clear_pid(&voltage_pid);
	ctl_clear_pid(&sinv_vlotage_pid);

    //        csp_sl_enable_output();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	
	flag_error = 0;
}

// Disable Output
GMP_STATIC_INLINE
void ctl_disable_output()
{
    //        csp_sl_disable_output();
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	flag_error = 1;
		
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
	
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

#endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
