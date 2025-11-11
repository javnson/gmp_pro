//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all definitions of peripheral objects in this file.
//
// User should implement the peripheral objects initialization in setup_peripheral function.
//
// This file is platform-related.
//

// GMP basic core header
#include <gmp_core.h>

// user main header
#include "user_main.h"



//////////////////////////////////////////////////////////////////////////
// definitions of peripheral
//



//// ADC DMA buffer
//uint32_t adc1_res[ADC1_SEQ_SIZE] = {0};
//uint32_t adc2_res[ADC2_SEQ_SIZE] = {0};
//
//// SIL standard port for Motor control
//tri_ptr_adc_channel_t uabc;
//tri_ptr_adc_channel_t iabc;
//
//ptr_adc_channel_t udc;
//ptr_adc_channel_t idc;
//
////pos_autoturn_encoder_t pos_enc;
//
//pwm_tri_channel_t pwm_out;
//
//// raw data
//adc_gt uabc_raw[3];
//adc_gt iabc_raw[3];
//adc_gt udc_raw;
//adc_gt idc_raw;
//
//// Encoder Interface
//ext_as5048a_encoder_t pos_enc;


//
// Input
//
ptr_adc_channel_t uc;
ptr_adc_channel_t uin;
ptr_adc_channel_t il;

//
// output
//
pwm_channel_t pwm_out;

// ADC result
uint32_t adc_results[8];

// ADC RESULT INDEX
typedef enum _tag_adc_result_nameplate
{
	ADC_RESULT_UIN = 0,
	ADC_RESULT_UOUT = 1,
	ADC_RESULT_IL = 2
}adc_result_nameplate_t;


/////////////////////////////////////////////////////////////////////////
// peripheral setup function
//

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{

	//
	// Step 1 Init all the ADC & PWM standard ports
	//

	ctl_init_ptr_adc_channel(
		// ptr_adc object
		&il,
		// pointer to ADC raw data
		&adc_results[ADC_RESULT_IL],
		// ADC Channel settings.
		// iqn is valid only when ctrl_gt is a fixed point type.
		2, 0.5, 12, 24);

	ctl_init_ptr_adc_channel(
		// ptr_adc object
		&uin,
		// pointer to ADC raw data
		&adc_results[ADC_RESULT_UIN],
		// ADC Channel settings.
		// iqn is valid only when ctrl_gt is a fixed point type.
		2, 0.5, 12, 24);

	ctl_init_ptr_adc_channel(
		// ptr_adc object
		&uc,
		// pointer to ADC raw data
		&adc_results[ADC_RESULT_UOUT],
		// ADC Channel settings.
		// iqn is valid only when ctrl_gt is a fixed point type.
		2, 0.5, 12, 24);

	ctl_init_pwm_channel(&pwm_out, 0, CONTROLLER_PWM_CMP_MAX);

	// 
	// Step II Connect Standard Interface to controller
	//

	// Attach ADC channel to Boost Controller
	ctl_attach_boost_ctrl_input(
		// Boost controller
		&boost_ctrl,
		// output capacitor voltage
		&uc.control_port,
		// inductor current
		&il.control_port,
		// input voltage
		&uin.control_port);

	//
	// STEP III: Starup PWM generator and ADC sampling
	//

	// Start Timer 3
	HAL_TIM_Base_Start_IT(&htim3);

	//Start Timer 1

	HAL_TIM_Base_Start_IT(&htim1);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	//	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	//	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	//	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
	//  HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_1);
	//	HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_2);
	//	HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_3);

	// Start ADC Conversion
	HAL_ADC_Start_DMA(&hadc1, adc_results, 6);


	// 

//    // Setup Debug Uart
//    //debug_uart = debug_uart_BASE;
//
//    gmp_base_print(TEXT_STRING("Hello World!\r\n"));
//
//		HAL_Delay(1);
//	
//    // Init ADC channel
//    ctl_init_ptr_adc_channel(
//        // bind idc channel with idc address
//        &idc, &idc_raw,
//        // ADC gain, ADC bias
//        float2ctrl(MTR_CTRL_CURRENT_GAIN), float2ctrl(MTR_CTRL_CURRENT_BIAS),
//        // ADC resolution, IQN
//        12, 24);
//
//    ctl_init_tri_ptr_adc_channel(
//        // bind ibac channel with iabc address
//        &iabc, iabc_raw,
//        // ADC gain, ADC bias
//        float2ctrl(MTR_CTRL_CURRENT_GAIN), float2ctrl(MTR_CTRL_CURRENT_BIAS),
//        // ADC resolution, IQN
//        12, 24);
//
//    ctl_init_ptr_adc_channel(
//        // bind udc channel with udc address
//        &udc, &udc_raw,
//        // ADC gain, ADC bias
//        float2ctrl(MTR_CTRL_VOLTAGE_GAIN), float2ctrl(MTR_CTRL_VOLTAGE_BIAS),
//        // ADC resolution, IQN
//        12, 24);
//
//    ctl_init_tri_ptr_adc_channel(
//        // bind vbac channel with vabc address
//        &uabc, uabc_raw,
//        // ADC gain, ADC bias
//        float2ctrl(MTR_CTRL_VOLTAGE_GAIN), float2ctrl(MTR_CTRL_VOLTAGE_BIAS),
//        // ADC resolution, IQN
//        12, 24);
//
//    //ctl_init_autoturn_pos_encoder(&pos_enc, MOTOR_PARAM_POLE_PAIRS, ((uint32_t)1 << 14) - 1);
////    ctl_init_as5048a_pos_encoder(&pos_enc, MOTOR_PARAM_POLE_PAIRS, SPI_ENCODER_BASE, SPI_ENCODER_NCS);
//
//    // bind peripheral to motor controller
//    ctl_attach_mtr_adc_channels(&pmsm_ctrl.mtr_interface,
//                                // phase voltage & phase current
//                                &iabc.control_port, &uabc.control_port,
//                                // dc bus voltage & dc bus current
//                                &idc.control_port, &udc.control_port);
//
//    ctl_attach_mtr_position(&pmsm_ctrl.mtr_interface, &pos_enc.encif);
//
//    ctl_attach_pmsm_bare_output(&pmsm_ctrl, &pwm_out.raw);
//
//    // output channel
//    ctl_init_pwm_tri_channel(&pwm_out, 0, CONTROLLER_PWM_CMP_MAX);
//
//    // Enabel ADC DMA
//    HAL_ADC_Start_DMA(&hadc1, adc1_res, ADC1_SEQ_SIZE);
//    HAL_ADC_Start_DMA(&hadc2, adc2_res, ADC2_SEQ_SIZE);
//
//    // Enable PWM peripheral
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
//    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

}

//////////////////////////////////////////////////////////////////////////
// interrupt functions and callback functions here

// ADC interrupt
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//if (hadc == &hadc2)
	{
		gmp_base_ctl_step();
	}
}

