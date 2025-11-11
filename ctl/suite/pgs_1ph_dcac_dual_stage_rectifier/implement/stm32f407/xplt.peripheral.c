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

ptr_adc_channel_t sinv_il;
ptr_adc_channel_t sinv_ig;
ptr_adc_channel_t sinv_uc;
ptr_adc_channel_t sinv_udc;

qpr_ctrl_t qpr_test;

//
// output
//
pwm_channel_t sinv_pwm_out[2];

// ADC result
uint32_t adc_results[8];

// ADC RESULT INDEX
typedef enum _tag_adc_result_nameplate
{
	ADC_RESULT_IL = 3,
	ADC_RESULT_IG = 1,
	ADC_RESULT_UC = 0,
	ADC_RESULT_UDC = 2

	//ADC_RESULT_IL = 0,
	//ADC_RESULT_IG = 1,
	//ADC_RESULT_UC = 2,
	//ADC_RESULT_UDC = 3
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

    //ctl_init_ptr_adc_channel(
    //    // ptr_adc object
    //    &sinv_il,
    //    // pointer to ADC raw data
    //    &adc_results[ADC_RESULT_IL],
    //    // ADC Channel settings.
    //    // iqn is valid only when ctrl_gt is a fixed point type.
    //    2, 0.5, 12, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &sinv_il,
        // pointer to ADC raw data
        &adc_results[ADC_RESULT_IL],
        // ADC Channel settings.
        // iqn is valid only when ctrl_gt is a fixed point type.
        2, 0.38, 12, 24);//0.844238

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &sinv_uc,
        // pointer to ADC raw data
        &adc_results[ADC_RESULT_UC],
        // ADC Channel settings.
        // iqn is valid only when ctrl_gt is a fixed point type.
        2.4646, 0.4, 12, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &sinv_ig,
        // pointer to ADC raw data
        &adc_results[ADC_RESULT_IG],
        // ADC Channel settings.
        // iqn is valid only when ctrl_gt is a fixed point type. 2
        2.2, 0.36, 12, 24);

        ctl_init_ptr_adc_channel(
        // ptr_adc object
        &sinv_udc,
        // pointer to ADC raw data
        &adc_results[ADC_RESULT_UDC],
        // ADC Channel settings.
        // iqn is valid only when ctrl_gt is a fixed point type.
        2.4646, 0.4, 12, 24);
	
	    ctl_init_pwm_channel(&sinv_pwm_out[0], 0, CONTROLLER_PWM_CMP_MAX);
    ctl_init_pwm_channel(&sinv_pwm_out[1], 0, CONTROLLER_PWM_CMP_MAX);


	// 
	// Step II Connect Standard Interface to controller
	//

	// Attach ADC channel to Boost Controller
//	ctl_attach_boost_ctrl_input(
//		// Boost controller
//		&boost_ctrl,
//		// output capacitor voltage
//		&uc.control_port,
//		// inductor current
//		&il.control_port,
//		// input voltage
//		&uin.control_port);

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
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	//	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	//	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	//	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
	//  HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_1);
	//	HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_2);
	//	HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_3);

	// Start ADC Conversion
	HAL_ADC_Start_DMA(&hadc1, adc_results, 6);

	//HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef * hdac, uint32_t Channel);
	
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

}

//////////////////////////////////////////////////////////////////////////
// interrupt functions and callback functions here

// ADC interrupt
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	//if (hadc == &hadc2)
	if(htim == &htim1)
	{
		gmp_base_ctl_step();
	}
}

