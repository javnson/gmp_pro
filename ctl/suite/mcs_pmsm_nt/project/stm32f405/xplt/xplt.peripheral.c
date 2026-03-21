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
#include <xplt.peripheral.h>


//////////////////////////////////////////////////////////////////////////
// definitions of peripheral
//

// ADC DMA buffer
uint32_t adc1_res[ADC1_SEQ_SIZE] = {0};
uint32_t adc2_res[ADC2_SEQ_SIZE] = {0};

// inverter side voltage feedback
tri_ptr_adc_channel_t uuvw;
adc_gt uuvw_src[3];

// inverter side current feedback
tri_ptr_adc_channel_t iuvw;
adc_gt iuvw_src[3];

// DC bus current & voltage feedback
ptr_adc_channel_t udc;
adc_gt udc_src;
ptr_adc_channel_t idc;
adc_gt idc_src;

// Encoder Interface
// ext_as5048a_encoder_t pos_enc;
uint32_t counter;

/////////////////////////////////////////////////////////////////////////
// peripheral setup function
//

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
    // Setup Debug Uart
    debug_uart = &huart2;

    gmp_base_print(TEXT_STRING("Hello World!\r\n"));

		HAL_Delay(1);
	

    // inverter side ADC
    ctl_init_tri_ptr_adc_channel(
        &uuvw, uuvw_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_tri_ptr_adc_channel(
        &iuvw, iuvw_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_ptr_adc_channel(
        &udc, &udc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_ptr_adc_channel(
        &idc, &idc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);


    //ctl_init_autoturn_pos_encoder(&pos_enc, MOTOR_PARAM_POLE_PAIRS, ((uint32_t)1 << 14) - 1);
//    ctl_init_as5048a_pos_encoder(&pos_enc, MOTOR_PARAM_POLE_PAIRS, SPI_ENCODER_BASE, SPI_ENCODER_NCS);

		// init TIM3 for QEP encoder
		HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    //
    // attach
    //
#if BUILD_LEVEL <= 2
    ctl_attach_mtr_current_ctrl_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &rg.enc, &spd_enc.encif);
#else  // BUILD_LEVEL
    ctl_attach_mtr_current_ctrl_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &pos_enc.encif, &spd_enc.encif);
#endif // BUILD_LEVEL


    // Enabel ADC DMA
    HAL_ADC_Start_DMA(&hadc1, adc1_res, ADC1_SEQ_SIZE);
    //HAL_ADC_Start_DMA(&hadc2, adc2_res, ADC2_SEQ_SIZE);

    HAL_ADCEx_InjectedStart_IT(&hadc1);

    // Enable PWM peripheral
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
		
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

		// Enable DAC channels
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		HAL_DAC_Start(&hdac, DAC_CHANNEL_2);



}

//////////////////////////////////////////////////////////////////////////
// interrupt functions and callback functions here

// ADC interrupt
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
// {
//     if (hadc == &hadc1)
//     {
//         gmp_base_ctl_step();
//     }
// }

/**
  * @brief  Injected conversion complete callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc == &hadc1)
    {
        gmp_base_ctl_step();
        counter++;
        if(counter >= 1000)   
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            counter = 0;
        }
    }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		// Index
    if(GPIO_Pin == GPIO_PIN_9)
    {
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        
    }
}

void send_monitor_data(void)
{}

	
	// a local small cache size, capable of covering the depth of the hardware FIFO (typically 16 bytes)
#define ISR_LOCAL_BUF_SIZE 16

void at_device_flush_rx_buffer(void)
{
    uint16_t fifoLevel;
    uint16_t rxBuf[ISR_LOCAL_BUF_SIZE];

//    // Read all FIFO content
//    while ((fifoLevel = SCI_getRxFIFOStatus(IRIS_UART_USB_BASE)) > 0)
//    {
//        // Get data
//        SCI_readCharArray(IRIS_UART_USB_BASE, rxBuf, fifoLevel);

//        // send to AT device
//        at_device_rx_isr(&at_dev, (char*)rxBuf, fifoLevel);
//    }
}
