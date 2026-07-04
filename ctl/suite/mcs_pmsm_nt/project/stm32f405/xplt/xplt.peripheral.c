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

// GPIO port
gpio_model_stm32_t user_led_entity;
extern gpio_halt user_led;

// a local small cache size, capable of covering the depth of the hardware FIFO (typically 16 bytes)
#define ISR_LOCAL_BUF_SIZE 16

// DMA buffer for UART
uint8_t rxBuf[ISR_LOCAL_BUF_SIZE];

/////////////////////////////////////////////////////////////////////////
// peripheral setup function
//

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
    // Setup Debug Uart
    debug_uart = &huart2;

    gmp_base_print(TEXT_STRING("Hello World!\r\n"));

    user_led_entity.gpio_port = GPIOC;
		user_led_entity.gpio_pin = GPIO_PIN_13;
	  user_led = &user_led_entity;

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
    ctl_attach_foc_core_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &rg.enc, &spd_enc.encif);
#else  // BUILD_LEVEL
    ctl_attach_foc_core_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &pos_enc.encif, &spd_enc.encif);
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

    // Enable UART RX DMA
    HAL_UART_Receive_DMA(debug_uart, rxBuf, ISR_LOCAL_BUF_SIZE);
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
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc == &hadc1)
    {
        gmp_base_ctl_step();
        counter++;
        if (counter >= 1000)
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
    if (GPIO_Pin == GPIO_PIN_9)
    {
        __HAL_TIM_SET_COUNTER(&htim3, 0);
    }
}

void send_monitor_data(void)
{
}

//=================================================================================================
// Debug interface

extern gmp_datalink_t dl;



uart_halt dl_uart_channel;

void flush_dl_tx_buffer()
{
    // Send head
    gmp_hal_uart_write(debug_uart, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl), gmp_dev_dl_get_tx_hw_hdr_size(&dl), 10);

    // Send data body, if necessary
    if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0)
    {
        gmp_hal_uart_write(debug_uart, gmp_dev_dl_get_tx_hw_pld_ptr(&dl), gmp_dev_dl_get_tx_hw_pld_size(&dl), 10);
    }
}

void flush_dl_rx_buffer(void)
{
    static volatile uint16_t last_read_pos = 0;

    // Get current DMA position, __HAL_DMA_GET_COUNTER wil get remaining quantity
    uint16_t current_pos = ISR_LOCAL_BUF_SIZE - __HAL_DMA_GET_COUNTER(debug_uart->hdmarx);

    // No new data arrived, return directly
    if (current_pos == last_read_pos)
        return;

    // Stop IRQ, prevent interruptting reading progress
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t len1 = 0, len2 = 0;
    uint16_t start1 = last_read_pos, start2 = 0;

    // Calculate valid length of DMA
    if (current_pos > last_read_pos)
    {
        // No warparound scene
        len1 = current_pos - last_read_pos;
    }
    else
    {
        // Warparound scene, two section of length
        len1 = ISR_LOCAL_BUF_SIZE - last_read_pos;
        len2 = current_pos;
    }

    // update last position
    last_read_pos = current_pos;

    // recover IQR
    __set_PRIMASK(primask);

    // Lock-free ring queue pushed into the protocol stack (very fast, O(1))
    if (len1 > 0)
        gmp_dev_dl_push_str(&dl, (data_gt*)&rxBuf[start1], len1);

    if (len2 > 0)
        gmp_dev_dl_push_str(&dl, (data_gt*)&rxBuf[start2], len2);
}

// DMA half-full callback
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART2)
    {
        flush_dl_rx_buffer();
    }
}

// DMA full callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART2)
    {
        flush_dl_rx_buffer();
    }
}
