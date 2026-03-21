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

// a local small cache size, capable of covering the depth of the hardware FIFO (typically 16 bytes)
#define ISR_LOCAL_BUF_SIZE 16
uint8_t rxBuf[ISR_LOCAL_BUF_SIZE];
volatile uint16_t last_read_pos = 0;

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
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
		
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

	

	
void at_device_flush_rx_buffer(void)
{
// 1. 获取当前 DMA 写指针位置
    // __HAL_DMA_GET_COUNTER 返回的是 DMA 还需要传输的数据量（剩余量）
    // 因此当前写入的绝对位置 = 总长度 - 剩余量
    uint16_t current_pos = ISR_LOCAL_BUF_SIZE - __HAL_DMA_GET_COUNTER(debug_uart->hdmarx);

    // 2. 临界区保护开始：防止主循环执行到一半时被中断抢占导致逻辑混乱
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // 如果没有新数据，直接退出
    if (current_pos == last_read_pos) {
        __set_PRIMASK(primask);
        return;
    }

    uint16_t len1 = 0, len2 = 0;
    uint16_t start1 = last_read_pos, start2 = 0;

    // 3. 计算需要读取的数据长度（处理环形缓冲区的折返）
    if (current_pos > last_read_pos) {
        // 正常线性增长
        len1 = current_pos - last_read_pos;
    } else {
        // DMA 指针已经折返 (Wraparound) 到缓冲区开头
        // 第一段：从上次读取位置到缓冲区末尾
        len1 = ISR_LOCAL_BUF_SIZE - last_read_pos;
        // 第二段：从缓冲区开头到当前写指针位置
        len2 = current_pos;
    }

    // 更新读指针
    last_read_pos = current_pos;

    // 4. 临界区保护结束：恢复中断
    // 注意：我们将耗时的 AT 串口提交流程放在开中断之后执行，尽量缩短关中断的时间
    __set_PRIMASK(primask);

    // 5. 提交数据给 AT 控制器
    if (len1 > 0) {
        at_device_rx_isr(&at_dev, (char*)&rxBuf[start1], len1);
    }
    if (len2 > 0) { // 只有在发生折返时，len2 才会大于 0
        at_device_rx_isr(&at_dev, (char*)&rxBuf[start2], len2);
    }
}

// DMA 接收半满回调
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) { // 替换为你的实际串口
        at_device_flush_rx_buffer();
    }
}

// DMA 接收全满（完成）回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) { // 替换为你的实际串口
        at_device_flush_rx_buffer();
    }
}
