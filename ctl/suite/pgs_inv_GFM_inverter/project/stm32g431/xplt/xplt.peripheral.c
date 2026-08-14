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
#include <ctl/component/dsa/dsa_dl_scope.h>

#include "user_main.h"
#include <xplt.peripheral.h>


//=================================================================================================
// definitions of peripheral

// inverter side voltage feedback
tri_ptr_adc_channel_t uuvw;
adc_gt uuvw_src[3];

// inverter side current feedback
tri_ptr_adc_channel_t iuvw;
adc_gt iuvw_src[3];

// grid side voltage feedback
tri_ptr_adc_channel_t vabc;
adc_gt vabc_src[3];

// grid side current feedback
tri_ptr_adc_channel_t iabc;
adc_gt iabc_src[3];

// DC bus current & voltage feedback
ptr_adc_channel_t udc;
adc_gt udc_src;
ptr_adc_channel_t idc;
adc_gt idc_src;

// a local small cache size, capable of covering the depth of the hardware FIFO (typically 16 bytes)
#define UART_RX_DMA_BUFFER_SIZE 32
uint8_t uart_rx_dma_buffer[UART_RX_DMA_BUFFER_SIZE];

uint32_t counter;

//=================================================================================================
// peripheral setup function

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{

    // Setup Debug Uart
    debug_uart = &huart2;

    // Test print function
    gmp_base_print(TEXT_STRING("Hello World!\r\n"));

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

    // grid side ADC
    ctl_init_tri_ptr_adc_channel(
        &vabc, vabc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_tri_ptr_adc_channel(
        &iabc, iabc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_CURRENT_BIAS),
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

    //
    // attach
    //
    ctl_attach_gfl_inv(
        // inv controller
        &inv_ctrl,
        // idc, udc
        &idc.control_port, &udc.control_port,
        // grid side iabc, vabc
        &iabc.control_port, &vabc.control_port);

    //
    // Enable Peripherals
    //

    // Encoder

    // init TIM3 for QEP encoder
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    // init TIM4 for QEP encoder
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    // Enabel ADC DMA
    //    HAL_ADC_Start_DMA(&hadc1, adc1_res, ADC1_SEQ_SIZE);

    // Enable ADC Injected channel
    HAL_ADCEx_InjectedStart_IT(&hadc1);
		HAL_ADCEx_InjectedStart_IT(&hadc2);

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

    // Enable UART Receive DMA
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_dma_buffer, UART_RX_DMA_BUFFER_SIZE);
}

//=================================================================================================
// ADC Interrupt ISR and controller related function

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
        user_step_dl_scope();

        counter++;
        if (counter >= 1000)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            counter = 0;
        }
    }
}

////=================================================================================================
//// communication functions and interrupt functions here

//// 10000 -> 1.0
//#define CAN_SCALE_FACTOR 10000

//// 32 bit union
//typedef union {
//    int32_t i32;
//    uint16_t u16[2]; /**< C2000 stores each 16-bit value in one addressable word. */
//} can_data_t;

//// CAN interrupt
//interrupt void INT_IRIS_CAN_0_ISR(void)
//{
//    uint32_t status = CAN_getInterruptCause(IRIS_CAN_BASE);

//    uint16_t rx_data[4];
//    can_data_t recv_content[2];

//    if (status == 1)
//    {
//        CAN_readMessage(IRIS_CAN_BASE, 1, rx_data);
//        CAN_clearInterruptStatus(CANA_BASE, 1);

//        // Control Flag, Enable System
//        if (rx_data[0] == 1)
//        {
//            cia402_send_cmd(&cia402_sm, CIA402_CMD_ENABLE_OPERATION);
//        }
//        if (rx_data[0] == 0)
//        {
//            cia402_send_cmd(&cia402_sm, CIA402_CMD_DISABLE_VOLTAGE);
//        }
//    }
//    else if (status == 2)
//    {
//        CAN_readMessage(IRIS_CAN_BASE, 2, (uint16_t*)recv_content);
//        CAN_clearInterruptStatus(CANA_BASE, 2);

//        // set target value
//#if BUILD_LEVEL == 1
//        // For level 1 Set target voltage
//        ctl_set_gfl_inv_voltage_openloop(&inv_ctrl, real2ctrl((float)recv_content[0].i32 / CAN_SCALE_FACTOR),
//                                         real2ctrl((float)recv_content[1].i32 / CAN_SCALE_FACTOR));

//#endif // BUILD_LEVEL
//    }

//    //
//    // Clear the interrupt flag
//    //
//    CAN_clearGlobalInterruptStatus(IRIS_CAN_BASE, CAN_GLOBAL_INT_CANINT0);

//    //
//    // Acknowledge the interrupt
//    //
//    Interrupt_clearACKGroup(INT_IRIS_CAN_0_INTERRUPT_ACK_GROUP);
//}

void send_monitor_data(void)
{
//    uint16_t rx_raw[4];
//    can_data_t tran_content[2];

//    // 0x201: Monitor Grid Voltage
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

//    CAN_sendMessage(IRIS_CAN_BASE, 4, 8, (uint16_t*)tran_content);

//    //0x202: Monitor inverter voltage
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

//    CAN_sendMessage(IRIS_CAN_BASE, 5, 8, (uint16_t*)tran_content);

//    // 0x203: Monitor grid current
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

//    CAN_sendMessage(IRIS_CAN_BASE, 6, 8, (uint16_t*)tran_content);

//    // 0x204: TODO Monitor inverter current
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

//    CAN_sendMessage(IRIS_CAN_BASE, 7, 8, (uint16_t*)tran_content);

//    // 0x205: TODO Monitor DC Voltage / Current
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

//    CAN_sendMessage(IRIS_CAN_BASE, 8, 8, (uint16_t*)tran_content);

//    // 0x206: Monitor Grid Voltage A and PLL output angle
//    tran_content[0].i32 = (int32_t)(inv_ctrl.vabc.dat[phase_A] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.pll.theta * CAN_SCALE_FACTOR);

//    CAN_sendMessage(IRIS_CAN_BASE, 9, 8, (uint16_t*)tran_content);

//    // 0x207: Monitor reserved
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

//    CAN_sendMessage(IRIS_CAN_BASE, 10, 8, (uint16_t*)tran_content);
}

extern gmp_datalink_t dl;

/** @brief Transmit the pending Data Link frame through the debug UART. */
void flush_dl_tx_buffer(void)
{
    gmp_hal_uart_write(debug_uart, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl),
                       gmp_dev_dl_get_tx_hw_hdr_size(&dl), 10);
    if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0)
    {
        gmp_hal_uart_write(debug_uart, gmp_dev_dl_get_tx_hw_pld_ptr(&dl),
                           gmp_dev_dl_get_tx_hw_pld_size(&dl), 10);
    }
}

/** @brief Polling hook retained for the shared Data Link task. */
void flush_dl_rx_buffer(void)
{}

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == &huart2)
    {
        static uint16_t last_position = 0;
        HAL_UART_RxEventTypeTypeDef event = HAL_UARTEx_GetRxEventType(huart);

        if (Size > last_position)
        {
            gmp_dev_dl_push_str(&dl, (data_gt*)&uart_rx_dma_buffer[last_position],
                                (uint16_t)(Size - last_position));
            last_position = Size;
        }

        /** Idle and full events terminate normal-mode DMA and require rearming. */
        if (event != HAL_UART_RXEVENT_HT)
        {
            last_position = 0;
            HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_dma_buffer, UART_RX_DMA_BUFFER_SIZE);
        }
    }
}
