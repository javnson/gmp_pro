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

//=================================================================================================
// peripheral setup function

extern UART_HandleTypeDef huart2;


// User should setup all the peripheral in this function.
void setup_peripheral(void)
{

    // Setup Debug Uart
    debug_uart = &huart2;

    // Test print function
    gmp_base_print(TEXT_STRING("Hello World!\r\n"));
    //asm(" RPT #255 || NOP");




}



//=================================================================================================
// ADC Interrupt ISR and controller related function

#ifdef HAL_ADC_MODULE_ENABLED

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
    }
}

#endif // HAL_ADC_MODULE_ENABLED



//=================================================================================================
// communication functions and interrupt functions here

// a local small cache size, capable of covering the depth of the hardware FIFO (typically 16 bytes)
#define ISR_LOCAL_BUF_SIZE 16

void at_device_flush_rx_buffer()
{
//    uint16_t fifoLevel;
//		size_gt rx_len;
//    data_gt rxBuf[ISR_LOCAL_BUF_SIZE];

//    // Read all FIFO content
//    while ((fifoLevel = gmp_hal_uart_get_rx_available(debug_uart)) > 0)
//    {
//        // Get data
//				gmp_hal_uart_read(debug_uart, rxBuf, fifoLevel, 5, &rx_len);

//        // send to AT device
//        at_device_rx_isr(&at_dev, (char*)rxBuf, fifoLevel);
//    }
}


