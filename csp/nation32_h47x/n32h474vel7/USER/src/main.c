/**
 * @file main.c
 * @author N32cube
 */
 //!!!!!!!!!!!!!!!!NOTE!!!!!!!!!!!!!!!
 // Code cannot be added between /* NTFx CODE START xxxxx*/ and /* NTFx CODE END xxxxx*/
 
/* NTFx CODE START Include*/
#include "main.h"
#include <stdio.h>
#include <stdint.h>
/* NTFx CODE END Include*/

#include <gmp_core.h>

/**
 * @brief  Main program.
 */
int main(void)
{
    /* NTFx CODE START Config*/
    RCC_Configuration();
    NVIC_Configuration();
    GPIO_Configuration();
    USART_Configuration();
    FDCAN_Configuration();
    I2C_Configuration();
    DAC_Configuration();
    RTC_Configuration();
    ADC_Configuration();
    CORDIC_Configuration();
    CRC_Configuration();
    FMAC_Configuration();
    SHRTIM_Configuration();
    TIM_Configuration();
    DMA_Configuration();
    /* NTFx CODE END Config*/
	
	//
	// Call GMP STARTUP function here
	//
	gmp_base_entry();
    while(1)
    {

    }
}


