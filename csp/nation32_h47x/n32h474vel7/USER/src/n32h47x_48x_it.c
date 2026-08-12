/**
 * @file n32h47x_48x_it.c
 * @author N32cube
 */

 
/* NTFx CODE START */
#include "n32h47x_48x_it.h"
#include "n32h47x_48x.h"
/* NTFx CODE END */

#include <gmp_core.h>

/* NTFx CODE START */
extern __IO uint32_t mwTick;
/**
 * @brief  This function handles NMI exception.
 */
void NMI_Handler(void)
{
/* NTFx CODE END */

}
/* NTFx CODE START */
/**
 * @brief  This function handles Hard Fault exception.
 */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    /* NTFx CODE END */

    }
}
/* NTFx CODE START */
/**
 * @brief  This function handles Memory Manage exception.
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
/* NTFx CODE END */

    }
}
/* NTFx CODE START */
/**
 * @brief  This function handles Bus Fault exception.
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
/* NTFx CODE END */

    }
}
/* NTFx CODE START */
/**
 * @brief  This function handles Usage Fault exception.
 */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
/* NTFx CODE END */

    }
}
/* NTFx CODE START */
/**
 * @brief  This function handles SVCall exception.
 */
void SVC_Handler(void)
{
/* NTFx CODE END */

}
/* NTFx CODE START */
/**
 * @brief  This function handles Debug Monitor exception.
 */
void DebugMon_Handler(void)
{
/* NTFx CODE END */

}
/* NTFx CODE START */
/**
 * @brief  This function handles SysTick Handler.
 */
void SysTick_Handler(void)
{
    mwTick++;
/* NTFx CODE END */

    /* GMP shares the Cortex-M millisecond time base with N32Cube. */
    gmp_step_system_tick();

}
/* NTFx CODE START(GTIM10_IRQHandler)*/
/**
    * @brief  This function handles GTIM10_IRQHandler.
    */
void GTIM10_IRQHandler(void)
{
/* NTFx CODE END */
	
/* NTFx CODE START */
    if (TIM_GetIntStatus(GTIM10, TIM_INT_UPDATE))
    {
        /*clear IT flag*/
        TIM_ClrIntPendingBit(GTIM10, TIM_INT_UPDATE);
/* NTFx CODE END */

    }
    
/* NTFx CODE START */
}
/* NTFx CODE END(GTIM10_IRQHandler)*/

