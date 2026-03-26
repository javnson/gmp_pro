/**
 * @file csp.general.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <csp.config.h>

//
// Instert a software breakpoint right here
// GMP library Debug Software Break Point Macro
// This instruction is valid in Cotex-M kernel chip.
//
#define GMP_DBG_SWBP __asm volatile("BKPT #0")

//////////////////////////////////////////////////////////////////////////
// Step II: Invoke all the STM32 general headers.
//

GMP_STATIC_INLINE void gmp_base_enter_critical()
{
    /* * 在 ARMv7-A 中，使用 CPSID i 指令屏蔽 IRQ
     * Xilinx 提供的宏定义封装了这些汇编指令
     */
    Xil_ExceptionDisable();
}

GMP_STATIC_INLINE void gmp_base_leave_critical()
{
    /* 使用 CPSIE i 指令使能 IRQ */
    Xil_ExceptionEnable();
}


extern uart_halt debug_uart;
