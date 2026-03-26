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

// 定义一个全局静态变量用于保存中断状态（支持嵌套需特殊处理）
static uint32_t _gmp_critical_status;

GMP_STATIC_INLINE void gmp_base_enter_critical()
{
    _gmp_critical_status = save_and_disable_interrupts();
}

GMP_STATIC_INLINE void gmp_base_leave_critical()
{
    restore_interrupts(_gmp_critical_status);
}

//////////////////////////////////////////////////////////////////////////
// Step II: Invoke all the RPI PICO general headers.
//



extern uart_halt debug_uart;
