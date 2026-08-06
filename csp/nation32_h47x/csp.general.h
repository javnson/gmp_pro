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

GMP_STATIC_INLINE void gmp_base_enter_critical()
{
    __disable_irq();
}

GMP_STATIC_INLINE void gmp_base_leave_critical()
{
    __enable_irq();
}

//////////////////////////////////////////////////////////////////////////
// Step II: Invoke all the STM32 general headers.
//

#include "n32h47x_48x.h"
#include "n32h47x_48x_cfg.h"

extern uart_halt debug_uart;

size_gt gmp_base_print_n32(const char* p_fmt, ...);

void gmp_step_system_tick(void);
