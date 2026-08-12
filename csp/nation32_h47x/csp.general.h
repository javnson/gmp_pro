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

#ifndef _FILE_NATION32_H47X_CSP_GENERAL_H_
#define _FILE_NATION32_H47X_CSP_GENERAL_H_

#include <csp.config.h>

//
// Instert a software breakpoint right here
// GMP library Debug Software Break Point Macro
// This instruction is valid in Cotex-M kernel chip.
//
#define GMP_DBG_SWBP __BKPT(0)

/** Enter a global interrupt critical section. */
GMP_STATIC_INLINE void gmp_base_enter_critical(void)
{
    __disable_irq();
}

/** Leave a global interrupt critical section. */
GMP_STATIC_INLINE void gmp_base_leave_critical(void)
{
    __enable_irq();
}

//////////////////////////////////////////////////////////////////////////
// Step II: Invoke all the N32H47x general headers.
//

#include <csp/nation32_h47x/common/gpio_model.n32h47x.h>
#include <csp/nation32_h47x/common/sys_model.n32h47x.h>

extern uart_halt debug_uart;

size_gt gmp_base_print_n32(const char* p_fmt, ...);

#endif // _FILE_NATION32_H47X_CSP_GENERAL_H_
