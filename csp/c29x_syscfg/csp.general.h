/**
 * @file csp.general.h
 * @brief TI F29H85x DriverLib binding used by GMP applications.
 */

#ifndef GMP_C29X_CSP_GENERAL_H
#define GMP_C29X_CSP_GENERAL_H

#include "driverlib.h"
#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GMP_DBG_SWBP ESTOP0

extern uart_halt debug_uart;

GMP_STATIC_INLINE void gmp_base_enter_critical(void)
{
    DINT;
}

GMP_STATIC_INLINE void gmp_base_leave_critical(void)
{
    EINT;
}

GMP_STATIC_INLINE void gmp_hal_wd_feed(void)
{
}

size_gt gmp_base_print_c29xsyscfg(const char *format, ...);
void gmp_step_system_tick(void);
void gmp_c29x_set_tick_divider(uint32_t calls_per_tick);

#ifdef __cplusplus
}
#endif

#endif /* GMP_C29X_CSP_GENERAL_H */
