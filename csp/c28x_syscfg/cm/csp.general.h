/**
 * @file csp.general.h
 * @brief Runtime contract for a C2000 integrated Cortex-M core.
 */

#ifndef GMP_C28X_SYSCFG_CM_CSP_GENERAL_H
#define GMP_C28X_SYSCFG_CM_CSP_GENERAL_H

#include "cm.h"
#include "driverlib_cm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GMP_DBG_SWBP __asm(" BKPT #0")

GMP_STATIC_INLINE void gmp_base_enter_critical(void)
{
    (void)Interrupt_disableInProcessor();
}

GMP_STATIC_INLINE void gmp_base_leave_critical(void)
{
    (void)Interrupt_enableInProcessor();
}

GMP_STATIC_INLINE void gmp_hal_wd_feed(void)
{
}

/** Advance the GMP millisecond clock; normally called by SysTick. */
void gmp_c28x_syscfg_cm_step_tick(void);

/** Board/application hooks required by the generic CM CSP adapter. */
void gmp_c28x_syscfg_cm_device_init(void);
void gmp_c28x_syscfg_cm_device_loop(void);
void gmp_c28x_syscfg_cm_post_start(void);

#ifdef __cplusplus
}
#endif

#endif
