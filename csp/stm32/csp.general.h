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

#if SPECIFY_GMP_OS_BACKEND == GMP_OS_BACKEND_FREERTOS
#include <FreeRTOS.h>
#include <task.h>
#endif

//
// Instert a software breakpoint right here
// GMP library Debug Software Break Point Macro
// This instruction is valid in Cotex-M kernel chip.
//
#define GMP_DBG_SWBP __asm volatile("BKPT #0")


/** @brief Enter a global interrupt critical section. */
GMP_STATIC_INLINE void gmp_base_enter_critical(void)
{
#if SPECIFY_GMP_OS_BACKEND == GMP_OS_BACKEND_FREERTOS
    taskENTER_CRITICAL();
#else
    __disable_irq();
#endif
}

/** @brief Leave a global interrupt critical section. */
GMP_STATIC_INLINE void gmp_base_leave_critical(void)
{
#if SPECIFY_GMP_OS_BACKEND == GMP_OS_BACKEND_FREERTOS
    taskEXIT_CRITICAL();
#else
    __enable_irq();
#endif
}

#if SPECIFY_GMP_OS_BACKEND == GMP_OS_BACKEND_FREERTOS
/** Saved interrupt mask used by explicit ISR-side critical sections. */
typedef UBaseType_t gmp_isr_critical_state_t;

GMP_STATIC_INLINE gmp_isr_critical_state_t
gmp_base_enter_critical_from_isr(void)
{
    return taskENTER_CRITICAL_FROM_ISR();
}

GMP_STATIC_INLINE void
gmp_base_leave_critical_from_isr(gmp_isr_critical_state_t state)
{
    taskEXIT_CRITICAL_FROM_ISR(state);
}
#endif

//////////////////////////////////////////////////////////////////////////
// Step II: Invoke all the STM32 general headers.
//

// STM32 System core support
#include <csp/stm32/common/sys_model.stm32.h>

// STM32 System Computing support
#include <csp/stm32/common/computing_model.stm32.h>

// STM32 GPIO support
#include <csp/stm32/common/gpio_model.stm32.h>

// STM32 general peripheral
//#include <csp/stm32/common/peripheral_model.stm32.h>

extern uart_halt debug_uart;
