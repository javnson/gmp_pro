/**
 * @file csp.general.h
 * @brief General Raspberry Pi Pico SDK integration for GMP.
 */

#ifndef GMP_CSP_RPI_PICO_GENERAL_H
#define GMP_CSP_RPI_PICO_GENERAL_H

#include <csp.config.h>
#include <hardware/sync.h>

#define GMP_DBG_SWBP __asm volatile("bkpt #0")

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Enter a core-local critical section while preserving the IRQ state.
 */
void gmp_base_enter_critical(void);

/**
 * @brief Leave a core-local critical section and restore the IRQ state.
 */
void gmp_base_leave_critical(void);

/** UART used by the default GMP diagnostic output implementation. */
extern uart_halt debug_uart;

#ifdef __cplusplus
}
#endif

#endif /* GMP_CSP_RPI_PICO_GENERAL_H */
