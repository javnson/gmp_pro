/**
 * @file gmp_stm32_freertos.h
 * @brief Configuration contract for the STM32 FreeRTOS GMP service task.
 */

#ifndef _FILE_GMP_STM32_FREERTOS_H_
#define _FILE_GMP_STM32_FREERTOS_H_

#include <stdint.h>

// Period of the cooperative GMP executor. The executor performs one bounded
// gmp_base_loop() iteration per wake-up.
#ifndef GMP_FREERTOS_SERVICE_PERIOD_MS
#define GMP_FREERTOS_SERVICE_PERIOD_MS (1U)
#endif

#if GMP_FREERTOS_SERVICE_PERIOD_MS == 0
#error "GMP_FREERTOS_SERVICE_PERIOD_MS must be greater than zero"
#endif

/** Observable count of GMP service-loop iterations. */
extern volatile uint32_t gmp_rtos_runtime_iterations;

#endif // _FILE_GMP_STM32_FREERTOS_H_
