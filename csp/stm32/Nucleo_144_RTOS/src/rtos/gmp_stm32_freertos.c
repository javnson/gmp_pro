/**
 * @file gmp_stm32_freertos.c
 * @brief FreeRTOS-owned execution adapter for the GMP background loop.
 */

#include <gmp_core.h>

#if SPECIFY_GMP_OS_BACKEND != GMP_OS_BACKEND_FREERTOS
#error "gmp_stm32_freertos.c requires the FreeRTOS GMP backend"
#endif

#include <FreeRTOS.h>
#include <task.h>

#include "gmp_stm32_freertos.h"

volatile uint32_t gmp_rtos_runtime_iterations;

void gmp_rtos_runtime_task(void* argument)
{
    TickType_t last_wake_time;
    TickType_t period_ticks;

    GMP_UNUSED_VAR(argument);
    gmp_rtos_runtime_iterations = 0U;

    // The kernel is already running here. GMP initialization may configure and
    // enable the fast control ISR without racing an unstarted RTOS.
    gmp_base_prepare();
    gmp_base_activate();

    last_wake_time = xTaskGetTickCount();
    period_ticks = pdMS_TO_TICKS(GMP_FREERTOS_SERVICE_PERIOD_MS);
    if (period_ticks == 0U)
        period_ticks = 1U;

    for (;;)
    {
        gmp_rtos_runtime_iterations++;
        gmp_base_loop();
        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}
