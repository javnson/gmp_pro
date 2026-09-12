/**
 * @file gmp_c28x_freertos.c
 * @brief TI C28x FreeRTOS ownership adapter for the GMP background executor.
 */

#include <gmp_core.h>

#if SPECIFY_GMP_OS_BACKEND != GMP_OS_BACKEND_FREERTOS
#error "gmp_c28x_freertos.c requires the FreeRTOS GMP backend"
#endif

#include <FreeRTOS.h>
#include <task.h>

#include "c2000_freertos.h"
#include <xplt.peripheral.h>

volatile uint32_t gmp_rtos_runtime_iterations;
volatile uint32_t gmp_rtos_user_task_heartbeats;
volatile uint16_t gmp_rtos_boot_state;

void gmp_rtos_runtime_task(void* argument)
{
    TickType_t service_delay = pdMS_TO_TICKS(GMP_FREERTOS_SERVICE_PERIOD_MS);

    GMP_UNUSED_VAR(argument);
    if (service_delay == 0U)
        service_delay = 1U;

    gmp_rtos_boot_state = 1U;
    gmp_base_prepare();
    gmp_rtos_boot_state = 2U;
    gmp_base_activate();
    xplt_rtos_activate();
    gmp_rtos_boot_state = 3U;

    for (;;)
    {
        gmp_rtos_runtime_iterations++;
        gmp_base_loop();
        /* GMP polling does not require deadline catch-up. A relative delay
         * avoids starving user tasks after a deliberately long DL response. */
        vTaskDelay(service_delay);
    }
}

void gmp_rtos_user_task(void* argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    GMP_UNUSED_VAR(argument);
    for (;;)
    {
        gmp_rtos_user_task_heartbeats++;
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000U));
    }
}

void gmp_rtos_app_start(void)
{
    gmp_rtos_runtime_iterations = 0U;
    gmp_rtos_user_task_heartbeats = 0U;
    gmp_rtos_boot_state = 0U;
    xplt_rtos_pre_scheduler();
    FreeRTOS_init();
}
