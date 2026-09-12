/**
 * @file rtos_app.c
 * @brief RTOS-owned task creation for the NUCLEO-H753ZI reference project.
 */

#include "rtos_app.h"

#include <gmp_core.h>

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"

#define GMP_RTOS_SERVICE_STACK_WORDS (1024U)
#define GMP_RTOS_USER_STACK_WORDS    (256U)
#define GMP_RTOS_SERVICE_PRIORITY    (configMAX_PRIORITIES - 2U)
#define GMP_RTOS_USER_PRIORITY       (tskIDLE_PRIORITY + 1U)

volatile uint32_t gmp_rtos_user_task_heartbeats;

static void gmp_rtos_user_task(void* argument)
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
    BaseType_t result;

    gmp_rtos_user_task_heartbeats = 0U;
    result = xTaskCreate(gmp_rtos_runtime_task, "gmp", 
                         GMP_RTOS_SERVICE_STACK_WORDS, NULL,
                         GMP_RTOS_SERVICE_PRIORITY, NULL);
    if (result != pdPASS)
        Error_Handler();

    result = xTaskCreate(gmp_rtos_user_task, "user",
                         GMP_RTOS_USER_STACK_WORDS, NULL,
                         GMP_RTOS_USER_PRIORITY, NULL);
    if (result != pdPASS)
        Error_Handler();

    vTaskStartScheduler();
    Error_Handler();
}

void vApplicationMallocFailedHook(void)
{
    Error_Handler();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char* task_name)
{
    GMP_UNUSED_VAR(task);
    GMP_UNUSED_VAR(task_name);
    Error_Handler();
}
