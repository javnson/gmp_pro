/**
 * @file FreeRTOSConfig.h
 * @brief Auditable FreeRTOS configuration for the GMP H753 reference.
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
#include "stm32h7xx.h"

extern uint32_t SystemCoreClock;

#define configUSE_PREEMPTION                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configUSE_TICKLESS_IDLE                   0
#define configCPU_CLOCK_HZ                        (SystemCoreClock)
#define configTICK_RATE_HZ                        ((TickType_t)1000U)
#define configMAX_PRIORITIES                      7
#define configMINIMAL_STACK_SIZE                  ((uint16_t)128U)
#define configMAX_TASK_NAME_LEN                   16
#define configUSE_16_BIT_TICKS                    0
#define configIDLE_SHOULD_YIELD                   1
#define configUSE_TASK_NOTIFICATIONS              1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES     1
#define configUSE_MUTEXES                         1
#define configUSE_RECURSIVE_MUTEXES               1
#define configUSE_COUNTING_SEMAPHORES              1
#define configQUEUE_REGISTRY_SIZE                 8
#define configUSE_QUEUE_SETS                      0
#define configUSE_TIME_SLICING                    1
#define configUSE_NEWLIB_REENTRANT                0
#define configENABLE_BACKWARD_COMPATIBILITY       1

#define configSUPPORT_STATIC_ALLOCATION           0
#define configSUPPORT_DYNAMIC_ALLOCATION          1
#define configTOTAL_HEAP_SIZE                     ((size_t)(32U * 1024U))
#define configAPPLICATION_ALLOCATED_HEAP          0

#define configUSE_IDLE_HOOK                       0
#define configUSE_TICK_HOOK                       0
#define configUSE_MALLOC_FAILED_HOOK              1
#define configCHECK_FOR_STACK_OVERFLOW            2

#define configUSE_CO_ROUTINES                     0
#define configMAX_CO_ROUTINE_PRIORITIES            2
#define configUSE_TIMERS                          0
#define configTIMER_TASK_PRIORITY                 2
#define configTIMER_QUEUE_LENGTH                  10
#define configTIMER_TASK_STACK_DEPTH              256

#define configUSE_TRACE_FACILITY                  1
#define configUSE_STATS_FORMATTING_FUNCTIONS      0
#define configGENERATE_RUN_TIME_STATS             0

#define INCLUDE_vTaskPrioritySet                  1
#define INCLUDE_uxTaskPriorityGet                 1
#define INCLUDE_vTaskDelete                       1
#define INCLUDE_vTaskSuspend                      1
#define INCLUDE_vTaskDelayUntil                   1
#define INCLUDE_vTaskDelay                        1
#define INCLUDE_xTaskGetSchedulerState            1
#define INCLUDE_xTaskGetIdleTaskHandle            0
#define INCLUDE_xTaskGetCurrentTaskHandle         0
#define INCLUDE_uxTaskGetStackHighWaterMark       1
#define INCLUDE_eTaskGetState                     1
#define INCLUDE_xTaskAbortDelay                   0
#define INCLUDE_xTaskGetHandle                    0

#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS                           __NVIC_PRIO_BITS
#else
#define configPRIO_BITS                           4
#endif
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
/*
 * Keep these as assembler-ready constants.  The ARMCC5/RVDS FreeRTOS port
 * expands configMAX_SYSCALL_INTERRUPT_PRIORITY inside an __asm function and
 * cannot evaluate a C shift expression there.  STM32H753 implements four
 * priority bits, so these values are identical to the expressions above.
 */
#define configKERNEL_INTERRUPT_PRIORITY          0xF0
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     0x50

/* The STM32 HAL tick uses TIM6. SysTick, PendSV and SVC belong to FreeRTOS. */
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler
#define xPortSysTickHandler                     SysTick_Handler

#define configASSERT(condition)                                             \
    do                                                                      \
    {                                                                       \
        if ((condition) == 0)                                               \
        {                                                                   \
            __disable_irq();                                                \
            for (;;)                                                        \
            {                                                               \
            }                                                               \
        }                                                                   \
    } while (0)

#endif // FREERTOS_CONFIG_H
