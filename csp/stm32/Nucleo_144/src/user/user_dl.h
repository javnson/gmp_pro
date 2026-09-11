/** @file user_dl.h GMP Data Link validation service for Nucleo-144. */

#ifndef GMP_STM32_NUCLEO_144_USER_DL_H
#define GMP_STM32_NUCLEO_144_USER_DL_H

#include <core/pm/function_scheduler/function_scheduler.h>

#ifndef GMP_NUCLEO_ENABLE_UART_DL
#define GMP_NUCLEO_ENABLE_UART_DL 1
#endif
#ifndef GMP_NUCLEO_ENABLE_ETHERNET_DL
#define GMP_NUCLEO_ENABLE_ETHERNET_DL 1
#endif
#ifndef GMP_NUCLEO_ENABLE_CONTROL
#define GMP_NUCLEO_ENABLE_CONTROL 1
#endif
#ifndef GMP_NUCLEO_ENABLE_STATUS_LED
#define GMP_NUCLEO_ENABLE_STATUS_LED 1
#endif
#ifndef GMP_NUCLEO_DUAL_CORE
#define GMP_NUCLEO_DUAL_CORE 0
#endif

#define GMP_NUCLEO_DUAL_CORE_MAGIC (0x47373535UL)

typedef struct
{
    uint32_t magic;
    volatile uint32_t cm7_scheduler_heartbeats;
    volatile uint32_t cm4_scheduler_heartbeats;
    volatile uint32_t cm7_control_steps;
} gmp_nucleo_dual_core_status_t;

#if GMP_NUCLEO_DUAL_CORE
extern volatile gmp_nucleo_dual_core_status_t gmp_nucleo_dual_core_status;
void user_dl_dual_core_bootstrap(void);
#endif

void user_dl_init(void);
gmp_task_status_t user_dl_task(gmp_task_t* task);
void user_dl_control_step(void);
void user_dl_scheduler_step(void);

#endif // GMP_STM32_NUCLEO_144_USER_DL_H

