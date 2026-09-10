/** @file user_dl.h Minimal GMP Data Link service for Nucleo-64 bring-up. */

#ifndef GMP_STM32_NUCLEO_64_USER_DL_H
#define GMP_STM32_NUCLEO_64_USER_DL_H

#include <core/pm/function_scheduler/function_scheduler.h>

void user_dl_init(void);
gmp_task_status_t user_dl_task(gmp_task_t* task);

#endif // GMP_STM32_NUCLEO_64_USER_DL_H
