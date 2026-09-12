/** @file user_dl.h GMP Data Link validation service for Nucleo-32. */

#ifndef GMP_STM32_NUCLEO_32_USER_DL_H
#define GMP_STM32_NUCLEO_32_USER_DL_H

#include <core/pm/function_scheduler/function_scheduler.h>

void user_dl_init(void);
gmp_task_status_t user_dl_task(gmp_task_t* task);
void user_dl_control_step(void);

#endif // GMP_STM32_NUCLEO_32_USER_DL_H
