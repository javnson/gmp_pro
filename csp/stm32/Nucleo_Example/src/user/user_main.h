/**
 * @file user_main.h
 * @brief Application scheduler declarations for the portable LaunchPad demo.
 */

#ifndef GMP_LAUNCHPAD_USER_MAIN_H
#define GMP_LAUNCHPAD_USER_MAIN_H

#include <core/pm/function_scheduler/function_scheduler.h>

/** @brief Initialize user services and register scheduled tasks. */
void init(void);

/** @brief Dispatch one cooperative scheduler iteration. */
void mainloop(void);

/** @brief Initialize platform-owned peripherals used by the user layer. */
void setup_peripheral(void);

/** Hardware acceptance counters exposed to the CCS Expressions view. */
extern volatile uint32_t startup_task_runs;
extern volatile uint32_t heartbeat_task_runs;
extern volatile uint32_t can_task_runs;

/** Debug-visible scheduler and task callbacks. */
extern gmp_scheduler_t scheduler;
extern gmp_task_t tasks[];
gmp_task_status_t user_task_startup_once(gmp_task_t* task);
gmp_task_status_t user_task_heartbeat(gmp_task_t* task);
gmp_task_status_t user_task_can(gmp_task_t* task);

#endif // GMP_LAUNCHPAD_USER_MAIN_H
