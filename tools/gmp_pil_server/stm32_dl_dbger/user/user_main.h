/**
 * @file user_main.h
 * @brief User-layer declarations for the C092 Data Link and DSA demo.
 */

#ifndef GMP_STM32_DL_DBGER_USER_MAIN_H
#define GMP_STM32_DL_DBGER_USER_MAIN_H

#include <core/pm/function_scheduler.h>

/** @brief Initialize user services and register scheduled tasks. */
void init(void);

/** @brief Dispatch one cooperative scheduler iteration. */
void mainloop(void);

/** @brief Initialize platform-owned peripherals used by the user layer. */
void setup_peripheral(void);

/** @brief Generate one sample and advance the DSA trigger/scope state. */
void user_dsa_timer_step(void);

#endif // GMP_STM32_DL_DBGER_USER_MAIN_H
