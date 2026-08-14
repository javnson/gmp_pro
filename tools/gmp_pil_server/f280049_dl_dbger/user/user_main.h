/**
 * @file user_main.h
 * @brief User-layer declarations for the F280049C Data Link and DSA demo.
 */

#ifndef GMP_F280049_DL_DBGER_USER_MAIN_H
#define GMP_F280049_DL_DBGER_USER_MAIN_H

#include <core/pm/function_scheduler/function_scheduler.h>

/** @brief Initialize user services and register scheduled tasks. */
void init(void);

/** @brief Dispatch one cooperative scheduler iteration. */
void mainloop(void);

/** @brief Initialize platform-owned peripherals used by the user layer. */
void setup_peripheral(void);

/** @brief Generate one sample and advance the DSA trigger/scope state. */
void user_dsa_timer_step(void);

#endif // GMP_F280049_DL_DBGER_USER_MAIN_H
