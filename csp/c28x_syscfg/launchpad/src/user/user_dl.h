/**
 * @file user_dl.h
 * @brief Data Link service boundary for the LaunchPad reference application.
 */

#ifndef GMP_LAUNCHPAD_USER_DL_H
#define GMP_LAUNCHPAD_USER_DL_H

#include <gmp_core.h>
#include <core/dev/datalink/datalink.h>
#include <core/pm/function_scheduler/function_scheduler.h>

/** @brief DSA acquisition states visible in the CCS Expressions view. */
typedef enum
{
    USER_DSA_WAITING = 0,
    USER_DSA_CAPTURING,
    USER_DSA_READY
} user_dsa_state_t;

/** @brief Initialize Data Link, tunable, memory, scope and optional PIL. */
void user_dl_init(void);

/** @brief Keep the UART receive FIFO shallow between scheduled protocol ticks. */
void user_dl_background(void);

/** @brief Execute one non-blocking Data Link protocol task. */
gmp_task_status_t user_dl_task(gmp_task_t* task);

/** @brief Apply user-visible signal-generator parameters. */
void user_dl_apply_signal_parameters(void);

/** @brief Arm a coherent trigger/scope capture. */
void user_dl_arm_dsa(void);

/** @brief Advance the 1 kHz Data Link scope signal source. */
void user_dl_dsa_timer_step(void);

/** Debug-visible Data Link and scope state. */
extern gmp_datalink_t datalink;
extern volatile uint32_t datalink_task_runs;
extern volatile user_dsa_state_t dsa_state;
extern volatile uint32_t dsa_generation;
extern float signal_frequency_hz;
extern float signal_gain;
extern float signal_dc_offset;

#endif // GMP_LAUNCHPAD_USER_DL_H
