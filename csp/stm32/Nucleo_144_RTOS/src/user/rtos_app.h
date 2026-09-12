/**
 * @file rtos_app.h
 * @brief FreeRTOS application ownership contract for the H753 reference.
 */

#ifndef GMP_NUCLEO_144_RTOS_APP_H
#define GMP_NUCLEO_144_RTOS_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** Start the RTOS-owned task set and scheduler. This function never returns. */
void gmp_rtos_app_start(void);

/** Observable proof that an application task runs beside the GMP task. */
extern volatile uint32_t gmp_rtos_user_task_heartbeats;

#ifdef __cplusplus
}
#endif

#endif // GMP_NUCLEO_144_RTOS_APP_H
