/**
 * @file gmp_rtos.h
 * @brief RTOS-owned entry contract for the GMP background executor.
 */

#ifndef _FILE_GMP_RTOS_H_
#define _FILE_GMP_RTOS_H_

#include <gmp_type.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Run GMP initialization and background polling in an RTOS task.
 *
 * The selected RTOS backend implements this function. The RTOS application or
 * generated project owns task creation, task priority, kernel initialization,
 * and kernel startup. This function does not return during normal operation.
 */
void gmp_rtos_runtime_task(void* argument);

#ifdef __cplusplus
}
#endif

#endif // _FILE_GMP_RTOS_H_
