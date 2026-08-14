/**
 * @file csp.general.h
 * @brief C-facing services supplied by the hosted Linux CSP.
 */

#ifndef GMP_LINUX_SIMULINK_CSP_GENERAL_H
#define GMP_LINUX_SIMULINK_CSP_GENERAL_H

#include <signal.h>
#include <core/std/gmp.std.h>

#ifndef GMP_DBG_SWBP
#define GMP_DBG_SWBP raise(SIGTRAP)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Enable the simulated controller output. */
void csp_sl_enable_output(void);

/** @brief Disable the simulated controller output. */
void csp_sl_disable_output(void);

/** @brief Read one scalar value from the simulated input panel. */
double csp_sl_get_panel_input(fast_gt channel);

#ifdef __cplusplus
}
#endif

#endif /* GMP_LINUX_SIMULINK_CSP_GENERAL_H */
