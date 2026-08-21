/**
 * @file csp.general.h
 * @brief C-callable services supplied by the hosted CCTL CSP.
 */

#ifndef GMP_CSP_CCTL_GENERAL_H
#define GMP_CSP_CCTL_GENERAL_H

#include <gmp_type.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** Enable simulated converter outputs at the CSP boundary. */
void csp_sl_enable_output(void);

/** Disable simulated converter outputs at the CSP boundary. */
void csp_sl_disable_output(void);

/** @return Nonzero while simulated converter outputs are enabled. */
fast_gt csp_cctl_output_is_enabled(void);

/** Hosted no-op watchdog feed service. */
void gmp_hal_wd_feed(void);

/** Hosted no-op watchdog enable service. */
void gmp_hal_wd_enable(void);

/** Hosted no-op watchdog disable service. */
void gmp_hal_wd_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* GMP_CSP_CCTL_GENERAL_H */
