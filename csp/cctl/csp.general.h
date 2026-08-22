/**
 * @file csp.general.h
 * @brief C-callable services supplied by the hosted CCTL CSP.
 */

#ifndef GMP_CSP_CCTL_GENERAL_H
#define GMP_CSP_CCTL_GENERAL_H

#include <gmp_type.h>
#include <stdint.h>

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

/** Notify the CSP that one controller interrupt is being dispatched. */
void csp_cctl_notify_controller_interrupt(void);

/** @return Number of controller interrupts dispatched in this simulation. */
uint64_t csp_cctl_controller_interrupt_count(void);

/** Number of controller-rate software oscilloscope channels. */
#define CSP_CCTL_SCOPE_CHANNEL_COUNT (16U)

/** Publish one controller value to the controller-rate CSV oscilloscope. */
void csp_cctl_scope_write(uint32_t channel, float value);

/** Read one controller-rate CSV oscilloscope channel. */
float csp_cctl_scope_read(uint32_t channel);

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
