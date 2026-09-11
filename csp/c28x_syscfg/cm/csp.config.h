/**
 * @file csp.config.h
 * @brief GMP CSP configuration for the Cortex-M core integrated in C2000 MCUs.
 */

#ifndef GMP_C28X_SYSCFG_CM_CSP_CONFIG_H
#define GMP_C28X_SYSCFG_CM_CSP_CONFIG_H

/* The CM is a finite bare-metal target and owns its startup/exit policy. */
#define SPECIFY_DISABLE_CSP_EXIT

#endif
