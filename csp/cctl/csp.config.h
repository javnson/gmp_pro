/**
 * @file csp.config.h
 * @brief Default compile-time configuration for the hosted CCTL CSP.
 */

#ifndef GMP_CSP_CCTL_CONFIG_H
#define GMP_CSP_CCTL_CONFIG_H

/** Mark this CSP as a finite hosted simulation environment. */
#ifndef SPECIFY_PC_ENVIRONMENT
#define SPECIFY_PC_ENVIRONMENT
#endif

/** Allow the hosted CSP to finish the finite GMP background loop cleanly. */
#ifndef SPECIFY_ENABLE_CSP_RUNTIME_EXIT
#define SPECIFY_ENABLE_CSP_RUNTIME_EXIT
#endif

/** Let the simulated MCU compute allocator invoke the CTL background loop. */
#ifndef SPECIFY_CSP_MANAGES_CTL_MAINLOOP
#define SPECIFY_CSP_MANAGES_CTL_MAINLOOP
#endif

/** Let the simulated MCU compute allocator invoke the user main loop. */
#ifndef SPECIFY_CSP_MANAGES_USER_MAINLOOP
#define SPECIFY_CSP_MANAGES_USER_MAINLOOP
#endif

/** Defensive upper bound; normal completion uses gmp_csp_should_exit(). */
#ifndef PC_ENV_MAX_ITERATION
#define PC_ENV_MAX_ITERATION ((size_t)-1)
#endif

/** Preserve controller arithmetic behavior unless a project overrides it. */
#ifndef SPECIFY_CTRL_GT_TYPE
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
#endif

/** Default hosted tick resolution in ticks per second. */
#ifndef GMP_BASE_TIME_TICK_RESOLUTION
#define GMP_BASE_TIME_TICK_RESOLUTION 1000U
#endif

#endif /* GMP_CSP_CCTL_CONFIG_H */
