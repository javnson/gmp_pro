/**
 * @file csp.typedef.h
 * @brief C-compatible type customization for the hosted CCTL CSP.
 */

#ifndef GMP_CSP_CCTL_TYPEDEF_H
#define GMP_CSP_CCTL_TYPEDEF_H

/** Keep controller arithmetic at float precision unless explicitly replaced. */
#ifndef SPECIFY_CTRL_GT_TYPE
#define SPECIFY_CTRL_GT_TYPE USING_FLOAT_FPU
#endif

/* All other portable C types use the definitions supplied by core/std. */

#endif /* GMP_CSP_CCTL_TYPEDEF_H */
