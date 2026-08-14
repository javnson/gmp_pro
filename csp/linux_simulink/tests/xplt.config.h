/**
 * @file xplt.config.h
 * @brief Minimal application contract for Linux CSP compile validation.
 */

#ifndef GMP_LINUX_SIMULINK_TEST_XPLT_CONFIG_H
#define GMP_LINUX_SIMULINK_TEST_XPLT_CONFIG_H

#define SPECIFY_DISABLE_GMP_CTL

typedef struct
{
    double time;
    double panel[4];
} rx_buf_t;

typedef struct
{
    int enable;
} tx_buf_t;

#endif /* GMP_LINUX_SIMULINK_TEST_XPLT_CONFIG_H */
