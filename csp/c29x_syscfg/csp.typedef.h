/**
 * @file csp.typedef.h
 * @brief Native GMP types for the byte-addressed C29 CPU.
 */

#ifndef GMP_C29X_CSP_TYPEDEF_H
#define GMP_C29X_CSP_TYPEDEF_H

/* GMP peripheral handles may be integer base addresses.  Keep NULL an integer
 * null constant so generic handle validity checks remain warning-free. */
#ifdef NULL
#undef NULL
#endif
#define NULL 0

/* C29x data, fast, time, and address types come from core/std/arch. */

#define GMP_PORT_GPIO_T uint32_t
#define GMP_PORT_UART_T uint32_t
#define GMP_PORT_I2C_T  uint32_t
#define GMP_PORT_SPI_T  uint32_t
#define GMP_PORT_CAN_T  uint32_t

#endif /* GMP_C29X_CSP_TYPEDEF_H */
