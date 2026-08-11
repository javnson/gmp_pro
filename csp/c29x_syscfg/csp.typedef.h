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

#define GMP_PORT_DATA_T              uint8_t
#define GMP_PORT_DATA_SIZE_PER_BITS  (8)
#define GMP_PORT_DATA_SIZE_PER_BYTES (1)

#define GMP_PORT_FAST_T              int_fast32_t
#define GMP_PORT_FAST_SIZE_PER_BITS  (32)
#define GMP_PORT_FAST_SIZE_PER_BYTES (4)

#define GMP_PORT_FAST16_T              int_fast32_t
#define GMP_PORT_FAST16_SIZE_PER_BITS  (32)
#define GMP_PORT_FAST16_SIZE_PER_BYTES (4)

#define GMP_PORT_GPIO_T uint32_t
#define GMP_PORT_UART_T uint32_t
#define GMP_PORT_I2C_T  uint32_t
#define GMP_PORT_SPI_T  uint32_t
#define GMP_PORT_CAN_T  uint32_t

#endif /* GMP_C29X_CSP_TYPEDEF_H */
