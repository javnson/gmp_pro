/**
 * @file csp.typedef.h
 * @brief GMP port types backed by the Raspberry Pi Pico SDK.
 */

#ifndef GMP_CSP_RPI_PICO_TYPEDEF_H
#define GMP_CSP_RPI_PICO_TYPEDEF_H

#include <stdint.h>

#include <pico/stdlib.h>
#include <pico/time.h>

#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <hardware/spi.h>
#include <hardware/uart.h>
#include <hardware/watchdog.h>

#define GMP_PORT_DATA_T              unsigned char
#define GMP_PORT_DATA_SIZE_PER_BITS  (8)
#define GMP_PORT_DATA_SIZE_PER_BYTES (1)

#define GMP_PORT_FAST8_T              int_fast32_t
#define GMP_PORT_FAST8_SIZE_PER_BITS  (32)
#define GMP_PORT_FAST8_SIZE_PER_BYTES (4)

#define GMP_PORT_FAST16_T              int_fast32_t
#define GMP_PORT_FAST16_SIZE_PER_BITS  (32)
#define GMP_PORT_FAST16_SIZE_PER_BYTES (4)

#define GMP_PORT_TIME_T              uint64_t
#define GMP_PORT_TIME_SIZE_PER_BITS  (64)
#define GMP_PORT_TIME_SIZE_PER_BYTES (8)
#define GMP_PORT_TIME_MAXIMUM        (UINT64_MAX)

/** Encode a Pico SDK GPIO number as a non-null GMP GPIO handle. */
#define GMP_RPI_PICO_GPIO(pin) ((void*)(uintptr_t)((uint32_t)(pin) + 1U))

/** Decode a valid GMP GPIO handle to its Pico SDK GPIO number. */
#define GMP_RPI_PICO_GPIO_NUM(handle) ((uint)((uintptr_t)(handle) - 1U))

/* A pointer-shaped handle preserves NULL as the unassigned-pin sentinel. */
#define GMP_PORT_GPIO_T void*

#define GMP_PORT_UART_T uart_inst_t*
#define GMP_PORT_SPI_T  spi_inst_t*
#define GMP_PORT_I2C_T  i2c_inst_t*

#endif /* GMP_CSP_RPI_PICO_TYPEDEF_H */
