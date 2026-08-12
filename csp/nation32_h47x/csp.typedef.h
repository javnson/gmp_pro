/**
 * @file csp.typedef.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include "n32h47x_48x.h"
#include "n32h47x_48x_fdcan.h"
#include "n32h47x_48x_gpio.h"
#include "n32h47x_48x_i2c.h"
#include "n32h47x_48x_spi.h"
#include "n32h47x_48x_usart.h"

#ifndef _FILE_CSP_TYPE_DEF_H_
#define _FILE_CSP_TYPE_DEF_H_

// This file is for Nations N32H47x/N32H48x microcontrollers.

// STM32 BASIC DATA TYPE
#define GMP_PORT_DATA_T              unsigned char
#define GMP_PORT_DATA_SIZE_PER_BITS  (8)
#define GMP_PORT_DATA_SIZE_PER_BYTES (1)

// FAST TYPES
#define GMP_PORT_FAST8_T              int_fast32_t
#define GMP_PORT_FAST8_SIZE_PER_BITS  (32)
#define GMP_PORT_FAST8_SIZE_PER_BYTES (4)

#define GMP_PORT_FAST16_T              int_fast32_t
#define GMP_PORT_FAST16_SIZE_PER_BITS  (32)
#define GMP_PORT_FAST16_SIZE_PER_BYTES (4)

// Peripheral handle types. GPIO needs both the port and pin mask, while the
// remaining peripherals can use the vendor register block pointer directly.
typedef struct _tag_gmp_gpio_n32h47x_t
{
    GPIO_Module *port;
    uint16_t pin;
} gmp_gpio_n32h47x_t;

#define GMP_PORT_GPIO_T gmp_gpio_n32h47x_t *

// specify the UART model
#define GMP_PORT_UART_T USART_Module*

// SPI interface
#define GMP_PORT_SPI_T SPI_Module*

// I2C interface
#define GMP_PORT_I2C_T I2C_Module*

// CAN/CAN-FD interface
#define GMP_PORT_CAN_T FDCAN_Module*

#endif
