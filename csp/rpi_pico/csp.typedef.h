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

//////////////////////////////////////////////////////////////////////////
// Step I: Select HAL library
//

#include <stdint.h>

#include <pico/stdlib.h>
#include <pico/time.h>

#include <hardware/i2c.h>
#include <hardware/spi.h>
#include <hardware/uart.h>
#include <hardware/watchdog.h>

#ifndef _FILE_CSP_TYPE_DEF_H_
#define _FILE_CSP_TYPE_DEF_H_

// This file is for STM32 series micro controller

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

// TIME TYPES
#define GMP_PORT_TIME_T              uint64_t
#define GMP_PORT_TIME_SIZE_PER_BITS  (64)
#define GMP_PORT_TIME_SIZE_PER_BYTES (8)
#define GMP_PORT_TIME_MAXIMUM (UINT64_MAX)

// peripheral types

// specify the GPIO model to be STM32 model
#define GMP_PORT_GPIO_T uint32_t

// specify the UART model
#define GMP_PORT_UART_T uart_inst_t*

// SPI interface
#define GMP_PORT_SPI_T spi_inst_t*

// IIC interface
#define GMP_PORT_I2C_T i2c_inst_t*

#endif
