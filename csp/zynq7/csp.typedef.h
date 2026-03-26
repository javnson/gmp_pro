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

#include "xparameters.h"

#include "xgpiops.h"
#include "xiicps.h"
#include "xspips.h"
#include "xuartps.h"
#include "xtime_l.h"

#ifndef _FILE_CSP_TYPE_DEF_H_
#define _FILE_CSP_TYPE_DEF_H_

// This file is for ZYNQ series micro controller

// ZYNQ BASIC DATA TYPE
#define GMP_PORT_DATA_T              unsigned char
#define GMP_PORT_DATA_SIZE_PER_BITS  (8)
#define GMP_PORT_DATA_SIZE_PER_BYTES (1)

// FAST TYPES

#define GMP_PORT_FAST8_T               int_fast32_t
#define GMP_PORT_FAST8_SIZE_PER_BITS   (32)
#define GMP_PORT_FAST8_SIZE_PER_BYTES  (4)

#define GMP_PORT_FAST16_T              int_fast32_t
#define GMP_PORT_FAST16_SIZE_PER_BITS  (32)
#define GMP_PORT_FAST16_SIZE_PER_BYTES (4)

// TIME TYPES
#define GMP_PORT_TIME_T              uint64_t
#define GMP_PORT_TIME_SIZE_PER_BITS  (64)
#define GMP_PORT_TIME_SIZE_PER_BYTES (8)
#define GMP_PORT_TIME_MAXIMUM        (UINT64_MAX)

// peripheral types

// specify the GPIO model
#define GMP_PORT_GPIO_T uint32_t

extern XGpioPs Gpio;

// specify the UART model
#define GMP_PORT_UART_T XUartPs*

// SPI interface
#define GMP_PORT_SPI_T XSpiPs*

// IIC interface
#define GMP_PORT_I2C_T XIicPs*

#endif
