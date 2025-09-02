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

// System Headers
#include <stdint.h>

// HDSC HC32 Series Device Driver Library(HC32DDL)
#include "hc32_ddl.h"

#ifndef _FILE_CSP_TYPE_DEF_H_
#define _FILE_CSP_TYPE_DEF_H_

// This file is for STM32 series micro controller

// STM32 BASIC DATA TYPE
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

// peripheral types

// HC32 GPIO MODEL
typedef struct _tag_gpio_model_hc32_t
{
    // GPIO port of HC32
    //
    en_port_t gpio_port;

    // GPIO pin of HC32
    //
    en_pin_t gpio_pin;

} gpio_model_hc32_t;

// specify the GPIO model to be HC32 model
#define GMP_PORT_GPIO_T gpio_model_hc32_t*

// specify the UART model
#define GMP_PORT_UART_T M4_USART_TypeDef*

#endif
