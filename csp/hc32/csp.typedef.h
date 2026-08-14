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

// Cortex-M data and fast integer types come from core/std/arch. This file
// only publishes HC32-specific peripheral handle types.

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
