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

#ifndef SPECIFY_PROJECT_GENERATED_BY_CUBEMX

// Due to STM32 has unique macro to specify the chip set,
// just judge these macros.
//
#if defined STM32G030xx
#include "stm32g0xx_hal.h"
#elif defined STM32G071xx
#include "stm32g0xx_hal.h"
#elif defined STM32G474xx
#include "stm32g4xx_hal.h"
#elif defined STM32G431xx
#include "stm32g4xx_hal.h"
#elif defined STM32L151xx
#include "stm32l1xx_hal.h"
#elif defined STM32L151xB
#include "stm32l1xx_hal.h"
#elif defined STM32F103xB
#include "stm32f1xx_hal.h"
#elif defined STM32F103x6
#include "stm32f1xx_hal.h"
#elif defined STM32F411xx
#include "stm32f4xx_hal.h"
#elif defined STM32F411xE
#include "stm32f4xx_hal.h"
#elif defined STM32U083xx
#include "stm32u0xx_hal.h"

#endif // STM32 SERIES

#else

// Cube MX will generate main.h so just use it.
#include "main.h"

#endif

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

// STM32 GPIO MODEL
typedef struct _tag_gpio_model_stm32_t
{
    // GPIO port of STM32
    //
    GPIO_TypeDef *gpio_port;

    // GPIO pin of STM32
    //
    uint32_t gpio_pin;

} gpio_model_stm32_t;

// specify the GPIO model to be STM32 model
#define GMP_PORT_GPIO_T gpio_model_stm32_t *

// specify the UART model
#define GMP_PORT_UART_T UART_HandleTypeDef *

// SPI interface
#define GMP_PORT_SPI_T SPI_HandleTypeDef *

#endif
