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

#ifndef _FILE_CSP_TYPE_DEF_H_
#define _FILE_CSP_TYPE_DEF_H_

// C28x data, fast, time, and address types come from core/std/arch. This
// file only publishes C28x SysConfig peripheral handle overrides.

// ....................................................................//
// basic container of PWM results
//
//#ifndef GMP_PORT_PWM_T
//#define GMP_PORT_PWM_T              int16_t
//#define GMP_PORT_PWM_SIZE_PER_BITS  (16)
//#define GMP_PORT_PWM_SIZE_PER_BYTES (2)
//#endif // GMP_PORT_PWM_T

#ifndef GMP_PORT_GPIO_T
#define GMP_PORT_GPIO_T uint32_t
#endif // GMP_PORT_GPIO_T

// C28x device peripheral
#ifndef GMP_PORT_UART_T
#define GMP_PORT_UART_T uint32_t
#endif // GMP_PORT_UART_T

#ifndef GMP_PORT_I2C_T
#define GMP_PORT_I2C_T uint32_t
#endif // GMP_PORT_I2C_T

#ifndef GMP_PORT_SPI_T
#define GMP_PORT_SPI_T uint32_t
#endif // GMP_PORT_SPI_T

#ifndef GMP_PORT_CAN_T
#define GMP_PORT_CAN_T uint32_t
#endif // GMP_PORT_CAN_T

#endif
