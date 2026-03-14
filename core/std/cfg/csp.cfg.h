/**
 * @file gmp_csp_cport.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief This file contains all the functions that User should implement it.
 * @version 0.1
 * @date 2024-09-28
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// #include "core/std/default.types.h"
// #include <core/dev/devif.h>

#ifndef _FILE_GMP_CSP_CPORT_H_
#define _FILE_GMP_CSP_CPORT_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//////////////////////////////////////////////////////////////////////////
// CSP Basic interface

// This function would be implemented by CSP (Chip support package).
// This function would be called firstly, by `gmp_init`.
//
void gmp_csp_startup(void);

// This function would be called every main loop routine.
//
void gmp_csp_loop(void);

// This function is the last function before mainloop.
// This function is implemented by CSP
//
void gmp_csp_post_process(void);

// GMP Exit routine
// This function should be implemented by CSP submodule.
// And generally, this function is a null function.
//
void gmp_csp_exit(void);

// GMP system stuck routine
// This function shoudl implemented by CSP
void gmp_csp_stuck_routine(void);

// GMP system not-implement function
void gmp_csp_not_implement(void);

//////////////////////////////////////////////////////////////////////////
// GPIO general port

///**
// * @brief Set mode of gpio port, mode 0: input, mode 1: output.
// * @param hgpio handle of gpio. Type of GPIO handle is given by CSP.
// * @param mode target mode of GPIO. mode 0 is input mode, 1 is output mode.
// */
//#ifndef gmp_hal_gpio_set_mode
//ec_gt gmp_hal_gpio_set_mode(gpio_halt hgpio, fast_gt mode);
//#endif // gmp_hal_gpio_set_mode
//
///**
// * @brief Write GPIO port. This port must be an output port.
// * Or, undefined things may happen.
// * @param hgpio handle of GPIO
// * @param level target electrical level of GPIO port.
// */
//#ifndef gmp_hal_gpio_write
//ec_gt gmp_hal_gpio_write(gpio_halt hgpio, fast_gt level);
//#endif // gmp_hal_gpio_write
///**
// * @brief Read GPIO port, This port should be an input port.
// * Or the return value is undefined.
// * @param hgpio handle of GPIO
// * @return fast_gt return GPIO electrical level
// */
//#ifndef gmp_hal_gpio_read
//fast_gt gmp_hal_gpio_read(gpio_halt hgpio);
//#endif // gmp_hal_gpio_read
//
///**
// * @brief Set GPIO electrical level to high.
// * if GPIO mode is not output mode, the result is undefined.
// * @param hgpio handle of GPIO
// */
//#ifndef gmp_hal_gpio_set
//ec_gt gmp_hal_gpio_set(gpio_halt hgpio);
//#endif // gmp_hal_gpio_set
//
///**
// * @brief Set GPIO electrical level to low.
// * if GPIO mode is not output mode, the result is undefined.
// * @param hgpio handle of GPIO
// */
//#ifndef gmp_hal_gpio_clear
//ec_gt gmp_hal_gpio_clear(gpio_halt hgpio);
//#endif
//
////////////////////////////////////////////////////////////////////////////
//// UART general port
//
///**
// * @brief send data via UART
// * @param huart handle of UART
// * @param data half_duplex data interface
// */
//#ifndef gmp_hal_uart_send
//ec_gt gmp_hal_uart_send(uart_halt huart, half_duplex_ift *data);
//#endif // gmp_hal_uart_send
//
///**
// * @brief receive data via UART
// * @param huart handle of UART
// * @param data half_duplex data interface
// */
//#ifndef gmp_hal_uart_recv
//ec_gt gmp_hal_uart_recv(uart_halt huart, half_duplex_ift *data);
//#endif
//
///**
// * @brief bind a duplex data buffer to UART channel.
// * @param huart handle of UART
// * @param data duplex data buffer
// */
//ec_gt gmp_hal_uart_bind_duplex_dma(uart_halt huart, duplex_ift *data);
//
///**
// * @brief start UART listen to receive routine
// * @param huart handle of UART
// */
//ec_gt gmp_hal_uart_listen(uart_halt huart);
//
///**
// * @brief Get UART listen status, return current receive bytes number.
// * @param huart
// * @return size_gt size of received bytes.
// */
//size_gt gmp_hal_uart_get_listen_status(uart_halt huart);
//
///**
// * @brief start UART consign to transmit routine.
// * @param huart handle of UART
// */
//ec_gt gmp_hal_uart_consign(uart_halt huart);
//
///**
// * @brief Get UART consign status, return if consign routine is free.
// * @param huart
// * @return fast_gt
// */
//fast_gt gmp_hal_uart_get_consign_status(uart_halt huart);
//
////////////////////////////////////////////////////////////////////////////
//// SPI general port
//
///**
// * @brief send data via half duplex SPI
// * @param spi handle of SPI
// * @param data half_duplex data interface
// */
//#ifndef gmp_hal_spi_send
//ec_gt gmp_hal_spi_send(spi_halt spi, half_duplex_ift *data);
//#endif // gmp_hal_spi_send
//
///**
// * @brief receive data via SPI
// * @param spi handle of SPI
// * @param data half_duplex data interface
// */
//#ifndef gmp_hal_spi_recv
//ec_gt gmp_hal_spi_recv(spi_halt spi, half_duplex_ift *data);
//#endif // gmp_hal_spi_recv
//
///**
// * @brief receive and transmit data via SPI interface
// * This function should only be called in SPI duplex mode.
// * @param spi handle of SPI
// * @param data duplex data interface
// */
//#ifndef gmp_hal_spi_send_recv
//ec_gt gmp_hal_spi_send_recv(spi_halt spi, duplex_ift *data);
//#endif // gmp_hal_spi_send_recv
//
////////////////////////////////////////////////////////////////////////////
//// IIC general port
//
///**
// * @brief IIC memory function, send a IIC memory frame.
// * @param iic handle of IIC
// * @param mem memory send message
// */
//#ifndef gmp_hal_iic_mem_send
//ec_gt gmp_hal_iic_mem_send(iic_halt iic, iic_memory_ift *mem);
//#endif // gmp_hal_iic_mem_send
//
///**
// * @brief IIC memory function, recveive a IIC memory frame.
// * @param iic handle of IIC
// * @param mem memory receive message
// */
//#ifndef gmp_hal_iic_mem_recv
//ec_gt gmp_hal_iic_mem_recv(iic_halt iic, iic_memory_ift *mem);
//#endif // gmp_hal_iic_mem_recv
//
///**
// * @brief IIC device send function
// * @param iic handle of IIC
// * @param msg IIC send message
// */
//#ifndef gmp_hal_iic_send
//ec_gt gmp_hal_iic_send(iic_halt iic, half_duplex_with_addr_ift *msg);
//#endif // gmp_hal_iic_send
//
///**
// * @brief IIC device receive function
// * @param iic handle of IIC
// * @param msg IIC receive message
// */
//#ifndef gmp_hal_iic_recv
//ec_gt gmp_hal_iic_recv(iic_halt iic, half_duplex_with_addr_ift *msg);
//#endif // gmp_hal_iic_recv

//////////////////////////////////////////////////////////////////////////
// CAN general port

//////////////////////////////////////////////////////////////////////////
// WatchDog peripheral general interface

/**
 * @brief This function may fresh IWDG counter.
 * This function should be implemented by CSP,
 * Every Loop routine, this function would be called.
 * CSP implementation should ensure that the function has only one thing is to feed the watchdog
 */
void gmp_hal_wd_feed(void);

void gmp_hal_wd_enable(void);

void gmp_hal_wd_disable(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_CSP_CPORT_H_
