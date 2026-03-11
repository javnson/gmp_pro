/**
 * @file peripheral_model.stm32.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2024-09-30
 * 
 * @copyright Copyright GMP(c) 2024
 * 
 */

// This file contains all the peripheral implements

#ifndef _FILE_PERIPHERAL_MODEL_STM32_H_
#define _FILE_PERIPHERAL_MODEL_STM32_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// ....................................................................//
// PWM controller
//
// kernel functions:
// + Enable PWM output
// + Disable PWM output
// + Set Duty
// + Set frequency
//
#ifdef HAL_TIM_MODULE_ENABLED

#endif // HAL_TIM_MODULE_ENABLED

// ....................................................................//
// UART
//
// kernel functions:
// + Write
// + Read
//
//#ifdef HAL_UART_MODULE_ENABLED

//    /**
//     * @brief STM32 UART handle
//     *
//     */
//    typedef struct _tag_uart_stm32_t
//    {
//        //
//        /**
//         * @brief Handle of STM32 UART
//         */
//        UART_HandleTypeDef *uart_handle;

//        /**
//         * @brief TX DMA handle
//         */
//        DMA_HandleTypeDef *uart_tx_dma_handle;

//        /**
//         * @brief RX DMA handle
//         */
//        DMA_HandleTypeDef *uart_rx_dma_handle;

//        /**
//         * @brief This is a buffer pointer of duplex interface.
//         * User may band to this buffer to get necessary informations.
//         */
//        duplex_ift *buffer;

//        /**
//         * @brief This is a buffer for receive.
//         * This buffer should have the same length with duplex buffer.
//         */
//        data_gt *recv_buf;

//    } stm32_uart_t;

///**
// * @brief Specify `stm32_uart_t` type as huart_gt type
// *
// */
//#define GMP_PORT_UART_T stm32_uart_t

//    /**
//     * @brief Setup GMP UART handle.
//     * This function should be called in `peripheral_mapping.c`
//     * @param huart handle of GMP handle
//     * @param uart_handle STM32 handle of UART
//     * @param uart_tx_dma_handle STM32 DMA handle of UART TX
//     * @param uart_rx_dma_handle STM32 DMA handle of UART RX
//     * @param data data buffer, DMA mode only
//     * @param recv_buf data buffer, DMA mode only
//     */
//    void gmp_hal_uart_setup(stm32_uart_t *huart, UART_HandleTypeDef *uart_handle, DMA_HandleTypeDef *uart_tx_dma_handle,
//                        DMA_HandleTypeDef *uart_rx_dma_handle, duplex_ift *data_buffer, data_gt *recv_buf);

//    //
//    // The following function prototype is copy from gmp_csp_cport.
//    // All these functions support async and sync UART transmition.
//    //

//    /**
//     * @brief send data via UART
//     * @param huart handle of UART
//     * @param data half_duplex data interface
//     */
//    ec_gt gmp_hal_uart_send(stm32_uart_t *huart, half_duplex_ift *data);

//    /**
//     * @brief receive data via UART
//     * @param huart handle of UART
//     * @param data half_duplex data interface
//     */
//    ec_gt gmp_hal_uart_recv(stm32_uart_t *huart, half_duplex_ift *data);

//    /**
//     * @brief bind a duplex data buffer to UART channel.
//     * @param huart handle of UART
//     * @param data duplex data buffer
//     */
//    ec_gt gmp_hal_uart_bind_duplex_dma(stm32_uart_t *huart, duplex_ift *data);

//    /**
//     * @brief start UART listen to receive routine
//     * @param huart handle of UART
//     */
//    ec_gt gmp_hal_uart_listen(stm32_uart_t *huart);

//    /**
//     * @brief Get UART listen status, return current receive bytes number.
//     * @param huart
//     * @return size_gt size of received bytes.
//     */
//    size_gt gmp_hal_uart_get_listen_status(stm32_uart_t *huart);

//    /**
//     * @brief start UART consign to transmit routine.
//     * @param huart handle of UART
//     */
//    ec_gt gmp_hal_uart_consign(stm32_uart_t *huart);

//    /**
//     * @brief Get UART consign status, return if consign routine is free.
//     * @param huart
//     * @return fast_gt
//     */
//    fast_gt gmp_hal_uart_get_consign_status(stm32_uart_t *huart);

//#endif // HAL_UART_MODULE_ENABLED

//// ....................................................................//
//// SPI
////
//// kernel functions:
//// + Write
//// + Read
//// + ReadWrite
//// + Set SPI Mode
////
//#ifdef HAL_SPI_MODULE_ENABLED

//    typedef struct _tag_stm32_spi_handle
//    {
//        // chip select
//        gpio_model_stm32_t *nchip_select;

//        // SPI handle
//        SPI_HandleTypeDef *hspi;
//    } stm32_spi_t;

//#define GMP_PORT_SPI_T stm32_spi_t

//    /**
//     * @brief GMP SPI peripheral interface
//     * This function should be called in peripheral mapping routine.
//     * @param spi  handle of spi device
//     * @param hspi handle of STM32 SPI device
//     * @param ncs  Chip Select GPIO
//     */
//    void gmp_hal_spi_setup(stm32_spi_t *spi,
//        SPI_HandleTypeDef *hspi, gpio_model_stm32_t *ncs);

//    /**
//     * @brief send data via half duplex SPI
//     * @param spi handle of SPI
//     * @param data half_duplex data interface
//     */
//    stat_gt gmp_hal_spi_send(stm32_spi_t *spi, half_duplex_ift *data);

//    /**
//     * @brief receive data via SPI
//     * @param spi handle of SPI
//     * @param data half_duplex data interface
//     */
//    stat_gt gmp_hal_spi_recv(stm32_spi_t *spi, half_duplex_ift *data);

//    /**
//     * @brief receive and transmit data via SPI interface
//     * This function should only be called in SPI duplex mode.
//     * @param spi handle of SPI
//     * @param data duplex data interface
//     */
//    stat_gt gmp_hal_spi_send_recv(stm32_spi_t *spi, duplex_ift *data);
size_gt gmp_hal_spi_read_write(spi_halt spi, data_gt* data_in, data_gt* data_out, size_gt length);

//#endif // HAL_SPI_MODULE_ENABLED

// ....................................................................//
// IIC
//
// kernel functions:
// + Write
// + Read
//
#ifdef HAL_I2C_MODULE_ENABLED

typedef I2C_HandleTypeDef* stm32_iic_t;

#define GMP_PORT_I2C_T stm32_iic_t

void gmp_hal_iic_write_cmd(stm32_iic_t* h, addr16_gt dev_addr, uint32_t cmd, size_gt cmd_len);

void gmp_hal_iic_write_reg(stm32_iic_t* h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t reg_data,
                           size_gt reg_len);

void gmp_hal_iic_write_mem(stm32_iic_t* h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, data_gt* mem,
                           size_gt mem_len);

uint32_t gmp_hal_iic_read_reg(stm32_iic_t* h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len,
                              size_gt reg_len);

void gmp_hal_iic_read_mem(stm32_iic_t* h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, data_gt* mem,
                          size_gt mem_len);

//    /**
//     * @brief GMP IIC peripheral interface
//     * This function should be called in peripheral mapping routine.
//     * @param iic  handle of iic device
//     * @param hi2c handle of STM32 IIC device
//     */
//    void gmp_hal_iic_setup(stm32_iic_t *iic, I2C_HandleTypeDef *hi2c);

//    /**
//     * @brief IIC memory function, send a IIC memory frame.
//     * @param iic handle of IIC
//     * @param mem memory send message
//     */
//    stat_gt gmp_hal_iic_mem_send(stm32_iic_t *iic, iic_memory_ift *mem);

//    /**
//     * @brief IIC memory function, recveive a IIC memory frame.
//     * @param iic handle of IIC
//     * @param mem memory receive message
//     */
//    stat_gt gmp_hal_iic_mem_recv(stm32_iic_t *iic, iic_memory_ift *mem);

//    /**
//     * @brief IIC device send function
//     * @param iic handle of IIC
//     * @param msg IIC send message
//     */
//    stat_gt gmp_hal_iic_send(stm32_iic_t *iic, half_duplex_with_addr_ift *msg);

//    /**
//     * @brief IIC device receive function
//     * @param iic handle of IIC
//     * @param msg IIC receive message
//     */
//    stat_gt gmp_hal_iic_recv(stm32_iic_t *iic, half_duplex_with_addr_ift *msg);

//#endif // HAL_I2C_MODULE_ENABLED

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PERIPHERAL_MODEL_STM32_H_
