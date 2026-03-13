/**
 * @file stm32_chip_common.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Computing Model
#include <csp/stm32/common/computing_model.stm32.h>

#ifdef HAL_CORDIC_MODULE_ENABLED

#endif // HAL_CORDIC_MODULE_ENABLED

//////////////////////////////////////////////////////////////////////////
// GPIO Model
#include <csp/stm32/common/gpio_model.stm32.h>

#ifdef HAL_GPIO_MODULE_ENABLED

/**
 * @brief Setup GPIO port and pin.
 * @param hgpio handle of GPIO
 * @param gpio_port GPIO port of STM32
 * @param gpio_pin GPIO pin of STM32
 */
void gmp_hal_gpio_init(gpio_model_stm32_t *hgpio, GPIO_TypeDef *gpio_port, uint32_t gpio_pin)
{
    hgpio->gpio_port = gpio_port;
    hgpio->gpio_pin = gpio_pin;
}

/**
 * @brief Set mode of gpio port, mode 0: input, mode 1: output.
 * @param hgpio handle of gpio. Type of GPIO handle is given by CSP.
 * @param mode target mode of GPIO. mode 0 is input mode, 1 is output mode.
 */
ec_gt gmp_hal_gpio_set_mode(gpio_model_stm32_t *hgpio, fast_gt mode)
{
    return GMP_EC_OK;
}

/**
 * @brief Write GPIO port. This port must be an output port.
 * Or, undefined things may happen.
 * @param hgpio handle of GPIO
 * @param level target electrical level of GPIO port.
 */
ec_gt gmp_hal_gpio_write(gpio_model_stm32_t *hgpio, fast_gt level)
{
    HAL_GPIO_WritePin(hgpio->gpio_port, hgpio->gpio_pin, (GPIO_PinState)level);
    return GMP_EC_OK;
}

/**
 * @brief Read GPIO port, This port should be an input port.
 * Or the return value is undefined.
 * @param hgpio handle of GPIO
 * @return fast_gt return GPIO electrical level
 */
fast_gt gmp_hal_gpio_read(gpio_model_stm32_t *hgpio)
{
    return HAL_GPIO_ReadPin(hgpio->gpio_port, hgpio->gpio_pin);
}

/**
 * @brief Set GPIO electrical level to high.
 * if GPIO mode is not output mode, the result is undefined.
 * @param hgpio handle of GPIO
 */
ec_gt gmp_hal_gpio_set(gpio_model_stm32_t *hgpio)
{
    HAL_GPIO_WritePin(hgpio->gpio_port, hgpio->gpio_pin, (GPIO_PinState)GPIO_PIN_SET);
    return GMP_EC_OK;
}

/**
 * @brief Set GPIO electrical level to low.
 * if GPIO mode is not output mode, the result is undefined.
 * @param hgpio handle of GPIO
 */
ec_gt gmp_hal_gpio_clear(gpio_model_stm32_t *hgpio)
{
    HAL_GPIO_WritePin(hgpio->gpio_port, hgpio->gpio_pin, (GPIO_PinState)GPIO_PIN_RESET);
    return GMP_EC_OK;
}

#endif // HAL_GPIO_MODULE_ENABLED

//////////////////////////////////////////////////////////////////////////
// peripheral Model
#include <csp/stm32/common/peripheral_model.stm32.h>

#ifdef HAL_UART_MODULE_ENABLED

///**
// * @brief Setup GMP UART handle.
// * This function should be called in `peripheral_mapping.c`
// * @param huart handle of GMP handle
// * @param uart_handle STM32 handle of UART
// * @param uart_tx_dma_handle STM32 DMA handle of UART TX
// * @param uart_rx_dma_handle STM32 DMA handle of UART RX
// * @param data data buffer, DMA mode only
// * @param recv_buf data buffer, DMA mode only
// */
//void gmp_hal_uart_setup(stm32_uart_t *huart, UART_HandleTypeDef *uart_handle, DMA_HandleTypeDef *uart_tx_dma_handle,
//                        DMA_HandleTypeDef *uart_rx_dma_handle, duplex_ift *data_buffer, data_gt *recv_buf)
//{
//    huart->uart_handle = uart_handle;
//    huart->uart_tx_dma_handle = uart_tx_dma_handle;
//    huart->uart_rx_dma_handle = uart_rx_dma_handle;
//
//    huart->recv_buf = recv_buf;
//    huart->buffer = data_buffer;
//}

/**
 * @brief send data via UART
 * @param huart handle of UART
 * @param data half_duplex data interface
 */
//ec_gt gmp_hal_uart_send(stm32_uart_t *huart, half_duplex_ift *data)
//{
//    assert(huart != nullptr);
//    assert(huart->uart_handle != nullptr);
//
//    assert(data != nullptr);
//
//    HAL_UART_Transmit(huart->uart_handle, data->buf, data->length, 1);
//    return GMP_EC_OK;
//}

///**
// * @brief receive data via UART
// * @param huart handle of UART
// * @param data half_duplex data interface
// */
//ec_gt gmp_hal_uart_recv(stm32_uart_t *huart, half_duplex_ift *data)
//{
//    HAL_UART_Receive(huart->uart_handle, data->buf, data->length, 1);
//    return GMP_EC_OK;
//}
//
///**
// * @brief bind a duplex data buffer to UART channel.
// * @param huart handle of UART
// * @param data duplex data buffer
// */
//ec_gt gmp_hal_uart_bind_duplex_dma(stm32_uart_t *huart, duplex_ift *data)
//{
//    huart->buffer = data;
//    return GMP_EC_OK;
//}
//
///**
// * @brief start UART listen to receive routine
// * @param huart handle of UART
// */
//ec_gt gmp_hal_uart_listen(stm32_uart_t *huart)
//{
//    HAL_UART_Receive_DMA(huart->uart_handle, (uint8_t *)huart->recv_buf, huart->buffer->capacity);
//    return GMP_EC_OK;
//}
//
///**
// * @brief Get UART listen status, return current receive bytes number.
// * @param huart
// * @return size_gt size of received bytes.
// */
//size_gt gmp_hal_uart_get_listen_status(stm32_uart_t *huart)
//{
//    size_gt data_length = huart->buffer->capacity - __HAL_DMA_GET_COUNTER(huart->uart_rx_dma_handle);
//    return data_length;
//}
//
////
///**
// * @brief This function check receive buffer and update rx_buf via receive buffer.
// * This function should be called in UART interrupt function
// * @param huart
// */
//ec_gt gmp_hal_uart_listen_routine(stm32_uart_t *uart)
//{
//    size_gt data_length;
//
//    if (__HAL_UART_GET_FLAG(uart->uart_handle, UART_FLAG_IDLE) == SET)
//    {
//        // 清除空闲标志位
//        __HAL_UART_CLEAR_IDLEFLAG(uart->uart_handle);
//
//        // 停止DMA的传输过程
//        HAL_UART_DMAStop(uart->uart_handle);
//
//        // 计算接收到的数据长度
//        data_length = uart->buffer->capacity - __HAL_DMA_GET_COUNTER(uart->uart_rx_dma_handle);
//
//        // 判定是否真的接收到数据
//        if (data_length >= 1)
//        {
//            // 此时确实有数据收到
//            // received_flag = 1;
//            // 将数据移出接收缓存，理论上应当移动到栈中
//            memcpy(uart->buffer->rx_buf, uart->recv_buf, data_length);
//        }
//
//        // 重新启动DMA接收
//        HAL_UART_Receive_DMA(uart->uart_handle, (uint8_t *)uart->recv_buf, uart->buffer->capacity);
//
//        // 启动MDA接收使能
//        __HAL_DMA_ENABLE(uart->uart_rx_dma_handle);
//
//        // 再次启用UART空闲状态的中断
//        __HAL_UART_ENABLE_IT(uart->uart_handle, UART_IT_IDLE);
//    }
//
//    return GMP_EC_OK;
//}
//
///**
// * @brief start UART consign to transmit routine.
// * @param huart handle of UART
// */
//ec_gt gmp_hal_uart_consign(stm32_uart_t *huart)
//{
//    HAL_StatusTypeDef stat;
//
//    // judge if a buffer has bind to the object
//    assert(huart != nullptr);
//
//    if (huart->buffer == nullptr || huart->buffer->tx_buf == nullptr)
//        // ignore this error
//        return GMP_EC_OK;
//
//    // Call DMA to send these data
//    if (HAL_DMA_GetState(huart->uart_tx_dma_handle) == HAL_DMA_STATE_READY)
//    {
//        stat = HAL_UART_Transmit_DMA(huart->uart_handle, huart->buffer->tx_buf, huart->buffer->length);
//    }
//
//   return GMP_EC_OK;
//    // if (stat == HAL_OK)
//    //     return content->length;
//    // else
//    //     return 0;
//}
//
///**
// * @brief Get UART consign status, return if consign routine is free.
// * @param huart
// * @return fast_gt
// */
//fast_gt gmp_hal_uart_get_consign_status(stm32_uart_t *huart)
//{
//    if (HAL_DMA_GetState(huart->uart_tx_dma_handle) == HAL_DMA_STATE_READY)
//        return 1; // DMA has released
//    else
//        return 0; // DMA is still in using
//}
//
#endif // HAL_UART_MODULE_ENABLED

////////////////////////////////////////////////////////////////////////
// SPI Model

#if defined HAL_SPI_MODULE_ENABLED

/**
 * @brief GMP SPI peripheral interface
 * This function should be called in peripheral mapping routine.
 * @param spi  handle of spi device
 * @param hspi handle of STM32 SPI device
 * @param ncs  Chip Select GPIO
 */
//void gmp_hal_spi_setup(stm32_spi_t *spi, SPI_HandleTypeDef *hspi, gpio_model_stm32_t *ncs)
//{
//    spi->hspi = hspi;
//    spi->nchip_select = ncs;
//}
//
/**
 * @brief send data via half duplex SPI
 * @param spi handle of SPI
 * @param data half_duplex data interface
 */
ec_gt gmp_hal_spi_send(spi_halt spi, half_duplex_ift *data)
{

    HAL_SPI_Transmit(spi, (uint8_t *)data->buf, data->length, 1);
    return GMP_EC_OK;
}

/**
 * @brief receive data via SPI
 * @param spi handle of SPI
 * @param data half_duplex data interface
 */
ec_gt gmp_hal_spi_recv(spi_halt spi, half_duplex_ift *data)
{
    HAL_SPI_Receive(spi, (uint8_t *)data->buf, data->length, 1);
    return GMP_EC_OK;
}

/**
 * @brief receive and transmit data via SPI interface
 * This function should only be called in SPI duplex mode.
 * @param spi handle of SPI
 * @param data duplex data interface
 */
ec_gt gmp_hal_spi_send_recv(spi_halt spi, duplex_ift *data)
{
    HAL_SPI_TransmitReceive(spi, (uint8_t *)data->tx_buf, (uint8_t *)data->rx_buf, data->length, 1);
    return GMP_EC_OK;
}

#endif // HAL_SPI_MODULE_ENABLED

////////////////////////////////////////////////////////////////////////////
//// IIC Model
//#ifdef HAL_I2C_MODULE_ENABLED
//
///**
// * @brief GMP IIC peripheral interface
// * This function should be called in peripheral mapping routine.
// * @param iic  handle of iic device
// * @param hi2c handle of STM32 IIC device
// */
//void gmp_hal_iic_setup(stm32_iic_t *iic, I2C_HandleTypeDef *hi2c)
//{
//    iic->iic = hi2c;
//}
//
///**
// * @brief IIC memory function, send a IIC memory frame.
// * @param iic handle of IIC
// * @param mem memory send message
// */
//ec_gt gmp_hal_iic_mem_send(stm32_iic_t *iic, iic_memory_ift *mem)
//{
//    HAL_I2C_Mem_Write(iic->iic, mem->dev_addr, mem->mem_addr, mem->mem_length, mem->msg, mem->length, 1);
//    return GMP_EC_OK;
//}
//
///**
// * @brief IIC memory function, recveive a IIC memory frame.
// * @param iic handle of IIC
// * @param mem memory receive message
// */
//ec_gt gmp_hal_iic_mem_recv(stm32_iic_t *iic, iic_memory_ift *mem)
//{
//    HAL_I2C_Mem_Read(iic->iic, mem->dev_addr, mem->mem_addr, mem->mem_length, mem->msg, mem->length, 1);
//    return GMP_EC_OK;
//}
//
///**
// * @brief IIC device send function
// * @param iic handle of IIC
// * @param msg IIC send message
// */
//ec_gt gmp_hal_iic_send(stm32_iic_t *iic, half_duplex_with_addr_ift *msg)
//{
//    HAL_I2C_Master_Transmit(iic->iic, msg->address, msg->msg, msg->length, 1);
//    return GMP_EC_OK;
//}
//
///**
// * @brief IIC device receive function
// * @param iic handle of IIC
// * @param msg IIC receive message
// */
//ec_gt gmp_hal_iic_recv(stm32_iic_t *iic, half_duplex_with_addr_ift *msg)
//{
//    HAL_I2C_Master_Receive(iic->iic, msg->address, msg->msg, msg->length, 1);
//    return GMP_EC_OK;
//}
//
//#endif // HAL_I2C_MODULE_ENABLED
//
//////////////////////////////////////////////////////////////////////////
// System Model
#include <csp/stm32/common/sys_model.stm32.h>

#ifdef HAL_RCC_MODULE_ENABLED

time_gt gmp_base_get_system_tick(void)
{
    return HAL_GetTick();
}

#endif

/**
 * @brief This function may fresh IWDG counter.
 * This function should be implemented by CSP,
 * Every Loop routine, this function would be called.
 * CSP implementation should ensure that the function has only one thing is to feed the watchdog
 */
void gmp_hal_wd_feed(void)
{
#if defined SPECIFY_ENABLE_FEED_WATCHDOG
    HAL_IWDG_Refresh(&hiwdg);
#endif // SPECIFY_ENABLE_FEED_WATCHDOG
}

// This function may be called and used to initilize all the peripheral.
void gmp_csp_startup(void)
{
}

// This function would be called when fatal error occorred.
void gmp_port_system_stuck(void)
{
}

// This function would be called when all the initilization process happened.
void gmp_csp_post_process(void)
{
}

// This function is unreachable.
void gmp_exit_routine(void)
{
}

// This function may invoke when main loop occurred.
void gmp_csp_loop(void)
{
}

uart_halt debug_uart;

// implement the gmp_debug_print routine.
size_gt gmp_base_print_c28xsyscfg(const char* p_fmt, ...)
{
    // if no one was specified to output, just ignore the request.
    if (debug_uart == NULL)
    {
        return 0;
    }

    // size_gt size = (size_gt)strlen(p_fmt);

    static data_gt str[GMP_BASE_PRINT_CHAR_EXT];
    memset(str, 0, GMP_BASE_PRINT_CHAR_EXT);

    va_list vArgs;
    va_start(vArgs, p_fmt);
    vsprintf((char*)str, (char const*)p_fmt, vArgs);
    va_end(vArgs);

    size_gt length = (size_gt)strlen((char*)str);

    HAL_UART_Transmit_DMA(debug_uart, str, length);

    return length;
}
