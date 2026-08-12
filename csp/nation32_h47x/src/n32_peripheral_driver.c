/**
 * @file n32_peripheral_driver.c
 * @brief GMP blocking GPIO/UART/SPI adapters for Nations N32H47x/N32H48x.
 */

#include <gmp_core.h>

static fast_gt n32_timeout_expired(time_gt start, time_gt timeout)
{
    return (timeout == 0U) || ((gmp_base_get_system_tick() - start) >= timeout);
}

ec_gt gmp_hal_gpio_set_dir(gpio_halt hgpio, gpio_dir_et dir)
{
    GPIO_InitType config;

    if ((hgpio == NULL) || (hgpio->port == NULL))
        return GMP_EC_INVALID_PARAM;

    GPIO_InitStruct(&config);
    config.Pin = hgpio->pin;
    config.GPIO_Pull = GPIO_NO_PULL;
    config.GPIO_Slew_Rate = GPIO_SLEW_RATE_SLOW;
    config.GPIO_Current = GPIO_DS_2mA;
    config.GPIO_Alternate = GPIO_NO_AF;
    config.GPIO_Mode = (dir == GMP_HAL_GPIO_DIR_OUT) ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
    GPIO_InitPeripheral(hgpio->port, &config);
    return GMP_EC_OK;
}

ec_gt gmp_hal_gpio_write(gpio_halt hgpio, fast_gt level)
{
    if ((hgpio == NULL) || (hgpio->port == NULL))
        return GMP_EC_INVALID_PARAM;

    if (level == GMP_HAL_GPIO_LOW)
        GPIO_ResetBits(hgpio->port, hgpio->pin);
    else
        GPIO_SetBits(hgpio->port, hgpio->pin);
    return GMP_EC_OK;
}

fast_gt gmp_hal_gpio_read(gpio_halt hgpio)
{
    if ((hgpio == NULL) || (hgpio->port == NULL))
        return GMP_HAL_GPIO_LOW;
    return GPIO_ReadInputDataBit(hgpio->port, hgpio->pin) ? GMP_HAL_GPIO_HIGH : GMP_HAL_GPIO_LOW;
}

fast_gt gmp_hal_uart_is_tx_busy(uart_halt uart)
{
    if (uart == NULL)
        return 0;
    return (USART_GetFlagStatus(uart, USART_FLAG_TXC) == RESET) ? 1 : 0;
}

size_gt gmp_hal_uart_get_rx_available(uart_halt uart)
{
    if (uart == NULL)
        return 0;
    return (USART_GetFlagStatus(uart, USART_FLAG_RXDNE) == SET) ? 1U : 0U;
}

ec_gt gmp_hal_uart_write(uart_halt uart, const data_gt *data, size_gt length, uint32_t timeout)
{
    size_gt i;
    time_gt start;

    if ((uart == NULL) || (data == NULL) || (length == 0U))
        return GMP_EC_INVALID_PARAM;

    start = gmp_base_get_system_tick();
    for (i = 0; i < length; ++i)
    {
        while (USART_GetFlagStatus(uart, USART_FLAG_TXDE) == RESET)
        {
            if (n32_timeout_expired(start, (time_gt)timeout))
                return GMP_EC_TIMEOUT;
        }
        USART_SendData(uart, data[i]);
    }

    while (USART_GetFlagStatus(uart, USART_FLAG_TXC) == RESET)
    {
        if (n32_timeout_expired(start, (time_gt)timeout))
            return GMP_EC_TIMEOUT;
    }
    return GMP_EC_OK;
}

ec_gt gmp_hal_uart_read(uart_halt uart, data_gt *data, size_gt length, uint32_t timeout, size_gt *bytes_read)
{
    size_gt count = 0U;
    time_gt start;

    if (bytes_read != NULL)
        *bytes_read = 0U;
    if ((uart == NULL) || (data == NULL) || (length == 0U))
        return GMP_EC_INVALID_PARAM;

    start = gmp_base_get_system_tick();
    while (count < length)
    {
        if (USART_GetFlagStatus(uart, USART_FLAG_RXDNE) == SET)
        {
            data[count++] = (data_gt)USART_ReceiveData(uart);
            continue;
        }
        if (n32_timeout_expired(start, (time_gt)timeout))
        {
            if (bytes_read != NULL)
                *bytes_read = count;
            return GMP_EC_TIMEOUT;
        }
    }

    if (bytes_read != NULL)
        *bytes_read = count;
    return GMP_EC_OK;
}

ec_gt gmp_hal_spi_bus_transfer(spi_halt hspi, const data_gt *tx_buf, data_gt *rx_buf, size_gt len, time_gt timeout)
{
    size_gt i;
    time_gt start;

    if ((hspi == NULL) || (len == 0U) || ((tx_buf == NULL) && (rx_buf == NULL)))
        return GMP_EC_INVALID_PARAM;

    start = gmp_base_get_system_tick();
    for (i = 0; i < len; ++i)
    {
        while (SPI_I2S_GetStatus(hspi, SPI_I2S_TE_FLAG) == RESET)
        {
            if (n32_timeout_expired(start, timeout))
                return GMP_EC_TIMEOUT;
        }
        SPI_I2S_TransmitData(hspi, (tx_buf != NULL) ? tx_buf[i] : 0xFFU);

        while (SPI_I2S_GetStatus(hspi, SPI_I2S_RNE_FLAG) == RESET)
        {
            if (n32_timeout_expired(start, timeout))
                return GMP_EC_TIMEOUT;
        }
        if (rx_buf != NULL)
            rx_buf[i] = (data_gt)SPI_I2S_ReceiveData(hspi);
        else
            (void)SPI_I2S_ReceiveData(hspi);
    }

    while (SPI_I2S_GetStatus(hspi, SPI_I2S_BUSY_FLAG) == SET)
    {
        if (n32_timeout_expired(start, timeout))
            return GMP_EC_TIMEOUT;
    }
    return GMP_EC_OK;
}

ec_gt gmp_hal_spi_bus_write(spi_halt hspi, const data_gt *tx_buf, size_gt len, time_gt timeout)
{
    return gmp_hal_spi_bus_transfer(hspi, tx_buf, NULL, len, timeout);
}

ec_gt gmp_hal_spi_bus_read(spi_halt hspi, data_gt *rx_buf, size_gt len, time_gt timeout)
{
    return gmp_hal_spi_bus_transfer(hspi, NULL, rx_buf, len, timeout);
}
