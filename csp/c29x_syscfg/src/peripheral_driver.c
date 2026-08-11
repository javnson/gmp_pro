/**
 * @file peripheral_driver.c
 * @brief GMP GPIO and UART ports implemented with F29H85x DriverLib.
 */

#include <gmp_core.h>

#include <limits.h>

#define GMP_C29X_TIMEOUT_US_PER_MS (1000U)

static uint32_t gmp_c29x_timeout_us(uint32_t timeout_ms)
{
    if (timeout_ms == 0U)
    {
        return 0U;
    }
    if (timeout_ms > (UINT32_MAX / GMP_C29X_TIMEOUT_US_PER_MS))
    {
        return UINT32_MAX;
    }
    return timeout_ms * GMP_C29X_TIMEOUT_US_PER_MS;
}

ec_gt gmp_hal_gpio_set_dir(gpio_halt gpio, gpio_dir_et direction)
{
    GPIO_setDirectionMode(
        (uint32_t)gpio,
        (direction == GMP_HAL_GPIO_DIR_OUT) ? GPIO_DIR_MODE_OUT : GPIO_DIR_MODE_IN);
    return GMP_EC_OK;
}

ec_gt gmp_hal_gpio_write(gpio_halt gpio, fast_gt level)
{
    GPIO_writePin((uint32_t)gpio, (level == GMP_HAL_GPIO_LOW) ? 0U : 1U);
    return GMP_EC_OK;
}

fast_gt gmp_hal_gpio_read(gpio_halt gpio)
{
    return (fast_gt)GPIO_readPin((uint32_t)gpio);
}

fast_gt gmp_hal_uart_is_tx_busy(uart_halt uart)
{
    return (fast_gt)UART_isBusy((uint32_t)uart);
}

fast_gt gmp_hal_uart_is_busy(uart_halt uart)
{
    return gmp_hal_uart_is_tx_busy(uart);
}

size_gt gmp_hal_uart_get_rx_available(uart_halt uart)
{
    return UART_isDataAvailable((uint32_t)uart) ? 1U : 0U;
}

ec_gt gmp_hal_uart_write(
    uart_halt uart, const data_gt *data, size_gt length, uint32_t timeout)
{
    uint32_t base = (uint32_t)uart;
    uint32_t remaining_us;
    size_gt index;

    if ((data == NULL) && (length != 0U))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    for (index = 0U; index < length; ++index)
    {
        remaining_us = gmp_c29x_timeout_us(timeout);
        while (!UART_isSpaceAvailable(base))
        {
            if (remaining_us == 0U)
            {
                return GMP_EC_TIMEOUT;
            }
            DEVICE_DELAY_US(1U);
            --remaining_us;
        }
        UART_writeCharNonBlocking(base, (uint8_t)data[index]);
    }
    return GMP_EC_OK;
}

ec_gt gmp_hal_uart_read(
    uart_halt uart,
    data_gt *data,
    size_gt length,
    uint32_t timeout,
    size_gt *bytes_read)
{
    uint32_t base = (uint32_t)uart;
    uint32_t remaining_us;
    size_gt index;

    if (bytes_read != NULL)
    {
        *bytes_read = 0U;
    }
    if ((data == NULL) && (length != 0U))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    for (index = 0U; index < length; ++index)
    {
        remaining_us = gmp_c29x_timeout_us(timeout);
        while (!UART_isDataAvailable(base))
        {
            if (remaining_us == 0U)
            {
                if (bytes_read != NULL)
                {
                    *bytes_read = index;
                }
                return GMP_EC_TIMEOUT;
            }
            DEVICE_DELAY_US(1U);
            --remaining_us;
        }
        data[index] = (data_gt)UART_readCharNonBlocking(base);
    }

    if (bytes_read != NULL)
    {
        *bytes_read = length;
    }
    return GMP_EC_OK;
}

static ec_gt gmp_c29x_i2c_status(uint32_t base)
{
    uint16_t status = I2C_getStatus(base);
    if ((status & I2C_STS_NO_ACK) != 0U)
    {
        I2C_clearStatus(base, I2C_STS_NO_ACK);
        return GMP_EC_NACK;
    }
    if ((status & I2C_STS_ARB_LOST) != 0U)
    {
        I2C_clearStatus(base, I2C_STS_ARB_LOST);
        return GMP_EC_BUSY;
    }
    return GMP_EC_OK;
}

static ec_gt gmp_c29x_i2c_wait_idle(uint32_t base, time_gt timeout)
{
    uint32_t remaining_us = gmp_c29x_timeout_us((uint32_t)timeout);
    while (I2C_isBusBusy(base))
    {
        ec_gt status = gmp_c29x_i2c_status(base);
        if (status != GMP_EC_OK)
            return status;
        if (remaining_us == 0U)
            return GMP_EC_TIMEOUT;
        DEVICE_DELAY_US(1U);
        --remaining_us;
    }
    return GMP_EC_OK;
}

static ec_gt gmp_c29x_i2c_wait_tx_space(uint32_t base, time_gt timeout)
{
    uint32_t remaining_us = gmp_c29x_timeout_us((uint32_t)timeout);
    while (I2C_getTxFIFOStatus(base) == I2C_FIFO_TXFULL)
    {
        ec_gt status = gmp_c29x_i2c_status(base);
        if (status != GMP_EC_OK)
            return status;
        if (remaining_us == 0U)
            return GMP_EC_TIMEOUT;
        DEVICE_DELAY_US(1U);
        --remaining_us;
    }
    return GMP_EC_OK;
}

static ec_gt gmp_c29x_i2c_wait_rx_data(uint32_t base, time_gt timeout)
{
    uint32_t remaining_us = gmp_c29x_timeout_us((uint32_t)timeout);
    while (I2C_getRxFIFOStatus(base) == I2C_FIFO_RXEMPTY)
    {
        ec_gt status = gmp_c29x_i2c_status(base);
        if (status != GMP_EC_OK)
            return status;
        if (remaining_us == 0U)
            return GMP_EC_TIMEOUT;
        DEVICE_DELAY_US(1U);
        --remaining_us;
    }
    return GMP_EC_OK;
}

static ec_gt gmp_c29x_i2c_wait_ardy(uint32_t base, time_gt timeout)
{
    uint32_t remaining_us = gmp_c29x_timeout_us((uint32_t)timeout);
    while ((I2C_getStatus(base) & I2C_STS_REG_ACCESS_RDY) == 0U)
    {
        ec_gt status = gmp_c29x_i2c_status(base);
        if (status != GMP_EC_OK)
            return status;
        if (remaining_us == 0U)
            return GMP_EC_TIMEOUT;
        DEVICE_DELAY_US(1U);
        --remaining_us;
    }
    I2C_clearStatus(base, I2C_STS_REG_ACCESS_RDY);
    return GMP_EC_OK;
}

static ec_gt gmp_c29x_i2c_wait_stop(uint32_t base, time_gt timeout)
{
    uint32_t remaining_us = gmp_c29x_timeout_us((uint32_t)timeout);
    while (I2C_getStopConditionStatus(base))
    {
        ec_gt status = gmp_c29x_i2c_status(base);
        if (status != GMP_EC_OK)
            return status;
        if (remaining_us == 0U)
            return GMP_EC_TIMEOUT;
        DEVICE_DELAY_US(1U);
        --remaining_us;
    }
    return gmp_c29x_i2c_wait_idle(base, timeout);
}

static void gmp_c29x_store_be(uint8_t *destination, uint32_t value, size_gt length)
{
    size_gt index;
    for (index = 0U; index < length; ++index)
        destination[index] = (uint8_t)(value >> (8U * (length - index - 1U)));
}

static ec_gt gmp_c29x_i2c_write(
    uint32_t base,
    addr16_gt device_address,
    const uint8_t *prefix,
    size_gt prefix_length,
    const data_gt *data,
    size_gt data_length,
    time_gt timeout)
{
    size_gt index;
    size_gt total_length = prefix_length + data_length;
    ec_gt status;

    if ((base == 0U) || (total_length == 0U) || (total_length > UINT16_MAX) ||
        ((prefix == NULL) && (prefix_length != 0U)) ||
        ((data == NULL) && (data_length != 0U)))
        return GMP_EC_INVALID_PARAM;

    status = gmp_c29x_i2c_wait_idle(base, timeout);
    if (status != GMP_EC_OK)
        return status;

    I2C_disableFIFO(base);
    I2C_clearStatus(base, I2C_STS_NO_ACK | I2C_STS_ARB_LOST | I2C_STS_REG_ACCESS_RDY);
    I2C_enableFIFO(base);
    I2C_setTargetAddress(base, (uint16_t)device_address);
    I2C_setDataCount(base, (uint16_t)total_length);
    I2C_setConfig(base, I2C_CONTROLLER_SEND_MODE);

    /* Prime the FIFO before START, as required by the TI polling examples. */
    for (index = 0U; (index < total_length) && (index < 16U); ++index)
        I2C_putData(base, (index < prefix_length) ? prefix[index] :
                    (uint8_t)data[index - prefix_length]);
    I2C_sendStartCondition(base);

    for (; index < total_length; ++index)
    {
        status = gmp_c29x_i2c_wait_tx_space(base, timeout);
        if (status != GMP_EC_OK)
            goto fail;
        I2C_putData(base, (index < prefix_length) ? prefix[index] :
                    (uint8_t)data[index - prefix_length]);
    }

    I2C_sendStopCondition(base);
    return gmp_c29x_i2c_wait_stop(base, timeout);

fail:
    I2C_sendStopCondition(base);
    return status;
}

static ec_gt gmp_c29x_i2c_read(
    uint32_t base,
    addr16_gt device_address,
    uint32_t address,
    size_gt address_length,
    data_gt *data,
    size_gt data_length,
    time_gt timeout)
{
    uint8_t address_bytes[4];
    size_gt index;
    ec_gt status;

    if ((base == 0U) || (address_length == 0U) || (address_length > 4U) ||
        (data == NULL) || (data_length == 0U) || (data_length > UINT16_MAX))
        return GMP_EC_INVALID_PARAM;

    status = gmp_c29x_i2c_wait_idle(base, timeout);
    if (status != GMP_EC_OK)
        return status;

    gmp_c29x_store_be(address_bytes, address, address_length);
    I2C_disableFIFO(base);
    I2C_clearStatus(base, I2C_STS_NO_ACK | I2C_STS_ARB_LOST | I2C_STS_REG_ACCESS_RDY);
    I2C_enableFIFO(base);
    I2C_setTargetAddress(base, (uint16_t)device_address);
    I2C_setDataCount(base, (uint16_t)address_length);
    I2C_setConfig(base, I2C_CONTROLLER_SEND_MODE);
    for (index = 0U; index < address_length; ++index)
        I2C_putData(base, address_bytes[index]);
    I2C_sendStartCondition(base);

    status = gmp_c29x_i2c_wait_ardy(base, timeout);
    if (status != GMP_EC_OK)
        goto fail;

    I2C_setDataCount(base, (uint16_t)data_length);
    I2C_setConfig(base, I2C_CONTROLLER_RECEIVE_MODE);
    I2C_sendStartCondition(base);
    I2C_sendStopCondition(base);
    for (index = 0U; index < data_length; ++index)
    {
        status = gmp_c29x_i2c_wait_rx_data(base, timeout);
        if (status != GMP_EC_OK)
            goto fail;
        data[index] = (data_gt)I2C_getData(base);
    }
    return gmp_c29x_i2c_wait_stop(base, timeout);

fail:
    I2C_sendStopCondition(base);
    return status;
}

ec_gt gmp_hal_iic_write_cmd(
    iic_halt iic, addr16_gt device_address, uint32_t command,
    size_gt command_length, time_gt timeout)
{
    uint8_t command_bytes[4];
    if ((command_length == 0U) || (command_length > 4U))
        return GMP_EC_INVALID_PARAM;
    gmp_c29x_store_be(command_bytes, command, command_length);
    return gmp_c29x_i2c_write((uint32_t)iic, device_address,
                              command_bytes, command_length, NULL, 0U, timeout);
}

ec_gt gmp_hal_iic_write_reg(
    iic_halt iic, addr16_gt device_address, addr32_gt register_address,
    size_gt address_length, uint32_t register_data, size_gt register_length,
    time_gt timeout)
{
    uint8_t bytes[8];
    if ((address_length == 0U) || (address_length > 4U) ||
        (register_length == 0U) || (register_length > 4U))
        return GMP_EC_INVALID_PARAM;
    gmp_c29x_store_be(bytes, (uint32_t)register_address, address_length);
    gmp_c29x_store_be(bytes + address_length, register_data, register_length);
    return gmp_c29x_i2c_write((uint32_t)iic, device_address, bytes,
                              address_length + register_length, NULL, 0U, timeout);
}

ec_gt gmp_hal_iic_write_mem(
    iic_halt iic, addr16_gt device_address, addr32_gt memory_address,
    size_gt address_length, const data_gt *memory, size_gt memory_length,
    time_gt timeout)
{
    uint8_t address_bytes[4];
    if ((address_length == 0U) || (address_length > 4U))
        return GMP_EC_INVALID_PARAM;
    gmp_c29x_store_be(address_bytes, (uint32_t)memory_address, address_length);
    return gmp_c29x_i2c_write((uint32_t)iic, device_address,
                              address_bytes, address_length,
                              memory, memory_length, timeout);
}

ec_gt gmp_hal_iic_read_reg(
    iic_halt iic, addr16_gt device_address, addr32_gt register_address,
    size_gt address_length, uint32_t *register_data, size_gt register_length,
    time_gt timeout)
{
    data_gt bytes[4];
    size_gt index;
    ec_gt status;
    uint32_t value = 0U;
    if ((register_data == NULL) || (register_length == 0U) || (register_length > 4U))
        return GMP_EC_INVALID_PARAM;
    status = gmp_c29x_i2c_read((uint32_t)iic, device_address,
                               (uint32_t)register_address, address_length,
                               bytes, register_length, timeout);
    if (status != GMP_EC_OK)
        return status;
    for (index = 0U; index < register_length; ++index)
        value = (value << 8U) | (uint32_t)bytes[index];
    *register_data = value;
    return GMP_EC_OK;
}

ec_gt gmp_hal_iic_read_mem(
    iic_halt iic, addr16_gt device_address, addr32_gt memory_address,
    size_gt address_length, data_gt *memory, size_gt memory_length,
    time_gt timeout)
{
    return gmp_c29x_i2c_read((uint32_t)iic, device_address,
                             (uint32_t)memory_address, address_length,
                             memory, memory_length, timeout);
}

static ec_gt gmp_c29x_spi_exchange(
    uint32_t base, const data_gt *transmit, data_gt *receive,
    size_gt length, time_gt timeout)
{
    size_gt index;
    if (length == 0U)
        return GMP_EC_OK;
    if ((base == 0U) || ((transmit == NULL) && (receive == NULL)))
        return GMP_EC_INVALID_PARAM;

    for (index = 0U; index < length; ++index)
    {
        uint32_t remaining_us = gmp_c29x_timeout_us((uint32_t)timeout);
        while (SPI_getTxFIFOStatus(base) == SPI_FIFO_TXFULL)
        {
            if (remaining_us == 0U)
                return GMP_EC_TIMEOUT;
            DEVICE_DELAY_US(1U);
            --remaining_us;
        }
        SPI_writeDataNonBlocking(
            base, (uint16_t)((transmit == NULL ? 0xFFU : transmit[index]) << 8U));

        remaining_us = gmp_c29x_timeout_us((uint32_t)timeout);
        while (SPI_getRxFIFOStatus(base) == SPI_FIFO_RXEMPTY)
        {
            if (remaining_us == 0U)
                return GMP_EC_TIMEOUT;
            DEVICE_DELAY_US(1U);
            --remaining_us;
        }
        if (receive != NULL)
            receive[index] = (data_gt)(SPI_readDataNonBlocking(base) & 0xFFU);
        else
            (void)SPI_readDataNonBlocking(base);
    }
    return GMP_EC_OK;
}

ec_gt gmp_hal_spi_bus_write(
    spi_halt spi, const data_gt *data, size_gt length, time_gt timeout)
{
    if ((data == NULL) && (length != 0U))
        return GMP_EC_INVALID_PARAM;
    return gmp_c29x_spi_exchange((uint32_t)spi, data, NULL, length, timeout);
}

ec_gt gmp_hal_spi_bus_read(
    spi_halt spi, data_gt *data, size_gt length, time_gt timeout)
{
    if ((data == NULL) && (length != 0U))
        return GMP_EC_INVALID_PARAM;
    return gmp_c29x_spi_exchange((uint32_t)spi, NULL, data, length, timeout);
}

ec_gt gmp_hal_spi_bus_transfer(
    spi_halt spi, const data_gt *transmit, data_gt *receive,
    size_gt length, time_gt timeout)
{
    if (((transmit == NULL) || (receive == NULL)) && (length != 0U))
        return GMP_EC_INVALID_PARAM;
    return gmp_c29x_spi_exchange((uint32_t)spi, transmit, receive, length, timeout);
}
