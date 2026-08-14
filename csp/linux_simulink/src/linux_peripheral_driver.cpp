/**
 * @file linux_peripheral_driver.cpp
 * @brief Safe hosted stubs for peripheral interfaces not modeled by Linux SIL.
 */

#include <gmp_core.h>

/** @brief Accept a GPIO direction request in the hosted stub backend. */
ec_gt gmp_hal_gpio_set_dir(gpio_halt gpio, gpio_dir_et direction)
{
    GMP_UNUSED_VAR(gpio);
    GMP_UNUSED_VAR(direction);
    return GMP_EC_OK;
}

/** @brief Accept a GPIO output request in the hosted stub backend. */
ec_gt gmp_hal_gpio_write(gpio_halt gpio, fast_gt level)
{
    GMP_UNUSED_VAR(gpio);
    GMP_UNUSED_VAR(level);
    return GMP_EC_OK;
}

/** @brief Return the default low level for an unmodeled hosted GPIO. */
fast_gt gmp_hal_gpio_read(gpio_halt gpio)
{
    GMP_UNUSED_VAR(gpio);
    return 0;
}

/** @brief Report that the hosted UART transmitter is idle. */
fast_gt gmp_hal_uart_is_tx_busy(uart_halt uart)
{
    GMP_UNUSED_VAR(uart);
    return 0;
}

/** @brief Report that an unmodeled hosted UART has no received data. */
size_gt gmp_hal_uart_get_rx_available(uart_halt uart)
{
    GMP_UNUSED_VAR(uart);
    return 0;
}

/** @brief Validate and accept a hosted UART write request. */
ec_gt gmp_hal_uart_write(uart_halt uart, const byte_gt* data, size_gt length, uint32_t timeout)
{
    GMP_UNUSED_VAR(timeout);
    return (uart == NULL || data == NULL || length == 0) ? GMP_EC_GENERAL_ERROR : GMP_EC_OK;
}

/** @brief Validate a hosted UART read request and report no received bytes. */
ec_gt gmp_hal_uart_read(uart_halt uart, byte_gt* data, size_gt length, uint32_t timeout, size_gt* bytes_read)
{
    GMP_UNUSED_VAR(timeout);
    if (bytes_read != NULL)
        *bytes_read = 0;
    return (uart == NULL || data == NULL || length == 0) ? GMP_EC_GENERAL_ERROR : GMP_EC_OK;
}

/** @brief Accept an unmodeled hosted I2C command write. */
ec_gt gmp_hal_iic_write_cmd(iic_halt handle, addr16_gt device_address, uint32_t command, size_gt command_length,
                            time_gt timeout)
{
    GMP_UNUSED_VAR(handle);
    GMP_UNUSED_VAR(device_address);
    GMP_UNUSED_VAR(command);
    GMP_UNUSED_VAR(command_length);
    GMP_UNUSED_VAR(timeout);
    return GMP_EC_OK;
}

/** @brief Accept an unmodeled hosted I2C register write. */
ec_gt gmp_hal_iic_write_reg(iic_halt handle, addr16_gt device_address, addr32_gt register_address,
                            size_gt address_length, uint32_t register_data, size_gt register_length, time_gt timeout)
{
    GMP_UNUSED_VAR(handle);
    GMP_UNUSED_VAR(device_address);
    GMP_UNUSED_VAR(register_address);
    GMP_UNUSED_VAR(address_length);
    GMP_UNUSED_VAR(register_data);
    GMP_UNUSED_VAR(register_length);
    GMP_UNUSED_VAR(timeout);
    return GMP_EC_OK;
}

/** @brief Validate and accept an unmodeled hosted I2C memory write. */
ec_gt gmp_hal_iic_write_mem(iic_halt handle, addr16_gt device_address, addr32_gt memory_address,
                            size_gt address_length, const byte_gt* memory, size_gt memory_length, time_gt timeout)
{
    GMP_UNUSED_VAR(handle);
    GMP_UNUSED_VAR(device_address);
    GMP_UNUSED_VAR(memory_address);
    GMP_UNUSED_VAR(address_length);
    GMP_UNUSED_VAR(memory_length);
    GMP_UNUSED_VAR(timeout);
    return memory == NULL ? GMP_EC_GENERAL_ERROR : GMP_EC_OK;
}

/** @brief Validate an unmodeled hosted I2C register read request. */
ec_gt gmp_hal_iic_read_reg(iic_halt handle, addr16_gt device_address, addr32_gt register_address,
                           size_gt address_length, uint32_t* register_data, size_gt register_length, time_gt timeout)
{
    GMP_UNUSED_VAR(handle);
    GMP_UNUSED_VAR(device_address);
    GMP_UNUSED_VAR(register_address);
    GMP_UNUSED_VAR(address_length);
    GMP_UNUSED_VAR(register_length);
    GMP_UNUSED_VAR(timeout);
    return register_data == NULL ? GMP_EC_GENERAL_ERROR : GMP_EC_OK;
}

/** @brief Validate an unmodeled hosted I2C memory read request. */
ec_gt gmp_hal_iic_read_mem(iic_halt handle, addr16_gt device_address, addr32_gt memory_address,
                           size_gt address_length, byte_gt* memory, size_gt memory_length, time_gt timeout)
{
    GMP_UNUSED_VAR(handle);
    GMP_UNUSED_VAR(device_address);
    GMP_UNUSED_VAR(memory_address);
    GMP_UNUSED_VAR(address_length);
    GMP_UNUSED_VAR(memory_length);
    GMP_UNUSED_VAR(timeout);
    return memory == NULL ? GMP_EC_GENERAL_ERROR : GMP_EC_OK;
}

/** @brief Validate and accept an unmodeled hosted SPI write. */
ec_gt gmp_hal_spi_bus_write(spi_halt spi, const byte_gt* transmit_buffer, size_gt length, time_gt timeout)
{
    GMP_UNUSED_VAR(timeout);
    return (spi == NULL || transmit_buffer == NULL || length == 0) ? GMP_EC_GENERAL_ERROR : GMP_EC_OK;
}

/** @brief Validate an unmodeled hosted SPI read request. */
ec_gt gmp_hal_spi_bus_read(spi_halt spi, byte_gt* receive_buffer, size_gt length, time_gt timeout)
{
    GMP_UNUSED_VAR(timeout);
    return (spi == NULL || receive_buffer == NULL || length == 0) ? GMP_EC_GENERAL_ERROR : GMP_EC_OK;
}

/** @brief Validate an unmodeled hosted full-duplex SPI transfer. */
ec_gt gmp_hal_spi_bus_transfer(spi_halt spi, const byte_gt* transmit_buffer, byte_gt* receive_buffer,
                               size_gt length, time_gt timeout)
{
    GMP_UNUSED_VAR(timeout);
    return (spi == NULL || transmit_buffer == NULL || receive_buffer == NULL || length == 0)
               ? GMP_EC_GENERAL_ERROR
               : GMP_EC_OK;
}
