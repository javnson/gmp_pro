/**
 * @file rpi_pico_peripheral.c
 * @brief GMP GPIO, UART, I2C and SPI ports for the Raspberry Pi Pico SDK.
 */

#include <gmp_core.h>

#include <limits.h>

#include <hardware/structs/spi.h>

static bool pico_gpio_is_valid(gpio_halt gpio)
{
    uintptr_t encoded = (uintptr_t)gpio;
    return encoded != 0U && encoded <= (uintptr_t)NUM_BANK0_GPIOS;
}

static absolute_time_t pico_deadline_from_ms(time_gt timeout_ms)
{
    uint32_t bounded_ms = timeout_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)timeout_ms;
    return make_timeout_time_ms(bounded_ms);
}

static uint pico_deadline_remaining_us(absolute_time_t deadline)
{
    int64_t remaining = absolute_time_diff_us(get_absolute_time(), deadline);
    if (remaining <= 0)
    {
        return 0U;
    }
    return remaining > (int64_t)UINT_MAX ? UINT_MAX : (uint)remaining;
}

static bool pico_deadline_expired(absolute_time_t deadline)
{
    return time_reached(deadline);
}

/**
 * @brief Configure a Pico GPIO as a floating input or push-pull output.
 * @param gpio Encoded Pico GPIO handle.
 * @param dir Requested GMP GPIO direction.
 * @return GMP_EC_OK on success, otherwise GMP_EC_GENERAL_ERROR.
 */
ec_gt gmp_hal_gpio_set_dir(gpio_halt gpio, gpio_dir_et dir)
{
    if (!pico_gpio_is_valid(gpio))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    uint pin = GMP_RPI_PICO_GPIO_NUM(gpio);
    gpio_init(pin);
    gpio_set_dir(pin, dir == GMP_HAL_GPIO_DIR_OUT ? GPIO_OUT : GPIO_IN);
    gpio_disable_pulls(pin);
    return GMP_EC_OK;
}

/**
 * @brief Write a logical level to a Pico GPIO.
 * @param gpio Encoded Pico GPIO handle.
 * @param level GMP_HAL_GPIO_LOW or GMP_HAL_GPIO_HIGH.
 * @return GMP_EC_OK on success, otherwise GMP_EC_GENERAL_ERROR.
 */
ec_gt gmp_hal_gpio_write(gpio_halt gpio, fast_gt level)
{
    if (!pico_gpio_is_valid(gpio))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    gpio_put(GMP_RPI_PICO_GPIO_NUM(gpio), level == GMP_HAL_GPIO_HIGH);
    return GMP_EC_OK;
}

/**
 * @brief Read the logical level of a Pico GPIO.
 * @param gpio Encoded Pico GPIO handle.
 * @return The GPIO level, or low for an invalid handle.
 */
fast_gt gmp_hal_gpio_read(gpio_halt gpio)
{
    if (!pico_gpio_is_valid(gpio))
    {
        return 0;
    }
    return gpio_get(GMP_RPI_PICO_GPIO_NUM(gpio)) ? GMP_HAL_GPIO_HIGH : GMP_HAL_GPIO_LOW;
}

/**
 * @brief Test whether a UART transmitter is still shifting data.
 * @param uart Pico SDK UART instance.
 * @return Nonzero while the transmitter is busy.
 */
fast_gt gmp_hal_uart_is_tx_busy(uart_halt uart)
{
    if (uart == NULL)
    {
        return 0;
    }
    return (uart_get_hw(uart)->fr & UART_UARTFR_BUSY_BITS) != 0U;
}

/**
 * @brief Test whether a UART is busy.
 * @param uart Pico SDK UART instance.
 * @return Nonzero while the transmitter is busy.
 */
fast_gt gmp_hal_uart_is_busy(uart_halt uart)
{
    return gmp_hal_uart_is_tx_busy(uart);
}

/**
 * @brief Report whether at least one UART receive byte is available.
 * @param uart Pico SDK UART instance.
 * @return Zero when empty, otherwise one.
 */
size_gt gmp_hal_uart_get_rx_available(uart_halt uart)
{
    /* The PL011 exposes only empty/not-empty, not the exact FIFO level. */
    return uart != NULL && uart_is_readable(uart) ? 1U : 0U;
}

/**
 * @brief Write bytes to a UART within a total timeout.
 * @param uart Pico SDK UART instance.
 * @param data Source byte buffer.
 * @param length Number of bytes to write.
 * @param timeout Total timeout in milliseconds.
 * @return GMP_EC_OK, GMP_EC_TIMEOUT, or GMP_EC_GENERAL_ERROR.
 */
ec_gt gmp_hal_uart_write(uart_halt uart, const data_gt* data, size_gt length, uint32_t timeout)
{
    if (uart == NULL || (length != 0U && data == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }
    if (length == 0U)
    {
        return GMP_EC_OK;
    }

    absolute_time_t deadline = pico_deadline_from_ms(timeout);
    for (size_gt i = 0; i < length; ++i)
    {
        while (!uart_is_writable(uart))
        {
            if (pico_deadline_expired(deadline))
            {
                return GMP_EC_TIMEOUT;
            }
            tight_loop_contents();
        }
        uart_get_hw(uart)->dr = data[i];
    }
    return GMP_EC_OK;
}

/**
 * @brief Read bytes from a UART within a total timeout.
 * @param uart Pico SDK UART instance.
 * @param data Destination byte buffer.
 * @param length Number of bytes requested.
 * @param timeout Total timeout in milliseconds.
 * @param bytes_read Optional number of bytes actually received.
 * @return GMP_EC_OK, GMP_EC_TIMEOUT, or GMP_EC_GENERAL_ERROR.
 */
ec_gt gmp_hal_uart_read(uart_halt uart, data_gt* data, size_gt length, uint32_t timeout, size_gt* bytes_read)
{
    if (bytes_read != NULL)
    {
        *bytes_read = 0U;
    }
    if (uart == NULL || (length != 0U && data == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }
    if (length == 0U)
    {
        return GMP_EC_OK;
    }

    absolute_time_t deadline = pico_deadline_from_ms(timeout);
    size_gt count = 0U;
    while (count < length)
    {
        while (!uart_is_readable(uart))
        {
            if (pico_deadline_expired(deadline))
            {
                if (bytes_read != NULL)
                {
                    *bytes_read = count;
                }
                return GMP_EC_TIMEOUT;
            }
            tight_loop_contents();
        }
        data[count++] = (data_gt)uart_get_hw(uart)->dr;
    }

    if (bytes_read != NULL)
    {
        *bytes_read = count;
    }
    return GMP_EC_OK;
}

static ec_gt pico_i2c_result(int result, size_t expected)
{
    if (result == (int)expected)
    {
        return GMP_EC_OK;
    }
    if (result == PICO_ERROR_TIMEOUT)
    {
        return GMP_EC_TIMEOUT;
    }
    if (result == PICO_ERROR_GENERIC)
    {
        return GMP_EC_NACK;
    }
    return GMP_EC_GENERAL_ERROR;
}

static bool pico_i2c_args_valid(iic_halt i2c, addr16_gt address, size_gt length, const void* data)
{
    return i2c != NULL && address <= 0x7FU && length <= (size_gt)INT_MAX && (length == 0U || data != NULL);
}

static void pico_pack_be(uint8_t* destination, uint32_t value, size_gt length)
{
    for (size_gt i = 0U; i < length; ++i)
    {
        destination[i] = (uint8_t)(value >> (8U * (length - i - 1U)));
    }
}

/**
 * @brief Write a big-endian command value to a 7-bit I2C address.
 * @param i2c Pico SDK I2C instance.
 * @param dev_addr Right-aligned 7-bit device address.
 * @param cmd Command value.
 * @param cmd_len Command length from one to four bytes.
 * @param timeout Total timeout in milliseconds.
 * @return GMP_EC_OK or a mapped Pico SDK transfer error.
 */
ec_gt gmp_hal_iic_write_cmd(iic_halt i2c, addr16_gt dev_addr, uint32_t cmd, size_gt cmd_len, time_gt timeout)
{
    uint8_t buffer[4];
    if (!pico_i2c_args_valid(i2c, dev_addr, cmd_len, buffer) || cmd_len == 0U || cmd_len > sizeof(buffer))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    pico_pack_be(buffer, cmd, cmd_len);
    uint timeout_us = pico_deadline_remaining_us(pico_deadline_from_ms(timeout));
    int result = i2c_write_timeout_us(i2c, (uint8_t)dev_addr, buffer, cmd_len, false, timeout_us);
    return pico_i2c_result(result, cmd_len);
}

/**
 * @brief Write a big-endian value to an I2C register.
 * @param i2c Pico SDK I2C instance.
 * @param dev_addr Right-aligned 7-bit device address.
 * @param reg_addr Register address.
 * @param addr_len Register address length from zero to four bytes.
 * @param reg_data Register value.
 * @param reg_len Register value length from one to four bytes.
 * @param timeout Total timeout in milliseconds.
 * @return GMP_EC_OK or a mapped Pico SDK transfer error.
 */
ec_gt gmp_hal_iic_write_reg(iic_halt i2c, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len,
                            uint32_t reg_data, size_gt reg_len, time_gt timeout)
{
    uint8_t buffer[8];
    size_gt total = addr_len + reg_len;
    if (!pico_i2c_args_valid(i2c, dev_addr, total, buffer) || addr_len > 4U || reg_len == 0U || reg_len > 4U)
    {
        return GMP_EC_GENERAL_ERROR;
    }

    pico_pack_be(buffer, reg_addr, addr_len);
    pico_pack_be(buffer + addr_len, reg_data, reg_len);
    uint timeout_us = pico_deadline_remaining_us(pico_deadline_from_ms(timeout));
    int result = i2c_write_timeout_us(i2c, (uint8_t)dev_addr, buffer, total, false, timeout_us);
    return pico_i2c_result(result, total);
}

/**
 * @brief Write a byte buffer after an I2C memory address in one transaction.
 * @param i2c Pico SDK I2C instance.
 * @param dev_addr Right-aligned 7-bit device address.
 * @param mem_addr Memory address.
 * @param addr_len Memory address length from zero to four bytes.
 * @param mem Source byte buffer.
 * @param mem_len Number of bytes to write.
 * @param timeout Total timeout in milliseconds.
 * @return GMP_EC_OK or a mapped Pico SDK transfer error.
 */
ec_gt gmp_hal_iic_write_mem(iic_halt i2c, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len,
                            const data_gt* mem, size_gt mem_len, time_gt timeout)
{
    if (addr_len > 4U || mem_len > (size_gt)(SIZE_MAX - addr_len))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    size_t total = (size_t)addr_len + (size_t)mem_len;
    if (total > (size_t)INT_MAX || i2c == NULL || dev_addr > 0x7FU || (mem_len != 0U && mem == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }
    if (total == 0U)
    {
        return GMP_EC_OK;
    }

    uint8_t* buffer = (uint8_t*)malloc(total);
    if (buffer == NULL)
    {
        return GMP_EC_GENERAL_ERROR;
    }
    pico_pack_be(buffer, mem_addr, addr_len);
    if (mem_len != 0U)
    {
        memcpy(buffer + addr_len, mem, mem_len);
    }

    uint timeout_us = pico_deadline_remaining_us(pico_deadline_from_ms(timeout));
    int result = i2c_write_timeout_us(i2c, (uint8_t)dev_addr, buffer, total, false, timeout_us);
    free(buffer);
    return pico_i2c_result(result, total);
}

/**
 * @brief Read bytes from an I2C memory address using a repeated start.
 * @param i2c Pico SDK I2C instance.
 * @param dev_addr Right-aligned 7-bit device address.
 * @param mem_addr Memory address.
 * @param addr_len Memory address length from zero to four bytes.
 * @param mem Destination byte buffer.
 * @param mem_len Number of bytes to read.
 * @param timeout Total timeout in milliseconds.
 * @return GMP_EC_OK or a mapped Pico SDK transfer error.
 */
ec_gt gmp_hal_iic_read_mem(iic_halt i2c, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len,
                           data_gt* mem, size_gt mem_len, time_gt timeout)
{
    if (addr_len > 4U || !pico_i2c_args_valid(i2c, dev_addr, mem_len, mem))
    {
        return GMP_EC_GENERAL_ERROR;
    }
    if (mem_len == 0U)
    {
        return GMP_EC_OK;
    }

    absolute_time_t deadline = pico_deadline_from_ms(timeout);
    if (addr_len != 0U)
    {
        uint8_t address_buffer[4];
        pico_pack_be(address_buffer, mem_addr, addr_len);
        uint remaining_us = pico_deadline_remaining_us(deadline);
        int result = i2c_write_timeout_us(i2c, (uint8_t)dev_addr, address_buffer, addr_len, true, remaining_us);
        ec_gt status = pico_i2c_result(result, addr_len);
        if (status != GMP_EC_OK)
        {
            return status;
        }
    }

    uint remaining_us = pico_deadline_remaining_us(deadline);
    int result = i2c_read_timeout_us(i2c, (uint8_t)dev_addr, mem, mem_len, false, remaining_us);
    return pico_i2c_result(result, mem_len);
}

/**
 * @brief Read a big-endian value from an I2C register.
 * @param i2c Pico SDK I2C instance.
 * @param dev_addr Right-aligned 7-bit device address.
 * @param reg_addr Register address.
 * @param addr_len Register address length from zero to four bytes.
 * @param reg_data_ret Destination for the decoded register value.
 * @param reg_len Register value length from one to four bytes.
 * @param timeout Total timeout in milliseconds.
 * @return GMP_EC_OK or a mapped Pico SDK transfer error.
 */
ec_gt gmp_hal_iic_read_reg(iic_halt i2c, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len,
                           uint32_t* reg_data_ret, size_gt reg_len, time_gt timeout)
{
    uint8_t buffer[4];
    if (reg_data_ret == NULL || reg_len == 0U || reg_len > sizeof(buffer))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    ec_gt status = gmp_hal_iic_read_mem(i2c, dev_addr, reg_addr, addr_len, buffer, reg_len, timeout);
    if (status != GMP_EC_OK)
    {
        return status;
    }

    uint32_t value = 0U;
    for (size_gt i = 0U; i < reg_len; ++i)
    {
        value = (value << 8U) | buffer[i];
    }
    *reg_data_ret = value;
    return GMP_EC_OK;
}

static ec_gt pico_spi_transfer(spi_halt spi, const data_gt* tx, data_gt* rx, size_gt length, time_gt timeout)
{
    if (spi == NULL || (length != 0U && tx == NULL && rx == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }
    if (length == 0U)
    {
        return GMP_EC_OK;
    }

    while (spi_is_readable(spi))
    {
        (void)spi_get_hw(spi)->dr;
    }

    absolute_time_t deadline = pico_deadline_from_ms(timeout);
    size_gt tx_remaining = length;
    size_gt rx_remaining = length;
    const size_gt fifo_depth = 8U;

    while (tx_remaining != 0U || rx_remaining != 0U)
    {
        if (tx_remaining != 0U && spi_is_writable(spi) && rx_remaining < tx_remaining + fifo_depth)
        {
            size_gt index = length - tx_remaining;
            spi_get_hw(spi)->dr = tx == NULL ? 0U : tx[index];
            --tx_remaining;
        }
        if (rx_remaining != 0U && spi_is_readable(spi))
        {
            size_gt index = length - rx_remaining;
            uint8_t value = (uint8_t)spi_get_hw(spi)->dr;
            if (rx != NULL)
            {
                rx[index] = value;
            }
            --rx_remaining;
        }
        if (pico_deadline_expired(deadline))
        {
            return GMP_EC_TIMEOUT;
        }
        tight_loop_contents();
    }

    while (spi_is_busy(spi))
    {
        if (pico_deadline_expired(deadline))
        {
            return GMP_EC_TIMEOUT;
        }
        tight_loop_contents();
    }
    return GMP_EC_OK;
}

/**
 * @brief Write bytes over an initialized Pico SPI bus.
 * @param spi Pico SDK SPI instance configured for 8-bit transfers.
 * @param tx_buf Source byte buffer.
 * @param len Number of bytes to write.
 * @param timeout Total timeout in milliseconds.
 * @return GMP_EC_OK, GMP_EC_TIMEOUT, or GMP_EC_GENERAL_ERROR.
 */
ec_gt gmp_hal_spi_bus_write(spi_halt spi, const data_gt* tx_buf, size_gt len, time_gt timeout)
{
    if (len != 0U && tx_buf == NULL)
    {
        return GMP_EC_GENERAL_ERROR;
    }
    return pico_spi_transfer(spi, tx_buf, NULL, len, timeout);
}

/**
 * @brief Read bytes over an initialized Pico SPI bus using zero fill bytes.
 * @param spi Pico SDK SPI instance configured for 8-bit transfers.
 * @param rx_buf Destination byte buffer.
 * @param len Number of bytes to read.
 * @param timeout Total timeout in milliseconds.
 * @return GMP_EC_OK, GMP_EC_TIMEOUT, or GMP_EC_GENERAL_ERROR.
 */
ec_gt gmp_hal_spi_bus_read(spi_halt spi, data_gt* rx_buf, size_gt len, time_gt timeout)
{
    if (len != 0U && rx_buf == NULL)
    {
        return GMP_EC_GENERAL_ERROR;
    }
    return pico_spi_transfer(spi, NULL, rx_buf, len, timeout);
}

/**
 * @brief Perform a full-duplex transfer over an initialized Pico SPI bus.
 * @param spi Pico SDK SPI instance configured for 8-bit transfers.
 * @param tx_buf Source byte buffer.
 * @param rx_buf Destination byte buffer.
 * @param len Number of bytes to transfer.
 * @param timeout Total timeout in milliseconds.
 * @return GMP_EC_OK, GMP_EC_TIMEOUT, or GMP_EC_GENERAL_ERROR.
 */
ec_gt gmp_hal_spi_bus_transfer(spi_halt spi, const data_gt* tx_buf, data_gt* rx_buf, size_gt len, time_gt timeout)
{
    if (len != 0U && (tx_buf == NULL || rx_buf == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }
    return pico_spi_transfer(spi, tx_buf, rx_buf, len, timeout);
}

#if defined(SPECIFY_ENABLE_GMP_CAN_SERVICE)
#include <core/dev/can/can_hook.h>

/**
 * @brief Forward a CAN capability query to the optional extension hook.
 * @details Pico microcontrollers have no native CAN controller.
 */
ec_gt gmp_hal_can_get_capabilities(can_halt hcan, gmp_can_capabilities_t* capabilities)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_get_capabilities)(hcan, capabilities);
#else
    GMP_UNUSED_VAR(hcan); GMP_UNUSED_VAR(capabilities); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Forward CAN configuration to the optional extension hook. */
ec_gt gmp_hal_can_configure(can_halt hcan, const gmp_can_config_t* config)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_configure)(hcan, config);
#else
    GMP_UNUSED_VAR(hcan); GMP_UNUSED_VAR(config); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Forward CAN start to the optional extension hook. */
ec_gt gmp_hal_can_start(can_halt hcan)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_start)(hcan);
#else
    GMP_UNUSED_VAR(hcan); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Forward CAN stop to the optional extension hook. */
ec_gt gmp_hal_can_stop(can_halt hcan)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_stop)(hcan);
#else
    GMP_UNUSED_VAR(hcan); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Forward an asynchronous CAN transmission to the optional extension hook. */
ec_gt gmp_hal_can_submit_tx(can_halt hcan, const gmp_can_frame_t* frame, uint32_t token)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_submit_tx)(hcan, frame, token);
#else
    GMP_UNUSED_VAR(hcan); GMP_UNUSED_VAR(frame); GMP_UNUSED_VAR(token); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Forward CAN filter configuration to the optional extension hook. */
ec_gt gmp_hal_can_set_filter(can_halt hcan, uint16_t slot, const gmp_can_hw_filter_t* filter)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_set_filter)(hcan, slot, filter);
#else
    GMP_UNUSED_VAR(hcan); GMP_UNUSED_VAR(slot); GMP_UNUSED_VAR(filter); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Forward a CAN state query to the optional extension hook. */
ec_gt gmp_hal_can_get_state(can_halt hcan, gmp_can_state_t* state)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_get_state)(hcan, state);
#else
    GMP_UNUSED_VAR(hcan); GMP_UNUSED_VAR(state); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Forward CAN bus recovery to the optional extension hook. */
ec_gt gmp_hal_can_recover(can_halt hcan)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_recover)(hcan);
#else
    GMP_UNUSED_VAR(hcan); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Service the optional CAN extension hook. */
void gmp_hal_can_service(can_halt hcan)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    GMP_CAN_HOOK_SYMBOL(_hal_can_service)(hcan);
#else
    GMP_UNUSED_VAR(hcan);
#endif
}
#endif /* SPECIFY_ENABLE_GMP_CAN_SERVICE */
