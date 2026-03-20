/**
 * @file windows_rt_tracer.cpp
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

fast_gt gmp_hal_uart_is_tx_busy(uart_halt uart)
{
    if (uart == NULL)
        return 0;

    return 0;
}

size_gt gmp_hal_uart_get_rx_available(uart_halt uart)
{
    if (uart == NULL)
        return 0;

    return 0; // No data available
}

/* ========================================================================= */
/* ==================== SAFE BLOCKING I/O FUNCTIONS ======================== */
/* ========================================================================= */

ec_gt gmp_hal_uart_write(uart_halt uart, const data_gt* data, size_gt length, uint32_t timeout)
{
    if (uart == NULL || data == NULL || length == 0)
        return GMP_EC_GENERAL_ERROR;

    return GMP_EC_OK;
}

ec_gt gmp_hal_uart_read(uart_halt uart, data_gt* data, size_gt length, uint32_t timeout, size_gt* bytes_read)
{
    if (uart == NULL || data == NULL || length == 0)
        return GMP_EC_GENERAL_ERROR;

    return GMP_EC_OK;
}

ec_gt gmp_hal_iic_write_cmd(iic_halt h, addr16_gt dev_addr, uint32_t cmd, size_gt cmd_len, time_gt timeout)
{
    return GMP_EC_OK;
}

ec_gt gmp_hal_iic_write_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t reg_data,
                            size_gt reg_len, time_gt timeout)
{
    return GMP_EC_OK;
}

ec_gt gmp_hal_iic_write_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, const data_gt* mem,
                            size_gt mem_len, time_gt timeout)
{
    return GMP_EC_OK;
}

ec_gt gmp_hal_iic_read_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t* reg_data_ret,
                           size_gt reg_len, time_gt timeout)
{
    return GMP_EC_OK;
}

ec_gt gmp_hal_iic_read_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, data_gt* mem,
                           size_gt mem_len, time_gt timeout)
{
    if (mem == NULL)
    {
        return GMP_EC_GENERAL_ERROR;
    }

    return GMP_EC_OK;
}

ec_gt gmp_hal_spi_bus_write(spi_halt hspi, const data_gt* tx_buf, size_gt len, time_gt timeout)
{
    if ((hspi == NULL) || (tx_buf == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    return GMP_EC_OK;
}

ec_gt gmp_hal_spi_bus_read(spi_halt hspi, data_gt* rx_buf, size_gt len, time_gt timeout)
{
    if ((hspi == NULL) || (rx_buf == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    return GMP_EC_OK;
}

ec_gt gmp_hal_spi_bus_transfer(spi_halt hspi, const data_gt* tx_buf, data_gt* rx_buf, size_gt len, time_gt timeout)
{
    if ((hspi == NULL) || (tx_buf == NULL) || (rx_buf == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    return GMP_EC_OK;
}
