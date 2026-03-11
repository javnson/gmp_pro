
#ifndef _FILE_PERIPHERAL_PORT_H_
#define _FILE_PERIPHERAL_PORT_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// typedef struct _tag_ring_buffer_t
//{
//     // buffer
//     data_gt *buffer;

//    // the position to read
//    size_gt read_pos;

//    // the write position
//    size_gt write_pos;

//    // length of buffer, the maximum length is `length - 1`
//    size_gt length;

//    fast_gt full;

//} ring_buffer_t;

//////////////////////////////////////////////////////////////////////////
// Virtual Port

// void gmp_hal_make_half_duplex_if(half_duplex_ift *if, data_gt *data, size_gt length);

//////////////////////////////////////////////////////////////////////////
// Async Port

// Write data to the ring buffer
GMP_STATIC_INLINE
void gmp_hal_buffer_write(ringbuf_t *ring, const data_gt *data, size_gt length)
{
    ringbuf_put_array(ring, data, length);
}

// Read data from the ring buffer
GMP_STATIC_INLINE
size_gt gmp_hal_buffer_read(ringbuf_t *ring, data_gt *data, size_gt length)
{
    size_gt max_length = ringbuf_get_valid_size(ring);
    if (max_length == 0)
        return 0;

    ringbuf_get_array(ring, data, max_length);
    return max_length
}

// General Peripheral Prototype functions
// These functions may implement by CSP.

//////////////////////////////////////////////////////////////////////////
// SPI interface

// This is a synchronize function
void gmp_hal_spi_write(spi_halt spi, const data_gt *data, size_gt length);

// This is a asynchronize function
void gmp_hal_spi_write_async(spi_halt spi, const data_gt *data, size_gt length);

// This is a synchronize function
size_gt gmp_hal_spi_read(spi_halt spi, data_gt *data, size_gt length);

// This is a asynchronize function
// size_gt gmp_hal_spi_read_async(spi_halt spi, data_gt *data, size_gt length);

// Wait till transmit/receive complete
fast_gt gmp_hal_spi_is_busy(spi_halt spi);

//////////////////////////////////////////////////////////////////////////
// UART interface

void gmp_hal_uart_write(uart_halt uart, const data_gt *data, size_gt length);

void gmp_hal_uart_write_async(uart_halt uart, const data_gt *data, size_gt length);

size_gt gmp_hal_uart_read(uart_halt uart, data_gt *data, size_gt length);

// size_gt gmp_hal_uart_read_async(uart_halt uart, data_gt *data, size_gt length);

// wait till transmit/receive complete.
fast_gt gmp_hal_uart_is_busy(uart_halt uart);

//////////////////////////////////////////////////////////////////////////
// IIC interface

/**
 * @brief   Write a command to the I2C device without register address.
 * * @param[in] h         I2C hardware handle.
 * @param[in] dev_addr  7-bit right-aligned device address.
 * @param[in] cmd       The command value to be sent.
 * @param[in] cmd_len   Length of the command in bytes.
 * @param[in] timeout   Maximum timeout in milliseconds.
 * * @return  ec_gt       Error code (GMP_EC_OK on success).
 */
ec_gt gmp_hal_iic_write_cmd(iic_halt h, addr16_gt dev_addr, uint32_t cmd, size_gt cmd_len, time_gt timeout);

/**
 * @brief   Write a single value to a specific register of the I2C device.
 * * @param[in] h         I2C hardware handle.
 * @param[in] dev_addr  7-bit right-aligned device address.
 * @param[in] reg_addr  Register address.
 * @param[in] addr_len  Length of the register address in bytes (e.g., 1 or 2).
 * @param[in] reg_data  Data to write into the register.
 * @param[in] reg_len   Length of the data in bytes.
 * @param[in] timeout   Maximum timeout in milliseconds.
 * * @return  ec_gt       Error code.
 */
ec_gt gmp_hal_iic_write_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t reg_data,
                            size_gt reg_len, time_gt timeout);

/**
 * @brief   Write a continuous memory block to the I2C device.
 * * @param[in] h         I2C hardware handle.
 * @param[in] dev_addr  7-bit right-aligned device address.
 * @param[in] mem_addr  Starting memory/register address.
 * @param[in] addr_len  Length of the memory address in bytes.
 * @param[in] mem       Pointer to the data buffer.
 * @param[in] mem_len   Number of data_gt elements to write.
 * @param[in] timeout   Maximum timeout in milliseconds.
 * * @return  ec_gt       Error code.
 */
ec_gt gmp_hal_iic_write_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, const data_gt* mem,
                            size_gt mem_len, time_gt timeout);

/**
 * @brief   Read a single value from a specific register of the I2C device.
 * * @param[in]  h            I2C hardware handle.
 * @param[in]  dev_addr     7-bit right-aligned device address.
 * @param[in]  reg_addr     Register address.
 * @param[in]  addr_len     Length of the register address in bytes.
 * @param[out] reg_data_ret Pointer to store the read data.
 * @param[in]  reg_len      Length of the data to read in bytes.
 * @param[in]  timeout      Maximum timeout in milliseconds.
 * * @return  ec_gt           Error code.
 */
ec_gt gmp_hal_iic_read_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t* reg_data_ret,
                           size_gt reg_len, time_gt timeout);

/**
 * @brief   Read a continuous memory block from the I2C device.
 * * @param[in]  h         I2C hardware handle.
 * @param[in]  dev_addr  7-bit right-aligned device address.
 * @param[in]  mem_addr  Starting memory/register address.
 * @param[in]  addr_len  Length of the memory address in bytes.
 * @param[out] mem       Pointer to the data buffer to store results.
 * @param[in]  mem_len   Number of data_gt elements to read.
 * @param[in]  timeout   Maximum timeout in milliseconds.
 * * @return  ec_gt        Error code.
 */
ec_gt gmp_hal_iic_read_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, data_gt* mem,
                           size_gt mem_len, time_gt timeout);


//////////////////////////////////////////////////////////////////////////
// CAN interface

// void gmp_hal_can_write(can_halt can);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PERIPHERAL_PORT_H_
