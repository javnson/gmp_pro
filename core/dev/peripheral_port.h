
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

void gmp_hal_iic_write_cmd(iic_halt* h, addr16_gt dev_addr, uint32_t cmd, size_gt cmd_len);

void gmp_hal_iic_write(iic_halt iic, fast16_gt dev_addr, const data_gt *data, size_gt length);

void gmp_hal_iic_write_async(iic_halt iic, fast16_gt dev_addr, const data_gt *data, size_gt length);

void gmp_hal_iic_write_mem(iic_halt iic, fast16_gt dev_addr, fast32_gt mem_addr, const data_gt *data, size_gt length);

void gmp_hal_iic_write_mem_async(iic_halt iic, fast16_gt dev_addr, fast32_gt mem_addr, fast_gt mem_length,
                                 const data_gt *data, size_gt length);

size_gt gmp_hal_iic_read(iic_halt iic, fast16_gt dev_addr, data_gt *data, size_gt length);

// size_gt gmp_hal_iic_read_async(iic_halt iic, fast16_gt dev_addr, data_gt *data, size_gt length);

size_gt gmp_hal_iic_read_mem(iic_halt iic, fast16_gt dev_addr, fast32_gt mem_addr, fast_gt mem_length, data_gt *data,
                             size_gt length);

// ssize_gt gmp_hal_iic_read_mem_async(iic_halt iic, fast16_gt dev_addr, fast32_gt mem_addr, fast_gt mem_length,
//                                    data_gt *data, size_gt length);

// wait till transmit/receive complete
fast_gt gmp_hal_iic_is_busy(iic_halt iic);

//////////////////////////////////////////////////////////////////////////
// CAN interface

// void gmp_hal_can_write(can_halt can);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PERIPHERAL_PORT_H_
