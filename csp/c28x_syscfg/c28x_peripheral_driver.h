

#ifndef _FILE_C28X_PERIPHERAL_DRIVER_H_
#define _FILE_C28X_PERIPHERAL_DRIVER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// debug port
extern uart_halt debug_uart;

//////////////////////////////////////////////////////////////////////////
// SPI interface

//// This is a synchronize function
//GMP_STATIC_INLINE
//void gmp_hal_spi_write(spi_halt spi, const data_gt *data, size_gt length)
//{
//    size_gt i;
//
//    for (i = 0; i < length; ++i)
//        SPI_writeDataBlockingFIFO(spi, data[i]);
//}
//
//// This is a asynchronize function
//// GMP_STATIC_INLINE
//// void gmp_hal_spi_write_async(spi_halt spi, const data_gt *data, size_gt length)
////{
////    for (size_gt i = 0; i < length; ++i)
////        SPI_writeDataBlockingFIFO(spi, data[i]);
////}
//
//// This is a synchronize function
//GMP_STATIC_INLINE
//size_gt gmp_hal_spi_read(spi_halt spi, data_gt *data, size_gt length)
//{
//    size_gt i;
//
//    for (i = 0; i < length; ++i)
//        data[i] = SPI_readDataBlockingFIFO(spi);
//
//    return length;
//}
//
//GMP_STATIC_INLINE
//size_gt gmp_hal_spi_read_write(spi_halt spi, data_gt *data_in, data_gt *data_out, size_gt length)
//{
//    size_gt i;
//    for (i = 0; i < length; ++i)
//    {
//        SPI_writeDataBlockingFIFO(spi, data_in[i]);
//        data_out[i] = SPI_readDataBlockingFIFO(spi);
//    }
//    return length;
//}
//
//// This is a asynchronize function
//// size_gt gmp_hal_spi_read_async(spi_halt spi, data_gt *data, size_gt length);
//
//// Wait till transmit/receive complete
//GMP_STATIC_INLINE
//fast_gt gmp_hal_spi_is_busy(spi_halt spi)
//{
//    return SPI_isBusy(spi);
//}

//////////////////////////////////////////////////////////////////////////
// UART interface
GMP_STATIC_INLINE
void gmp_hal_uart_write(uart_halt uart, const data_gt *data, size_gt length)
{
    size_gt i;

    for (i = 0; i < length; ++i)
        SCI_writeCharBlockingFIFO(uart, data[i]);
}

// void gmp_hal_uart_write_async(uart_halt uart, const data_gt *data, size_gt length)
//{

//}
GMP_STATIC_INLINE
size_gt gmp_hal_uart_read(uart_halt uart, data_gt *data, size_gt length)
{
    size_gt i;

    for (i = 0; i < length; ++i)
        data[i] = SCI_readCharBlockingFIFO(uart);

    return length;
}

// size_gt gmp_hal_uart_read_async(uart_halt uart, data_gt *data, size_gt length);

// wait till transmit/receive complete.
GMP_STATIC_INLINE
fast_gt gmp_hal_uart_is_busy(uart_halt uart)
{
    return SCI_isTransmitterBusy(uart);
}


#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_C28X_PERIPHERAL_DRIVER_H_
