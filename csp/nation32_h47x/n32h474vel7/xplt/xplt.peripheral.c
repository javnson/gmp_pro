//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all definitions of peripheral objects in this file.
//
// User should implement the peripheral objects initialization in setup_peripheral function.
//
// This file is platform-related.
//

// GMP basic core header
#include <gmp_core.h>

#include "user_main.h"
#include <xplt.peripheral.h>


//=================================================================================================
// definitions of peripheral

//=================================================================================================
// peripheral setup function


// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
    // Setup Debug Uart
    //debug_uart = IRIS_UART_USB_BASE;
}

//=================================================================================================
// ADC Interrupt ISR and controller related function

// ADC interrupt
//interrupt void MainISR(void)
//{
//    //
//    // call GMP ISR  Controller operation callback function
//    //
//    gmp_base_ctl_step();
//}

//=================================================================================================
// communication functions and interrupt functions here

void flush_dl_tx_buffer()
{
    //// Send head
    //gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl), gmp_dev_dl_get_tx_hw_hdr_size(&dl), 10);

    //// Send data body, if necessary
    //if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0)
    //{
    //    gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_pld_ptr(&dl), gmp_dev_dl_get_tx_hw_pld_size(&dl),
    //                       10);
    //}
}

void flush_dl_rx_buffer()
{
    //uint16_t fifoLevel;
    //data_gt rxBuf[ISR_LOCAL_BUF_SIZE];

    //// read all FIFO messages
    //fifoLevel = SCI_getRxFIFOStatus(IRIS_UART_USB_BASE);

    //if (fifoLevel > 0)
    //{
    //    SCI_readCharArray(IRIS_UART_USB_BASE, (uint16_t*)rxBuf, fifoLevel);

    //    // Lock-free ring queue pushed into the protocol stack (very fast, O(1))
    //    gmp_dev_dl_push_str(&dl, rxBuf, fifoLevel);
    //}
}
