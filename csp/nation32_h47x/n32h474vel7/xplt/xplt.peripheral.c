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

static gmp_gpio_n32h47x_t user_led_object = {GPIOA, GPIO_PIN_3};

//=================================================================================================
// peripheral setup function


// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
    /* D1 is the red, active-high LED on PA3. */
    user_led = &user_led_object;
    (void)gmp_hal_gpio_set_dir(user_led, GMP_HAL_GPIO_DIR_OUT);
    (void)gmp_hal_gpio_write(user_led, GMP_HAL_GPIO_LOW);

    /* USART1 is wired to NS-LINK through J5: PA9 TX, PA10 RX, 115200 8N1. */
    debug_uart = USART1;
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
    /* Send protocol header first. */
    (void)gmp_hal_uart_write(debug_uart, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl), gmp_dev_dl_get_tx_hw_hdr_size(&dl), 10U);

    /* Then send the payload when the current frame has one. */
    if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0U)
    {
        (void)gmp_hal_uart_write(debug_uart, gmp_dev_dl_get_tx_hw_pld_ptr(&dl),
                                 gmp_dev_dl_get_tx_hw_pld_size(&dl), 10U);
    }
}

void flush_dl_rx_buffer()
{
    data_gt rx_buf[16];
    size_gt count = 0U;

    while ((count < (size_gt)sizeof(rx_buf)) && (gmp_hal_uart_get_rx_available(debug_uart) > 0U))
    {
        rx_buf[count++] = (data_gt)USART_ReceiveData(debug_uart);
    }

    if (count > 0U)
        gmp_dev_dl_push_str(&dl, rx_buf, count);
}
