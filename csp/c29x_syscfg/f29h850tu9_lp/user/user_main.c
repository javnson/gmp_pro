/**
 * @file user_main.c
 * @brief LaunchPad LED and GMP u8 Data Link echo validation application.
 */

#include <gmp_core.h>
#include <core/dev/datalink/datalink.h>

#include "user_main.h"

static gmp_datalink_t launchpad_datalink;
static time_gt last_led_tick;

static void launchpad_uart_rx_pump(void)
{
    byte_gt byte;
    size_gt bytes_read;

    while (gmp_hal_uart_get_rx_available(debug_uart) != 0U)
    {
        if (gmp_hal_uart_read(debug_uart, &byte, 1U, 1U, &bytes_read) == GMP_EC_OK)
        {
            gmp_dev_dl_push_byte(&launchpad_datalink, byte);
        }
    }
}

static void launchpad_datalink_tx(void)
{
    const byte_gt *buffer;
    size_gt length;

    buffer = gmp_dev_dl_get_tx_hw_hdr(&launchpad_datalink, &length);
    if ((length != 0U) &&
        (gmp_hal_uart_write(debug_uart, buffer, length, 20U) != GMP_EC_OK))
    {
        gmp_dev_dl_tx_state_done(&launchpad_datalink);
        return;
    }

    buffer = gmp_dev_dl_get_tx_hw_pld(&launchpad_datalink, &length);
    if (length != 0U)
    {
        (void)gmp_hal_uart_write(debug_uart, buffer, length, 20U);
    }
    gmp_dev_dl_tx_state_done(&launchpad_datalink);
}

void setup_peripheral(void)
{
    debug_uart = myUART0_BASE;
    gmp_c29x_set_tick_divider(1U);
}

void init(void)
{
    gmp_dev_dl_init(&launchpad_datalink);
    last_led_tick = gmp_base_get_system_tick();
}

void mainloop(void)
{
    gmp_dl_event_t event;

    launchpad_uart_rx_pump();
    event = gmp_dev_dl_loop_cb(&launchpad_datalink);
    if (event == GMP_DL_EVENT_TX_RDY)
    {
        launchpad_datalink_tx();
    }
    else if (event == GMP_DL_EVENT_RX_OK)
    {
        gmp_dev_dl_default_rx_handler(&launchpad_datalink);
    }

    DEVICE_DELAY_US(1000U);
    gmp_step_system_tick();
    if (gmp_base_is_delay_elapsed(last_led_tick, 500U))
    {
        GPIO_togglePin(myBoardLED0_GPIO);
        GPIO_togglePin(myBoardLED1_GPIO);
        last_led_tick = gmp_base_get_system_tick();
    }
}
