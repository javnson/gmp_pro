/**
 * @file example.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// This file provide a set of function that CSP must defined.

#include <gmp_core.h>
#include <tools/gmp_sil/udp_helper/asio_udp_helper.hpp>

#include <iostream>
#include <stdlib.h>

// ASIO helper object
asio_udp_helper *helper = nullptr;

// ASIO helper will send or receive message via this structure.
half_duplex_ift simulink_rx;
half_duplex_ift simulink_tx;

// buffer for rx & tx
extern "C"
{
    gmp_pc_simulink_rx_buffer_t simulink_rx_buffer;
    gmp_pc_simulink_tx_buffer_t simulink_tx_buffer;
}

// Simulink Enable signal
void csp_sl_enable_output(void)
{
    simulink_tx_buffer.enable = 1;
}

// Simulink Disable signal
void csp_sl_disable_output(void)
{
    simulink_tx_buffer.enable = 0;
}

// Simulink Panel Input
double csp_sl_get_panel_input(fast_gt channel)
{
    if (channel <= 3)
    {
        return simulink_rx_buffer.panel[channel];
    }
    return 0;
}

// User should invoke this function to get time (system tick).
time_gt gmp_port_system_tick(void)
{
    return (time_gt)(simulink_rx_buffer.time * GMP_BASE_TIME_TICK_RESOLUTION);
}

// This function may be called and used to initialize all the peripheral.
void gmp_csp_startup(void)
{
    // static uint32_t default_debug_dev_place_holder = 0;
    //  Specify a non-zero value to enable print.
    // default_debug_dev = &default_debug_dev_place_holder;

    // Setup ASIO helper
    helper = asio_udp_helper::parse_network_config(GMP_ASIO_CONFIG_JSON);

    if (helper == nullptr)
    {
        std::cout << "Cannot create ASIO Helper.\r\n" << std::endl;
        exit(1);
    }

    // Connect to Simulink Model
    helper->connect_to_target();

#ifdef GMP_ASIO_ENABLE_STOP_CMD
    // enable this program acknowledge "Stop" Command from Simulink
    // BUG REPORT this function may cause exception
    // helper->server_ack_cmd();
#endif // GMP_ASIO_ENABLE_STOP_CMD

    gmp_base_print("[INFO] Simulink RX buffer size: %llu\r\n", sizeof(simulink_rx_buffer));
    gmp_base_print("[INFO] Simulink TX buffer size: %llu\r\n", sizeof(simulink_tx_buffer));

    // Config send & recv buffer
    gmp_dev_init_half_duplex_channel(&simulink_rx, (data_gt *)&simulink_rx_buffer, sizeof(simulink_rx_buffer),
                                     sizeof(simulink_rx_buffer));
    // simulink_rx.buf = (data_gt *)&simulink_rx_buffer;
    // simulink_rx.length = sizeof(simulink_rx_buffer);
    // simulink_rx.capacity = sizeof(simulink_rx_buffer);

    gmp_dev_init_half_duplex_channel(&simulink_tx, (data_gt *)&simulink_tx_buffer, sizeof(simulink_tx_buffer),
                                     sizeof(simulink_tx_buffer));
    // simulink_tx.buf = (data_gt *)&simulink_tx_buffer;
    // simulink_tx.length = sizeof(simulink_tx_buffer);
    // simulink_tx.capacity = sizeof(simulink_tx_buffer);
}

// This function may be called and used to initialize all the peripheral.
void gmp_csp_post_process(void)
{
    // Send the first message to enable the Simulink model.
    helper->send_msg((char *)simulink_tx.buf, simulink_tx.length);
}

// This function is unreachable.
void gmp_csp_exit(void)
{
    delete helper;

    printf("[GMP EXIT FUNCTION] GMP will leave.\r\n");
}

// This function may invoke when main loop occurred.
void gmp_csp_loop(void)
{

    //////////////////////////////////////////////////////////////////////////
    // Here is controller loop simulate routine
    //
    static size_gt controller_loop_tick = 0;

    if (++controller_loop_tick >= GMP_PC_CONTROLLER_DIV_PER_MAINLOOP)
    {
        // Clear division
        controller_loop_tick = 0;

        // parameters validate
        if (simulink_tx.buf == nullptr || simulink_rx.buf == nullptr)
        {
            std::cout << "You should initialize Simulink TX and Simulink RX object.\r\n";

            helper->release_connect();

            delete helper;
            helper = nullptr;
        }

        if (helper == nullptr)
        {
            std::cout << "ASIO Helper Objects has been deleted.\r\n";
            return;
        }

        // Receive message from Simulink
        if (helper->recv_msg((char *)simulink_rx.buf, simulink_rx.length))
        {
            std::cout << "receive complete." << std::endl;

            std::cout << "received " << helper->recv_counter << " Bytes, aka " << (double)helper->recv_counter / 1024
                      << "kBytes" << std::endl;
            std::cout << "transmitted " << helper->tran_counter << " Bytes, aka " << (double)helper->tran_counter / 1024
                      << "kBytes" << std::endl;

            delete helper;

            system("@pause");
            exit(0);
        }

        // Controller operation here
        gmp_base_ctl_step();

        // Send message to Simulink
        helper->send_msg((char *)simulink_tx.buf, simulink_tx.length);
    }
}

// This function would be called when fatal error occurred.
void gmp_port_system_stuck(void)
{

    printf("[FATAL] GMP system stuck is invoked.\r\n");

    for (;;)
    {
    }
}

// Windows print function
ec_gt windows_print_function(uint32_t *handle, half_duplex_ift *port)
{
    // allow handle not be referenced.
    UNUSED_PARAMETER(handle);

    for (size_gt i = 0; i < port->length; ++i)
        putchar(port->buf[i]);

    return GMP_EC_OK;
}

// Windows Simulink system tick function
time_gt gmp_base_get_system_tick()
{
    return (time_gt)(simulink_rx_buffer.time * SPECIFY_SYSTEM_TICK_FREQUENCY);
}
