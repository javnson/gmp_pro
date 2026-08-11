/**
 * @file  windows_simulink_main.c
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

// Unified TCP/UDP SIL transport.
#include <tools/gmp_sil/sil_helper/gmp_sil_helper.hpp>

// Trace RT module
#include <ctrl_rt_trace.h>

#include <iostream>
#include <stdlib.h>

// SIL helper object
gmp::sil::gmp_sil_helper* helper = nullptr;

// ASIO helper will send or receive message via this structure.
//half_duplex_ift simulink_rx;
//half_duplex_ift simulink_tx;

// trace rt module context
trace_rt_context_t trace_rt_context;

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

    try
    {
        auto instance = gmp::sil::gmp_sil_helper::from_json(GMP_ASIO_CONFIG_JSON);
        const auto& session = instance->config().session;
        if (session.request_payload_size != sizeof(simulink_rx_buffer) ||
            session.response_payload_size != sizeof(simulink_tx_buffer))
            throw gmp::sil::sil_error("Controller buffer sizes do not match the JSON SIL ABI contract.");
        instance->connect();
        helper = instance.release();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "Cannot establish GMP SIL session: " << exception.what() << std::endl;
        exit(1);
    }

    gmp_base_print("[INFO] Simulink RX buffer size: %llu\r\n", sizeof(simulink_rx_buffer));
    gmp_base_print("[INFO] Simulink TX buffer size: %llu\r\n", sizeof(simulink_tx_buffer));

    // Config send & recv buffer
    //gmp_dev_init_half_duplex_channel(&simulink_rx, (data_gt*)&simulink_rx_buffer, sizeof(simulink_rx_buffer),
    //                                 sizeof(simulink_rx_buffer));
    // simulink_rx.buf = (data_gt *)&simulink_rx_buffer;
    // simulink_rx.length = sizeof(simulink_rx_buffer);
    // simulink_rx.capacity = sizeof(simulink_rx_buffer);

    //gmp_dev_init_half_duplex_channel(&simulink_tx, (data_gt*)&simulink_tx_buffer, sizeof(simulink_tx_buffer),
    //                                 sizeof(simulink_tx_buffer));
    // simulink_tx.buf = (data_gt *)&simulink_tx_buffer;
    // simulink_tx.length = sizeof(simulink_tx_buffer);
    // simulink_tx.capacity = sizeof(simulink_tx_buffer);

    // init trace rt objects
#ifdef CTRL_FS
    trace_rt_entity_init(&trace_rt_context, 1000.0 / CTRL_FS);
#else
    trace_rt_entity_init(&trace_rt_context, 1.0);
#endif // CTRL_FS
}

// This function may be called and used to initialize all the peripheral.
void gmp_csp_post_process(void)
{
    // create & save tracert file
    gmp_trace_rt_generate_layout(&trace_rt_context);
}

// This function is unreachable.
void gmp_csp_exit(void)
{
    if (helper != nullptr)
    {
        helper->close();
        delete helper;
        helper = nullptr;
    }

    gmp_trace_rt_release(&trace_rt_context);

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
        //if (simulink_tx.buf == nullptr || simulink_rx.buf == nullptr)
        //{
        //    std::cout << "You should initialize Simulink TX and Simulink RX object.\r\n";

        //    helper->release_connect();

        //    delete helper;
        //    helper = nullptr;
        //}

        if (helper == nullptr)
        {
            std::cout << "ASIO Helper Objects has been deleted.\r\n";
            return;
        }

        try
        {
            const auto event = helper->receive_event();
            if (event.kind == gmp::sil::protocol::frame_kind::data_request)
            {
                memcpy(&simulink_rx_buffer, event.payload.data(), sizeof(simulink_rx_buffer));
                gmp_base_ctl_step();
                helper->respond(event, &simulink_tx_buffer, sizeof(simulink_tx_buffer));
            }
            else if (event.kind == gmp::sil::protocol::frame_kind::simulation_state)
            {
                const auto status = helper->decode_state(event);
                helper->acknowledge_state(event);
                if (status.state == gmp::sil::protocol::simulation_state::completed ||
                    status.state == gmp::sil::protocol::simulation_state::aborted ||
                    status.state == gmp::sil::protocol::simulation_state::faulted)
                {
                    gmp_base_print("[INFO] Simulink session ended explicitly at major step %llu.\r\n",
                                   status.major_step);
                    gmp_csp_exit();
                    exit(0);
                }
            }
        }
        catch (const std::exception& exception)
        {
            std::cerr << "GMP SIL communication failed: " << exception.what() << std::endl;
            gmp_csp_exit();
            exit(1);
        }
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
//ec_gt windows_print_function(uint32_t* handle, half_duplex_ift* port)
//{
//    // allow handle not be referenced.
//    UNUSED_PARAMETER(handle);
//
//    for (size_gt i = 0; i < port->length; ++i)
//        putchar(port->buf[i]);
//
//    return GMP_EC_OK;
//}

// Windows Simulink system tick function
time_gt gmp_base_get_system_tick()
{
    return (time_gt)(simulink_rx_buffer.time * SPECIFY_SYSTEM_TICK_FREQUENCY);
}
