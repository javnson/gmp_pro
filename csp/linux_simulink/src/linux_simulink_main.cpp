/**
 * @file linux_simulink_main.cpp
 * @brief GMP runtime port for native Linux SIL/PIL controller processes.
 */

#include <gmp_core.h>
#include <ctrl_rt_trace.h>
#include <tools/gmp_sil/sil_helper/gmp_sil_helper.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>

namespace
{
std::unique_ptr<gmp::sil::gmp_sil_helper> sil_helper;
}

trace_rt_context_t trace_rt_context;

extern "C"
{
gmp_pc_simulink_rx_buffer_t simulink_rx_buffer;
gmp_pc_simulink_tx_buffer_t simulink_tx_buffer;
}

/** @brief Mark the simulated power-stage output as enabled. */
void csp_sl_enable_output(void)
{
    simulink_tx_buffer.enable = 1;
}

/** @brief Mark the simulated power-stage output as disabled. */
void csp_sl_disable_output(void)
{
    simulink_tx_buffer.enable = 0;
}

/**
 * @brief Read a scalar panel input received from the simulation endpoint.
 * @param channel Zero-based panel channel in the range 0 through 3.
 * @return Channel value, or zero when the channel is outside the supported range.
 */
double csp_sl_get_panel_input(fast_gt channel)
{
    return channel <= 3 ? simulink_rx_buffer.panel[channel] : 0.0;
}

/** @brief Return elapsed simulation time expressed in base time ticks. */
time_gt gmp_port_system_tick(void)
{
    return static_cast<time_gt>(simulink_rx_buffer.time * GMP_BASE_TIME_TICK_RESOLUTION);
}

/** @brief Return the default coarse system clock used by GMP schedulers. */
time_gt gmp_base_get_system_tick(void)
{
    return static_cast<time_gt>(simulink_rx_buffer.time * SPECIFY_SYSTEM_TICK_FREQUENCY);
}

/** @brief Establish the framed SIL/PIL session and initialize runtime tracing. */
void gmp_csp_startup(void)
{
    try
    {
        auto instance = gmp::sil::gmp_sil_helper::from_json(GMP_ASIO_CONFIG_JSON);
        const auto& session = instance->config().session;
        if (session.request_payload_size != sizeof(simulink_rx_buffer) ||
            session.response_payload_size != sizeof(simulink_tx_buffer))
        {
            throw gmp::sil::sil_error("Controller buffer sizes do not match the JSON SIL ABI contract.");
        }

        const auto& network = instance->config();
        std::cout << "[INFO] GMP Linux SIL controller is waiting for Simulink over "
                  << (network.transport == gmp::sil::transport_kind::tcp ? "TCP" : "UDP") << " on port "
                  << network.receive_port << '.' << std::endl;
        instance->connect();
        std::cout << "[INFO] GMP SIL session established; controller loop is starting." << std::endl;
        sil_helper = std::move(instance);
    }
    catch (const std::exception& error)
    {
        std::cerr << "[ERROR] Cannot establish the GMP SIL session: " << error.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    gmp_base_print("[INFO] Simulink RX buffer size: %zu\r\n", sizeof(simulink_rx_buffer));
    gmp_base_print("[INFO] Simulink TX buffer size: %zu\r\n", sizeof(simulink_tx_buffer));

#ifdef CTRL_FS
    trace_rt_entity_init(&trace_rt_context, 1000.0 / CTRL_FS);
#else
    trace_rt_entity_init(&trace_rt_context, 1.0);
#endif
}

/** @brief Emit trace metadata after application initialization completes. */
void gmp_csp_post_process(void)
{
    gmp_trace_rt_generate_layout(&trace_rt_context);
}

/** @brief Close the SIL/PIL transport and release runtime trace resources. */
void gmp_csp_exit(void)
{
    if (sil_helper != nullptr)
        sil_helper->close();
    sil_helper.reset();
    gmp_trace_rt_release(&trace_rt_context);
    std::puts("[INFO] GMP Linux SIL controller exited.");
}

/** @brief Process one background SIL/PIL transport iteration. */
void gmp_csp_loop(void)
{
    static size_gt controller_loop_tick = 0;
    if (++controller_loop_tick < GMP_PC_CONTROLLER_DIV_PER_MAINLOOP)
        return;
    controller_loop_tick = 0;

    if (sil_helper == nullptr)
    {
        std::cerr << "[ERROR] The GMP SIL connection object is unavailable." << std::endl;
        return;
    }

    try
    {
        const auto event = sil_helper->receive_event();
        if (event.kind == gmp::sil::protocol::frame_kind::data_request)
        {
            std::memcpy(&simulink_rx_buffer, event.payload.data(), sizeof(simulink_rx_buffer));
#if defined(SPECIFY_ENABLE_GMP_CTL) && !defined(SPECIFY_DISABLE_GMP_CTL)
            gmp_base_ctl_step();
#endif
            sil_helper->respond(event, &simulink_tx_buffer, sizeof(simulink_tx_buffer));
        }
        else if (event.kind == gmp::sil::protocol::frame_kind::simulation_state)
        {
            const auto status = sil_helper->decode_state(event);
            sil_helper->acknowledge_state(event);
            if (status.state == gmp::sil::protocol::simulation_state::completed ||
                status.state == gmp::sil::protocol::simulation_state::aborted ||
                status.state == gmp::sil::protocol::simulation_state::faulted)
            {
                gmp_base_print("[INFO] Simulink session ended explicitly at major step %llu.\r\n",
                               static_cast<unsigned long long>(status.major_step));
                gmp_csp_exit();
                std::exit(EXIT_SUCCESS);
            }
        }
    }
    catch (const std::exception& error)
    {
        std::cerr << "[ERROR] GMP SIL communication failed: " << error.what() << std::endl;
        gmp_csp_exit();
        std::exit(EXIT_FAILURE);
    }
}

/** @brief Abort the hosted process after an unrecoverable GMP fault. */
void gmp_csp_stuck_routine(void)
{
    std::cerr << "[FATAL] GMP entered the Linux CSP stuck routine." << std::endl;
    std::abort();
}

/** @brief Report a call to a CSP service that has no hosted implementation. */
void gmp_csp_not_implement(void)
{
    std::cerr << "[ERROR] GMP invoked an unimplemented Linux CSP service." << std::endl;
}
