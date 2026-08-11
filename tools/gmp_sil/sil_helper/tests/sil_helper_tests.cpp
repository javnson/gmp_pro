#include <tools/gmp_sil/sil_helper/gmp_sil_helper.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
namespace sil = gmp::sil;

namespace
{

void require(bool condition, const std::string& message)
{
    if (!condition)
        throw std::runtime_error(message);
}

std::uint16_t free_port(bool tcp)
{
    asio::io_context io;
    if (tcp)
    {
        asio::ip::tcp::acceptor acceptor(io, {asio::ip::make_address("127.0.0.1"), 0U});
        return acceptor.local_endpoint().port();
    }
    asio::ip::udp::socket socket(io, {asio::ip::make_address("127.0.0.1"), 0U});
    return socket.local_endpoint().port();
}

sil::configuration make_config(sil::transport_kind transport, sil::endpoint_role role, std::uint16_t server_port,
                               std::uint16_t client_port)
{
    sil::configuration config;
    config.transport = transport;
    config.role = role;
    config.target_address = "127.0.0.1";
    config.bind_address = "127.0.0.1";
    config.transmit_port = role == sil::endpoint_role::client ? server_port : client_port;
    config.receive_port = role == sil::endpoint_role::client ? client_port : server_port;
    config.connect_timeout = 2s;
    config.startup_io_timeout = 2s;
    config.established_io_timeout = 2s;
    config.established_after_frames = 1U;
    config.max_payload = 4096U;
    config.session.id = sil::protocol::parse_session_id("00112233445566778899aabbccddeeff");
    config.session.request_payload_size = 32U;
    config.session.response_payload_size = 24U;
    return config;
}

void run_transport_session(sil::transport_kind transport, std::uint16_t server_port, std::uint16_t client_port,
                           std::uint8_t session_tag)
{
    auto server_config = make_config(transport, sil::endpoint_role::server, server_port, client_port);
    auto client_config = make_config(transport, sil::endpoint_role::client, server_port, client_port);
    server_config.session.id[0] = session_tag;
    client_config.session.id[0] = session_tag;
    sil::gmp_sil_helper server(server_config);
    std::exception_ptr server_failure;
    std::thread server_thread([&]() {
        try
        {
            server.connect();
            auto event = server.receive_event();
            const auto running = server.decode_state(event);
            require(running.state == sil::protocol::simulation_state::running && running.major_step == 0U,
                    "server did not receive running state");
            server.acknowledge_state(event);

            event = server.receive_event();
            require(event.kind == sil::protocol::frame_kind::data_request && event.payload.size() == 32U,
                    "server did not receive the ABI-sized request");
            std::vector<std::uint8_t> response(event.payload.rbegin(), event.payload.rbegin() + 24U);
            server.respond(event, response.data(), response.size());

            event = server.receive_event();
            const auto completed = server.decode_state(event);
            require(completed.state == sil::protocol::simulation_state::completed && completed.major_step == 1U,
                    "server did not receive explicit completion state");
            server.acknowledge_state(event);
        }
        catch (...)
        {
            server_failure = std::current_exception();
        }
    });

    // Production starts the controller before Simulink.  Give the server
    // thread time to bind its UDP socket (or begin TCP accept) before the
    // client emits the one-shot session_hello datagram.
    std::this_thread::sleep_for(25ms);

    std::exception_ptr client_failure;
    try
    {
        sil::gmp_sil_helper client(client_config);
        client.connect();
        client.notify_state(sil::protocol::simulation_state::running, 0U);
        std::vector<std::uint8_t> request(32U);
        for (std::size_t index = 0U; index < request.size(); ++index)
            request[index] = static_cast<std::uint8_t>(index ^ session_tag);
        const auto response = client.exchange(request.data(), request.size());
        std::vector<std::uint8_t> expected(request.rbegin(), request.rbegin() + 24U);
        require(response == expected, "facade corrupted the SIL request/response payload");
        client.notify_state(sil::protocol::simulation_state::completed, 1U);
        client.close();
    }
    catch (...)
    {
        client_failure = std::current_exception();
        server.abort();
    }

    server_thread.join();
    if (client_failure)
        std::rethrow_exception(client_failure);
    if (server_failure)
        std::rethrow_exception(server_failure);
}

void run_transport_test(sil::transport_kind transport)
{
    const bool tcp = transport == sil::transport_kind::tcp;
    const auto server_port = free_port(tcp);
    auto client_port = free_port(false);
    while (client_port == server_port)
        client_port = free_port(false);
    run_transport_session(transport, server_port, client_port, 0x31U);
}

void test_two_independent_sessions(sil::transport_kind transport)
{
    const bool tcp = transport == sil::transport_kind::tcp;
    std::vector<std::uint16_t> ports;
    while (ports.size() < 4U)
    {
        const auto candidate = free_port(ports.size() % 2U == 0U ? tcp : false);
        if (std::find(ports.begin(), ports.end(), candidate) == ports.end())
            ports.push_back(candidate);
    }

    std::exception_ptr first_failure;
    std::exception_ptr second_failure;
    std::thread first([&]() {
        try
        {
            run_transport_session(transport, ports[0], ports[1], 0x51U);
        }
        catch (...)
        {
            first_failure = std::current_exception();
        }
    });
    std::thread second([&]() {
        try
        {
            run_transport_session(transport, ports[2], ports[3], 0xA7U);
        }
        catch (...)
        {
            second_failure = std::current_exception();
        }
    });
    first.join();
    second.join();
    if (first_failure)
        std::rethrow_exception(first_failure);
    if (second_failure)
        std::rethrow_exception(second_failure);
}

void test_abi_rejected_before_network()
{
    auto config = make_config(sil::transport_kind::udp, sil::endpoint_role::client, 12510U, 12511U);
    config.max_payload = 16U;
    bool caught = false;
    try
    {
        sil::gmp_sil_helper helper(config);
    }
    catch (const sil::sil_error&)
    {
        caught = true;
    }
    require(caught, "JSON ABI larger than max_payload was accepted");
}

void test_server_waits_without_timeout(sil::transport_kind transport)
{
    const bool tcp = transport == sil::transport_kind::tcp;
    const auto server_port = free_port(tcp);
    auto client_port = free_port(false);
    while (client_port == server_port)
        client_port = free_port(false);

    auto server_config = make_config(transport, sil::endpoint_role::server, server_port, client_port);
    auto client_config = make_config(transport, sil::endpoint_role::client, server_port, client_port);
    server_config.startup_io_timeout = 25ms;
    client_config.startup_io_timeout = 500ms;
    server_config.connect_timeout = 25ms;
    client_config.connect_timeout = 500ms;
    server_config.startup_timeout_enabled = false;
    client_config.startup_timeout_enabled = true;

    sil::gmp_sil_helper server(server_config);
    std::exception_ptr server_failure;
    std::thread server_thread([&]() {
        try
        {
            server.connect();
            server.close();
        }
        catch (...)
        {
            server_failure = std::current_exception();
        }
    });

    // Deliberately exceed the server's configured short timeout. The
    // controller/server must continue waiting, while the Simulink/client side
    // keeps its own bounded startup policy.
    std::this_thread::sleep_for(150ms);
    std::exception_ptr client_failure;
    try
    {
        sil::gmp_sil_helper client(client_config);
        client.connect();
        client.close();
    }
    catch (...)
    {
        client_failure = std::current_exception();
        server.abort();
    }
    server_thread.join();
    if (client_failure)
        std::rethrow_exception(client_failure);
    if (server_failure)
        std::rethrow_exception(server_failure);
}

void test_udp_client_times_out_without_controller()
{
    const auto server_port = free_port(false);
    auto client_port = free_port(false);
    while (client_port == server_port)
        client_port = free_port(false);

    auto client_config = make_config(sil::transport_kind::udp, sil::endpoint_role::client,
                                     server_port, client_port);
    client_config.startup_io_timeout = 100ms;
    client_config.connect_timeout = 100ms;
    client_config.startup_timeout_enabled = true;

    const auto started = std::chrono::steady_clock::now();
    bool caught_timeout = false;
    try
    {
        sil::gmp_sil_helper client(client_config);
        client.connect();
    }
    catch (const std::exception& exception)
    {
        caught_timeout = std::string(exception.what()).find("timed out") != std::string::npos;
    }
    const auto elapsed = std::chrono::steady_clock::now() - started;
    require(caught_timeout, "UDP Simulink client did not report a missing-controller timeout");
    require(elapsed >= 75ms && elapsed < 2s, "UDP startup timeout was not bounded as configured");
}

} // namespace

int main()
{
    try
    {
        std::cout.setf(std::ios::unitbuf);
        run_transport_test(sil::transport_kind::udp);
        std::cout << "[PASS] unified UDP session\n";
        run_transport_test(sil::transport_kind::tcp);
        std::cout << "[PASS] unified TCP session\n";
        test_two_independent_sessions(sil::transport_kind::udp);
        test_two_independent_sessions(sil::transport_kind::tcp);
        std::cout << "[PASS] two isolated concurrent sessions\n";
        test_server_waits_without_timeout(sil::transport_kind::udp);
        test_server_waits_without_timeout(sil::transport_kind::tcp);
        test_udp_client_times_out_without_controller();
        std::cout << "[PASS] asymmetric controller-wait/client-timeout startup policy\n";
        test_abi_rejected_before_network();
        std::cout << "[PASS] ABI validation\n";
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "[FAIL] " << exception.what() << '\n';
        return 1;
    }
}
