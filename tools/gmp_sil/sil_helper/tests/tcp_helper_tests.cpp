#include "asio_tcp_helper.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace transport = gmp::sil::tcp_transport;
using namespace std::chrono_literals;

namespace
{

void require(bool condition, const std::string& message)
{
    if (!condition)
        throw std::runtime_error(message);
}

template <typename Function>
void require_error(transport::error_code expected, Function&& function, const std::string& message)
{
    try
    {
        function();
    }
    catch (const transport::transport_error& exception)
    {
        require(exception.code() == expected, message + ": wrong transport error code");
        return;
    }
    throw std::runtime_error(message + ": expected transport_error");
}

transport::configuration server_config()
{
    transport::configuration config;
    config.endpoint_role = transport::role::server;
    config.bind_address = "127.0.0.1";
    config.port = 0U;
    config.connect_timeout = 2s;
    config.startup_io_timeout = 2s;
    config.established_io_timeout = 2s;
    config.max_payload = 4096U;
    return config;
}

transport::configuration client_config(std::uint16_t port)
{
    transport::configuration config;
    config.endpoint_role = transport::role::client;
    config.target_address = "127.0.0.1";
    config.port = port;
    config.connect_timeout = 2s;
    config.startup_io_timeout = 2s;
    config.established_io_timeout = 2s;
    config.max_payload = 4096U;
    return config;
}

std::vector<std::uint8_t> make_payload(std::size_t size, std::uint8_t seed)
{
    std::vector<std::uint8_t> payload(size);
    for (std::size_t index = 0; index < payload.size(); ++index)
        payload[index] = static_cast<std::uint8_t>((index * 37U + seed) & 0xFFU);
    return payload;
}

void test_header_codec()
{
    transport::frame_header header;
    header.kind = transport::frame_kind::command_response;
    header.flags = 0x01020304U;
    header.sequence = 0x0102030405060708ULL;
    header.payload_size = 0x11223344U;
    const auto encoded = transport::encode_header(header);

    require(encoded[0] == 'G' && encoded[1] == 'M' && encoded[2] == 'P' && encoded[3] == 'T',
            "wire magic is not stable");
    require(encoded[12] == 0x01U && encoded[19] == 0x08U, "sequence is not network byte order");
    require(encoded[20] == 0x11U && encoded[23] == 0x44U, "payload size is not network byte order");

    const auto decoded = transport::decode_header(encoded.data(), encoded.size());
    require(decoded.kind == header.kind && decoded.flags == header.flags && decoded.sequence == header.sequence &&
                decoded.payload_size == header.payload_size,
            "header codec did not round-trip");

    auto bad_magic = encoded;
    bad_magic[0] ^= 0xFFU;
    require_error(transport::error_code::protocol_error,
                  [&]() { (void)transport::decode_header(bad_magic.data(), bad_magic.size()); },
                  "invalid frame magic was accepted");
}

void test_request_response_stress()
{
    transport::asio_tcp_helper server(server_config());
    server.listen();
    const auto port = server.local_port();
    require(port != 0U, "server did not acquire a loopback port");

    std::exception_ptr server_failure;
    std::thread server_thread([&]() {
        try
        {
            server.accept();
            for (std::size_t iteration = 0; iteration < 512U; ++iteration)
            {
                auto request = server.receive();
                require(request.header.kind == transport::frame_kind::data_request, "server received wrong frame kind");
                require(request.payload.size() == 264U, "server received wrong SIL request size");
                std::reverse(request.payload.begin(), request.payload.end());

                transport::frame response;
                response.header.kind = transport::frame_kind::data_response;
                response.header.sequence = request.header.sequence;
                response.payload.assign(request.payload.begin(), request.payload.begin() + 200U);
                server.send(response);
            }

            auto heartbeat = server.receive();
            require(heartbeat.header.kind == transport::frame_kind::heartbeat, "server did not receive heartbeat");
            transport::frame response;
            response.header.kind = transport::frame_kind::heartbeat_response;
            response.header.sequence = heartbeat.header.sequence;
            server.send(response);
        }
        catch (...)
        {
            server_failure = std::current_exception();
        }
    });

    transport::asio_tcp_helper client(client_config(port));
    client.connect();
    for (std::size_t iteration = 0; iteration < 512U; ++iteration)
    {
        auto request = make_payload(264U, static_cast<std::uint8_t>(iteration));
        auto expected = request;
        std::reverse(expected.begin(), expected.end());
        expected.resize(200U);
        const auto response = client.transact(transport::frame_kind::data_request,
                                              transport::frame_kind::data_response, request);
        require(response.payload == expected, "binary request/response payload was corrupted");
    }
    const auto heartbeat = client.ping();
    require(heartbeat.payload.empty(), "heartbeat response should be empty");

    server_thread.join();
    if (server_failure)
        std::rethrow_exception(server_failure);
}

void test_fragmented_stream_receive()
{
    asio::io_context io;
    asio::ip::tcp::acceptor acceptor(io, {asio::ip::make_address("127.0.0.1"), 0U});
    const auto port = acceptor.local_endpoint().port();
    const auto payload = make_payload(307U, 0x5AU);

    transport::frame_header header;
    header.kind = transport::frame_kind::data_response;
    header.sequence = 77U;
    header.payload_size = static_cast<std::uint32_t>(payload.size());
    const auto encoded = transport::encode_header(header);

    std::thread raw_server([&]() {
        asio::ip::tcp::socket socket(io);
        acceptor.accept(socket);
        for (const auto byte : encoded)
        {
            asio::write(socket, asio::buffer(&byte, 1U));
            std::this_thread::sleep_for(100us);
        }
        for (std::size_t offset = 0; offset < payload.size(); offset += 7U)
        {
            const auto count = std::min<std::size_t>(7U, payload.size() - offset);
            asio::write(socket, asio::buffer(payload.data() + offset, count));
        }
    });

    transport::asio_tcp_helper client(client_config(port));
    client.connect();
    const auto received = client.receive();
    require(received.header.sequence == 77U && received.payload == payload,
            "exact-length reader failed on fragmented TCP stream");
    raw_server.join();
}

void test_receive_timeout_is_bounded()
{
    asio::io_context io;
    asio::ip::tcp::acceptor acceptor(io, {asio::ip::make_address("127.0.0.1"), 0U});
    const auto port = acceptor.local_endpoint().port();
    std::thread raw_server([&]() {
        asio::ip::tcp::socket socket(io);
        acceptor.accept(socket);
        std::this_thread::sleep_for(500ms);
    });

    auto config = client_config(port);
    config.startup_io_timeout = 100ms;
    transport::asio_tcp_helper client(config);
    client.connect();
    const auto started = std::chrono::steady_clock::now();
    require_error(transport::error_code::timeout, [&]() { (void)client.receive(); },
                  "silent peer did not produce a timeout");
    const auto elapsed = std::chrono::steady_clock::now() - started;
    require(elapsed >= 80ms && elapsed < 1s, "receive timeout was not bounded as configured");
    require(!client.connected(), "timed-out stream must be closed because framing is ambiguous");
    raw_server.join();
}

void test_established_link_uses_long_timeout()
{
    asio::io_context io;
    asio::ip::tcp::acceptor acceptor(io, {asio::ip::make_address("127.0.0.1"), 0U});
    const auto port = acceptor.local_endpoint().port();
    std::exception_ptr server_failure;
    std::thread raw_server([&]() {
        try
        {
            asio::ip::tcp::socket socket(io);
            acceptor.accept(socket);
            for (std::uint64_t sequence = 1U; sequence <= 2U; ++sequence)
            {
                if (sequence == 2U)
                    std::this_thread::sleep_for(250ms);
                transport::frame_header header;
                header.kind = transport::frame_kind::data_response;
                header.sequence = sequence;
                header.payload_size = 1U;
                const auto encoded = transport::encode_header(header);
                const std::uint8_t payload = static_cast<std::uint8_t>(sequence);
                asio::write(socket, asio::buffer(encoded));
                asio::write(socket, asio::buffer(&payload, 1U));
            }
        }
        catch (...)
        {
            server_failure = std::current_exception();
        }
    });

    auto config = client_config(port);
    config.startup_io_timeout = 100ms;
    config.established_io_timeout = 500ms;
    config.established_after_frames = 1U;
    transport::asio_tcp_helper client(config);
    client.connect();
    const auto first = client.receive();
    require(first.payload == std::vector<std::uint8_t>{1U}, "first complete frame was corrupted");
    require(client.established_timeout_active() && client.effective_io_timeout() == 500ms,
            "first complete frame did not enable established-link timeout");
    const auto second = client.receive();
    require(second.payload == std::vector<std::uint8_t>{2U},
            "established-link timeout did not tolerate a debugger-sized pause");
    require(client.completed_receive_frames() == 2U, "complete receive frame counter is incorrect");

    const auto abort_started = std::chrono::steady_clock::now();
    client.abort();
    require(std::chrono::steady_clock::now() - abort_started < 100ms && !client.connected(),
            "end-of-transfer abort did not close an idle connection immediately");

    raw_server.join();
    if (server_failure)
        std::rethrow_exception(server_failure);
}

void test_active_abort_interrupts_receive()
{
    asio::io_context io;
    asio::ip::tcp::acceptor acceptor(io, {asio::ip::make_address("127.0.0.1"), 0U});
    const auto port = acceptor.local_endpoint().port();
    std::thread raw_server([&]() {
        asio::ip::tcp::socket socket(io);
        acceptor.accept(socket);
        std::uint8_t byte = 0U;
        std::error_code ignored;
        socket.read_some(asio::buffer(&byte, 1U), ignored);
    });

    auto config = client_config(port);
    config.startup_io_timeout = 5s;
    config.established_io_timeout = 30s;
    transport::asio_tcp_helper client(config);
    client.connect();

    bool caught = false;
    transport::error_code observed = transport::error_code::io_error;
    const auto started = std::chrono::steady_clock::now();
    std::thread receiver([&]() {
        try
        {
            (void)client.receive();
        }
        catch (const transport::transport_error& exception)
        {
            caught = true;
            observed = exception.code();
        }
    });
    std::this_thread::sleep_for(100ms);
    client.abort();
    receiver.join();
    raw_server.join();
    const auto elapsed = std::chrono::steady_clock::now() - started;

    require(caught && observed == transport::error_code::aborted,
            "active abort did not report the dedicated aborted status");
    require(elapsed < 1s, "active abort waited for the configured receive timeout");
    require(!client.connected(), "actively aborted stream remained connected");
}

void test_partial_payload_disconnect()
{
    asio::io_context io;
    asio::ip::tcp::acceptor acceptor(io, {asio::ip::make_address("127.0.0.1"), 0U});
    const auto port = acceptor.local_endpoint().port();
    std::thread raw_server([&]() {
        asio::ip::tcp::socket socket(io);
        acceptor.accept(socket);
        transport::frame_header header;
        header.kind = transport::frame_kind::data_response;
        header.sequence = 1U;
        header.payload_size = 100U;
        const auto encoded = transport::encode_header(header);
        asio::write(socket, asio::buffer(encoded));
        const auto partial = make_payload(17U, 1U);
        asio::write(socket, asio::buffer(partial));
        std::error_code ignored;
        socket.shutdown(asio::ip::tcp::socket::shutdown_both, ignored);
        socket.close(ignored);
    });

    transport::asio_tcp_helper client(client_config(port));
    client.connect();
    require_error(transport::error_code::disconnected, [&]() { (void)client.receive(); },
                  "partial payload disconnect was not detected");
    raw_server.join();
}

void test_oversized_frame_is_rejected_before_allocation()
{
    asio::io_context io;
    asio::ip::tcp::acceptor acceptor(io, {asio::ip::make_address("127.0.0.1"), 0U});
    const auto port = acceptor.local_endpoint().port();
    std::thread raw_server([&]() {
        asio::ip::tcp::socket socket(io);
        acceptor.accept(socket);
        transport::frame_header header;
        header.kind = transport::frame_kind::data_response;
        header.sequence = 1U;
        header.payload_size = 4097U;
        const auto encoded = transport::encode_header(header);
        asio::write(socket, asio::buffer(encoded));
    });

    transport::asio_tcp_helper client(client_config(port));
    client.connect();
    require_error(transport::error_code::payload_too_large, [&]() { (void)client.receive(); },
                  "oversized payload was not rejected");
    raw_server.join();
}

void test_sequence_mismatch_closes_stream()
{
    transport::asio_tcp_helper server(server_config());
    server.listen();
    const auto port = server.local_port();
    std::exception_ptr server_failure;
    std::thread server_thread([&]() {
        try
        {
            server.accept();
            const auto request = server.receive();
            transport::frame response;
            response.header.kind = transport::frame_kind::data_response;
            response.header.sequence = request.header.sequence + 1U;
            server.send(response);
        }
        catch (...)
        {
            server_failure = std::current_exception();
        }
    });

    transport::asio_tcp_helper client(client_config(port));
    client.connect();
    transport::error_code observed = transport::error_code::io_error;
    bool caught = false;
    try
    {
        (void)client.transact(transport::frame_kind::data_request, transport::frame_kind::data_response,
                              make_payload(16U, 3U));
    }
    catch (const transport::transport_error& exception)
    {
        caught = true;
        observed = exception.code();
    }
    server_thread.join();
    if (server_failure)
        std::rethrow_exception(server_failure);
    require(caught && observed == transport::error_code::sequence_mismatch,
            "mismatched transaction sequence was not rejected");
    require(!client.connected(), "sequence mismatch must close the ambiguous stream");
}

void test_json_configuration()
{
    const auto path = std::filesystem::temp_directory_path() / "gmp_sil_helper_test.json";
    {
        std::ofstream stream(path);
        stream << R"({"transport":"tcp","role":"server","bind_address":"127.0.0.1","port":0,)"
                  R"("connect_timeout_ms":250,"startup_io_timeout_ms":125,)"
                  R"("established_io_timeout_ms":900000,"established_after_frames":3,"max_payload":8192,)"
                  R"("no_delay":true,"keep_alive":false})";
    }
    const auto config = transport::parse_configuration(path.string());
    std::filesystem::remove(path);
    require(config.endpoint_role == transport::role::server && config.port == 0U &&
                config.connect_timeout == 250ms && config.startup_io_timeout == 125ms &&
                config.established_io_timeout == 900000ms && config.established_after_frames == 3U &&
                config.max_payload == 8192U && config.no_delay && !config.keep_alive,
            "JSON configuration did not preserve values");
}

} // namespace

int main()
{
    const std::vector<std::pair<std::string, std::function<void()>>> tests = {
        {"header codec", test_header_codec},
        {"request/response stress", test_request_response_stress},
        {"fragmented stream receive", test_fragmented_stream_receive},
        {"bounded receive timeout", test_receive_timeout_is_bounded},
        {"established-link long timeout", test_established_link_uses_long_timeout},
        {"active abort", test_active_abort_interrupts_receive},
        {"partial payload disconnect", test_partial_payload_disconnect},
        {"oversized frame", test_oversized_frame_is_rejected_before_allocation},
        {"sequence mismatch", test_sequence_mismatch_closes_stream},
        {"JSON configuration", test_json_configuration},
    };

    std::size_t failures = 0U;
    for (const auto& [name, test] : tests)
    {
        try
        {
            test();
            std::cout << "[PASS] " << name << '\n';
        }
        catch (const std::exception& exception)
        {
            ++failures;
            std::cerr << "[FAIL] " << name << ": " << exception.what() << '\n';
        }
    }
    if (failures != 0U)
    {
        std::cerr << failures << " TCP helper test(s) failed.\n";
        return 1;
    }
    std::cout << "All TCP helper tests passed.\n";
    return 0;
}
