/**
 * @file asio_tcp_helper.hpp
 * @brief Framed, timeout-bounded TCP transport for GMP SIL/PIL experiments.
 *
 * TCP is a byte stream, not a message transport.  This helper therefore adds
 * an explicit, endian-stable frame header and exact-length reads.  One duplex
 * connection carries simulation data, commands and heartbeat frames.  It does
 * not retry a transaction automatically: after a connection failure the
 * caller cannot know whether the remote controller executed the last request.
 */

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#if defined(_WIN32)
#include <SDKDDKVer.h>
#endif

#ifndef ASIO_STANDALONE
#define ASIO_STANDALONE
#endif
#include <asio.hpp>
#include <nlohmann/json.hpp>

namespace gmp::sil::tcp_transport
{

constexpr std::uint32_t frame_magic = 0x474D5054U; // ASCII "GMPT".
constexpr std::uint16_t protocol_version = 1U;
constexpr std::size_t wire_header_size = 24U;
constexpr std::uint32_t default_max_payload = 16U * 1024U * 1024U;

enum class role
{
    client,
    server
};

enum class frame_kind : std::uint16_t
{
    data_request = 1U,
    data_response = 2U,
    command = 3U,
    command_response = 4U,
    heartbeat = 5U,
    heartbeat_response = 6U,
    error = 7U
};

enum class error_code
{
    timeout,
    disconnected,
    protocol_error,
    payload_too_large,
    configuration_error,
    sequence_mismatch,
    unexpected_frame,
    io_error
};

class transport_error : public std::runtime_error
{
  public:
    transport_error(error_code code, std::string message) : std::runtime_error(std::move(message)), code_(code) {}

    [[nodiscard]] error_code code() const noexcept
    {
        return code_;
    }

  private:
    error_code code_;
};

struct frame_header
{
    std::uint32_t magic = frame_magic;
    std::uint16_t version = protocol_version;
    frame_kind kind = frame_kind::data_request;
    std::uint32_t flags = 0U;
    std::uint64_t sequence = 0U;
    std::uint32_t payload_size = 0U;
};

struct frame
{
    frame_header header;
    std::vector<std::uint8_t> payload;
};

struct configuration
{
    role endpoint_role = role::client;
    std::string target_address = "127.0.0.1";
    std::string bind_address = "0.0.0.0";
    std::uint16_t port = 12510U;
    std::chrono::milliseconds connect_timeout{5000};
    std::chrono::milliseconds io_timeout{5000};
    std::uint32_t max_payload = default_max_payload;
    bool no_delay = true;
    bool keep_alive = true;
};

namespace detail
{

inline void put_u16(std::uint8_t* output, std::uint16_t value)
{
    output[0] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    output[1] = static_cast<std::uint8_t>(value & 0xFFU);
}

inline void put_u32(std::uint8_t* output, std::uint32_t value)
{
    output[0] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
    output[1] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
    output[2] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    output[3] = static_cast<std::uint8_t>(value & 0xFFU);
}

inline void put_u64(std::uint8_t* output, std::uint64_t value)
{
    for (std::size_t index = 0; index < 8U; ++index)
        output[index] = static_cast<std::uint8_t>((value >> ((7U - index) * 8U)) & 0xFFU);
}

inline std::uint16_t get_u16(const std::uint8_t* input)
{
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(input[0]) << 8U) |
                                      static_cast<std::uint16_t>(input[1]));
}

inline std::uint32_t get_u32(const std::uint8_t* input)
{
    return (static_cast<std::uint32_t>(input[0]) << 24U) | (static_cast<std::uint32_t>(input[1]) << 16U) |
           (static_cast<std::uint32_t>(input[2]) << 8U) | static_cast<std::uint32_t>(input[3]);
}

inline std::uint64_t get_u64(const std::uint8_t* input)
{
    std::uint64_t value = 0U;
    for (std::size_t index = 0; index < 8U; ++index)
        value = (value << 8U) | static_cast<std::uint64_t>(input[index]);
    return value;
}

inline bool is_known_kind(frame_kind kind)
{
    const auto value = static_cast<std::uint16_t>(kind);
    return value >= static_cast<std::uint16_t>(frame_kind::data_request) &&
           value <= static_cast<std::uint16_t>(frame_kind::error);
}

} // namespace detail

inline std::array<std::uint8_t, wire_header_size> encode_header(const frame_header& header)
{
    std::array<std::uint8_t, wire_header_size> output{};
    detail::put_u32(output.data(), header.magic);
    detail::put_u16(output.data() + 4U, header.version);
    detail::put_u16(output.data() + 6U, static_cast<std::uint16_t>(header.kind));
    detail::put_u32(output.data() + 8U, header.flags);
    detail::put_u64(output.data() + 12U, header.sequence);
    detail::put_u32(output.data() + 20U, header.payload_size);
    return output;
}

inline frame_header decode_header(const std::uint8_t* data, std::size_t size)
{
    if (data == nullptr || size != wire_header_size)
        throw transport_error(error_code::protocol_error, "TCP frame header has an invalid size.");

    frame_header header;
    header.magic = detail::get_u32(data);
    header.version = detail::get_u16(data + 4U);
    header.kind = static_cast<frame_kind>(detail::get_u16(data + 6U));
    header.flags = detail::get_u32(data + 8U);
    header.sequence = detail::get_u64(data + 12U);
    header.payload_size = detail::get_u32(data + 20U);

    if (header.magic != frame_magic)
        throw transport_error(error_code::protocol_error, "TCP frame magic does not match GMP SIL.");
    if (header.version != protocol_version)
        throw transport_error(error_code::protocol_error, "TCP frame protocol version is unsupported.");
    if (!detail::is_known_kind(header.kind))
        throw transport_error(error_code::protocol_error, "TCP frame kind is invalid.");
    return header;
}

inline configuration parse_configuration(const std::string& config_file)
{
    std::ifstream stream(config_file);
    if (!stream.is_open())
        throw transport_error(error_code::configuration_error, "Cannot open TCP configuration: " + config_file);

    nlohmann::json document;
    try
    {
        stream >> document;
    }
    catch (const std::exception& exception)
    {
        throw transport_error(error_code::configuration_error,
                              "Cannot parse TCP configuration: " + std::string(exception.what()));
    }

    configuration result;
    try
    {
        if (document.contains("transport") && document.at("transport").get<std::string>() != "tcp")
            throw transport_error(error_code::configuration_error, "Configuration transport must be 'tcp'.");

        const std::string configured_role = document.value("role", "client");
        if (configured_role == "client")
            result.endpoint_role = role::client;
        else if (configured_role == "server")
            result.endpoint_role = role::server;
        else
            throw transport_error(error_code::configuration_error, "TCP role must be 'client' or 'server'.");

        result.target_address = document.value("target_address", result.target_address);
        result.bind_address = document.value("bind_address", result.bind_address);
        const auto configured_port = document.value("port", static_cast<unsigned>(result.port));
        const auto connect_timeout_ms = document.value("connect_timeout_ms", result.connect_timeout.count());
        const auto io_timeout_ms = document.value("io_timeout_ms", result.io_timeout.count());
        const auto configured_max_payload = document.value("max_payload", result.max_payload);

        if (configured_port > 65535U || (configured_port == 0U && result.endpoint_role == role::client))
            throw transport_error(error_code::configuration_error, "TCP port is outside the valid range.");
        if (connect_timeout_ms <= 0 || io_timeout_ms <= 0 || configured_max_payload == 0U)
            throw transport_error(error_code::configuration_error, "TCP timeouts and max_payload must be positive.");

        result.port = static_cast<std::uint16_t>(configured_port);
        result.connect_timeout = std::chrono::milliseconds(connect_timeout_ms);
        result.io_timeout = std::chrono::milliseconds(io_timeout_ms);
        result.max_payload = configured_max_payload;
        result.no_delay = document.value("no_delay", result.no_delay);
        result.keep_alive = document.value("keep_alive", result.keep_alive);
    }
    catch (const transport_error&)
    {
        throw;
    }
    catch (const std::exception& exception)
    {
        throw transport_error(error_code::configuration_error,
                              "Invalid TCP configuration value: " + std::string(exception.what()));
    }
    return result;
}

class asio_tcp_helper
{
  public:
    explicit asio_tcp_helper(configuration config)
        : config_(std::move(config)), socket_(io_), next_sequence_(1U)
    {
        if (config_.connect_timeout.count() <= 0 || config_.io_timeout.count() <= 0 || config_.max_payload == 0U)
            throw transport_error(error_code::configuration_error,
                                  "TCP timeouts and max_payload must be positive.");
    }

    asio_tcp_helper(const asio_tcp_helper&) = delete;
    asio_tcp_helper& operator=(const asio_tcp_helper&) = delete;

    ~asio_tcp_helper()
    {
        close();
    }

    void listen()
    {
        std::scoped_lock lock(operation_mutex_);
        if (config_.endpoint_role != role::server)
            throw transport_error(error_code::configuration_error, "listen() requires server role.");
        if (acceptor_ && acceptor_->is_open())
            return;

        std::error_code ec;
        const auto address = asio::ip::make_address(config_.bind_address, ec);
        if (ec || !address.is_v4())
            throw transport_error(error_code::configuration_error, "bind_address must be a valid IPv4 address.");

        acceptor_ = std::make_unique<asio::ip::tcp::acceptor>(io_);
        acceptor_->open(asio::ip::tcp::v4(), ec);
        if (!ec)
            acceptor_->set_option(asio::socket_base::reuse_address(true), ec);
        if (!ec)
            acceptor_->bind(asio::ip::tcp::endpoint(address, config_.port), ec);
        if (!ec)
            acceptor_->listen(asio::socket_base::max_listen_connections, ec);
        if (ec)
        {
            close_acceptor_noexcept();
            throw transport_error(error_code::io_error, "Cannot listen on TCP socket: " + ec.message());
        }
    }

    void accept()
    {
        std::scoped_lock lock(operation_mutex_);
        if (config_.endpoint_role != role::server)
            throw transport_error(error_code::configuration_error, "accept() requires server role.");
        if (!acceptor_ || !acceptor_->is_open())
            throw transport_error(error_code::configuration_error, "listen() must be called before accept().");

        close_socket_noexcept();
        socket_ = asio::ip::tcp::socket(io_);
        run_timed_control(
            [this](auto handler) { acceptor_->async_accept(socket_, std::move(handler)); },
            [this]() {
                std::error_code ignored;
                acceptor_->cancel(ignored);
            },
            config_.connect_timeout, "TCP accept");
        configure_connected_socket();
    }

    void connect()
    {
        if (config_.endpoint_role == role::server)
        {
            listen();
            accept();
            return;
        }

        std::scoped_lock lock(operation_mutex_);
        std::error_code ec;
        const auto address = asio::ip::make_address(config_.target_address, ec);
        if (ec || !address.is_v4())
            throw transport_error(error_code::configuration_error, "target_address must be a valid IPv4 address.");

        close_socket_noexcept();
        socket_ = asio::ip::tcp::socket(io_);
        const asio::ip::tcp::endpoint endpoint(address, config_.port);
        run_timed_control(
            [this, endpoint](auto handler) { socket_.async_connect(endpoint, std::move(handler)); },
            [this]() { close_socket_noexcept(); }, config_.connect_timeout, "TCP connect");
        configure_connected_socket();
    }

    void close() noexcept
    {
        std::scoped_lock lock(operation_mutex_);
        close_socket_noexcept();
        close_acceptor_noexcept();
    }

    [[nodiscard]] bool connected() const noexcept
    {
        return connected_.load();
    }

    [[nodiscard]] bool listening() const noexcept
    {
        return acceptor_ && acceptor_->is_open();
    }

    [[nodiscard]] std::uint16_t local_port() const
    {
        std::error_code ec;
        if (acceptor_ && acceptor_->is_open())
        {
            const auto endpoint = acceptor_->local_endpoint(ec);
            if (!ec)
                return endpoint.port();
        }
        if (socket_.is_open())
        {
            const auto endpoint = socket_.local_endpoint(ec);
            if (!ec)
                return endpoint.port();
        }
        return 0U;
    }

    void send(const frame& outgoing)
    {
        std::scoped_lock lock(operation_mutex_);
        send_unlocked(outgoing);
    }

    frame receive()
    {
        std::scoped_lock lock(operation_mutex_);
        return receive_unlocked();
    }

    frame transact(frame_kind request_kind, frame_kind response_kind, const std::vector<std::uint8_t>& payload,
                   std::uint32_t flags = 0U)
    {
        std::scoped_lock lock(operation_mutex_);
        const std::uint64_t sequence = next_sequence_.fetch_add(1U);
        frame request;
        request.header.kind = request_kind;
        request.header.flags = flags;
        request.header.sequence = sequence;
        request.payload = payload;
        send_unlocked(request);

        frame response = receive_unlocked();
        if (response.header.sequence != sequence)
        {
            close_socket_noexcept();
            throw transport_error(error_code::sequence_mismatch, "TCP response sequence does not match request.");
        }
        if (response.header.kind != response_kind)
        {
            close_socket_noexcept();
            throw transport_error(error_code::unexpected_frame, "TCP response kind does not match request.");
        }
        return response;
    }

    frame ping()
    {
        return transact(frame_kind::heartbeat, frame_kind::heartbeat_response, {});
    }

  private:
    template <typename StartOperation, typename CancelOperation>
    std::size_t run_timed_io(StartOperation start_operation, CancelOperation cancel_operation,
                             std::chrono::milliseconds timeout, const char* operation_name)
    {
        std::error_code operation_error = asio::error::would_block;
        std::size_t transferred = 0U;
        bool completed = false;
        bool timed_out = false;
        asio::steady_timer timer(io_);
        timer.expires_after(timeout);

        start_operation([&](const std::error_code& ec, std::size_t count) {
            operation_error = ec;
            transferred = count;
            completed = true;
            std::error_code ignored;
            timer.cancel(ignored);
        });
        timer.async_wait([&](const std::error_code& ec) {
            if (!ec && !completed)
            {
                timed_out = true;
                cancel_operation();
            }
        });

        io_.restart();
        io_.run();
        if (timed_out)
        {
            close_socket_noexcept();
            throw transport_error(error_code::timeout, std::string(operation_name) + " timed out.");
        }
        if (operation_error)
            throw_io_error(operation_error, operation_name);
        return transferred;
    }

    template <typename StartOperation, typename CancelOperation>
    void run_timed_control(StartOperation start_operation, CancelOperation cancel_operation,
                           std::chrono::milliseconds timeout, const char* operation_name)
    {
        std::error_code operation_error = asio::error::would_block;
        bool completed = false;
        bool timed_out = false;
        asio::steady_timer timer(io_);
        timer.expires_after(timeout);

        start_operation([&](const std::error_code& ec) {
            operation_error = ec;
            completed = true;
            std::error_code ignored;
            timer.cancel(ignored);
        });
        timer.async_wait([&](const std::error_code& ec) {
            if (!ec && !completed)
            {
                timed_out = true;
                cancel_operation();
            }
        });

        io_.restart();
        io_.run();
        if (timed_out)
        {
            close_socket_noexcept();
            throw transport_error(error_code::timeout, std::string(operation_name) + " timed out.");
        }
        if (operation_error)
            throw_io_error(operation_error, operation_name);
    }

    void configure_connected_socket()
    {
        std::error_code ec;
        socket_.set_option(asio::ip::tcp::no_delay(config_.no_delay), ec);
        if (!ec)
            socket_.set_option(asio::socket_base::keep_alive(config_.keep_alive), ec);
        if (ec)
        {
            close_socket_noexcept();
            throw transport_error(error_code::io_error, "Cannot configure TCP socket: " + ec.message());
        }
        connected_.store(true);
    }

    void send_unlocked(const frame& outgoing)
    {
        require_connection();
        if (outgoing.payload.size() > config_.max_payload || outgoing.payload.size() > UINT32_MAX)
            throw transport_error(error_code::payload_too_large, "TCP payload exceeds configured maximum.");
        if (!detail::is_known_kind(outgoing.header.kind))
            throw transport_error(error_code::protocol_error, "Cannot send an unknown TCP frame kind.");

        frame_header header = outgoing.header;
        header.magic = frame_magic;
        header.version = protocol_version;
        header.payload_size = static_cast<std::uint32_t>(outgoing.payload.size());
        const auto encoded_header = encode_header(header);

        std::vector<std::uint8_t> wire;
        wire.reserve(encoded_header.size() + outgoing.payload.size());
        wire.insert(wire.end(), encoded_header.begin(), encoded_header.end());
        wire.insert(wire.end(), outgoing.payload.begin(), outgoing.payload.end());
        run_timed_io(
            [this, &wire](auto handler) { asio::async_write(socket_, asio::buffer(wire), std::move(handler)); },
            [this]() {
                std::error_code ignored;
                socket_.cancel(ignored);
            },
            config_.io_timeout, "TCP write");
    }

    frame receive_unlocked()
    {
        require_connection();
        std::array<std::uint8_t, wire_header_size> encoded_header{};
        run_timed_io(
            [this, &encoded_header](auto handler) {
                asio::async_read(socket_, asio::buffer(encoded_header), std::move(handler));
            },
            [this]() {
                std::error_code ignored;
                socket_.cancel(ignored);
            },
            config_.io_timeout, "TCP header read");

        frame incoming;
        try
        {
            incoming.header = decode_header(encoded_header.data(), encoded_header.size());
        }
        catch (...)
        {
            close_socket_noexcept();
            throw;
        }
        if (incoming.header.payload_size > config_.max_payload)
        {
            close_socket_noexcept();
            throw transport_error(error_code::payload_too_large, "Received TCP payload exceeds configured maximum.");
        }

        incoming.payload.resize(incoming.header.payload_size);
        if (!incoming.payload.empty())
        {
            run_timed_io(
                [this, &incoming](auto handler) {
                    asio::async_read(socket_, asio::buffer(incoming.payload), std::move(handler));
                },
                [this]() {
                    std::error_code ignored;
                    socket_.cancel(ignored);
                },
                config_.io_timeout, "TCP payload read");
        }
        return incoming;
    }

    void require_connection() const
    {
        if (!connected_.load() || !socket_.is_open())
            throw transport_error(error_code::disconnected, "TCP connection is not open.");
    }

    [[noreturn]] void throw_io_error(const std::error_code& ec, const char* operation_name)
    {
        close_socket_noexcept();
        const bool disconnected = ec == asio::error::eof || ec == asio::error::connection_reset ||
                                  ec == asio::error::connection_aborted || ec == asio::error::broken_pipe ||
                                  ec == asio::error::not_connected || ec == asio::error::operation_aborted;
        throw transport_error(disconnected ? error_code::disconnected : error_code::io_error,
                              std::string(operation_name) + " failed: " + ec.message());
    }

    void close_socket_noexcept() noexcept
    {
        connected_.store(false);
        std::error_code ignored;
        if (socket_.is_open())
        {
            socket_.cancel(ignored);
            socket_.shutdown(asio::ip::tcp::socket::shutdown_both, ignored);
            socket_.close(ignored);
        }
    }

    void close_acceptor_noexcept() noexcept
    {
        std::error_code ignored;
        if (acceptor_ && acceptor_->is_open())
        {
            acceptor_->cancel(ignored);
            acceptor_->close(ignored);
        }
    }

    configuration config_;
    mutable std::mutex operation_mutex_;
    asio::io_context io_;
    asio::ip::tcp::socket socket_;
    std::unique_ptr<asio::ip::tcp::acceptor> acceptor_;
    std::atomic<bool> connected_{false};
    std::atomic<std::uint64_t> next_sequence_;
};

} // namespace gmp::sil::tcp_transport
