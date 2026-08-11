/**
 * @file framed_udp_helper.hpp
 * @brief Timeout-bounded, framed UDP transport for GMP SIL.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#if defined(_WIN32)
#include <SDKDDKVer.h>
#include <mstcpip.h>
#endif

#ifndef ASIO_STANDALONE
#define ASIO_STANDALONE
#endif
#include <asio.hpp>

#include <tools/gmp_sil/sil_helper/gmp_sil_protocol.hpp>

namespace gmp::sil::udp_transport
{

constexpr std::uint32_t maximum_udp_payload = 65507U;
constexpr std::uint32_t default_max_payload = maximum_udp_payload - protocol::wire_header_size;

enum class role
{
    client,
    server
};

enum class error_code
{
    timeout,
    aborted,
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

struct configuration
{
    role endpoint_role = role::client;
    std::string target_address = "127.0.0.1";
    std::string bind_address = "0.0.0.0";
    std::uint16_t port = 12510U;       //!< Remote port for a client; local listen port for a server.
    std::uint16_t local_port = 0U;     //!< Optional fixed client receive port.
    std::chrono::milliseconds startup_io_timeout{5000};
    std::chrono::milliseconds established_io_timeout{2000000};
    bool startup_timeout_enabled = true; //!< False waits for the first peer frame until abort().
    std::uint64_t established_after_frames = protocol::startup_qualification_frames;
    std::uint32_t max_payload = default_max_payload;
};

class framed_udp_helper
{
  public:
    explicit framed_udp_helper(configuration config)
        : config_(std::move(config)), socket_(io_), next_sequence_(1U)
    {
        if (config_.startup_io_timeout.count() <= 0 || config_.established_io_timeout.count() <= 0 ||
            config_.max_payload == 0U || config_.max_payload > default_max_payload || config_.port == 0U)
            throw transport_error(error_code::configuration_error, "Invalid GMP SIL UDP configuration.");
    }

    framed_udp_helper(const framed_udp_helper&) = delete;
    framed_udp_helper& operator=(const framed_udp_helper&) = delete;

    ~framed_udp_helper()
    {
        close();
    }

    void open()
    {
        std::scoped_lock lock(operation_mutex_);
        close_socket_noexcept();
        abort_requested_.store(false);
        completed_receive_frames_.store(0U);
        next_sequence_.store(1U);
        peer_known_.store(false);
        peer_locked_.store(false);

        std::error_code ec;
        const auto bind_address = asio::ip::make_address(config_.bind_address, ec);
        if (ec || !bind_address.is_v4())
            throw transport_error(error_code::configuration_error, "UDP bind_address must be numeric IPv4.");

        socket_.open(asio::ip::udp::v4(), ec);
        if (!ec)
            socket_.set_option(asio::socket_base::reuse_address(true), ec);
        const auto bind_port = config_.endpoint_role == role::server ? config_.port : config_.local_port;
        if (!ec)
            socket_.bind(asio::ip::udp::endpoint(bind_address, bind_port), ec);
        if (ec)
        {
            close_socket_noexcept();
            throw transport_error(error_code::io_error, "Cannot open GMP SIL UDP socket: " + ec.message());
        }

#if defined(_WIN32)
        // Windows reports an ICMP "port unreachable" as WSAECONNRESET on a
        // later UDP receive.  Redundant terminal-state datagrams can trigger
        // that condition after the peer has already closed; treating it as a
        // stream reset poisons the next SIL session, so request BSD behavior.
        BOOL report_udp_reset = FALSE;
        DWORD returned = 0U;
        if (::WSAIoctl(socket_.native_handle(), SIO_UDP_CONNRESET, &report_udp_reset,
                       sizeof(report_udp_reset), nullptr, 0U, &returned, nullptr, nullptr) == SOCKET_ERROR)
        {
            const auto native_error = ::WSAGetLastError();
            close_socket_noexcept();
            throw transport_error(error_code::io_error,
                                  "Cannot configure GMP SIL UDP reset behavior: " +
                                      std::system_category().message(native_error));
        }
#endif

        if (config_.endpoint_role == role::client)
        {
            const auto target_address = asio::ip::make_address(config_.target_address, ec);
            if (ec || !target_address.is_v4())
            {
                close_socket_noexcept();
                throw transport_error(error_code::configuration_error,
                                      "UDP target_address must be numeric IPv4.");
            }
            peer_ = asio::ip::udp::endpoint(target_address, config_.port);
            peer_known_.store(true);
        }
        opened_.store(true);
    }

    void abort() noexcept
    {
        abort_requested_.store(true);
        opened_.store(false);
        if (operation_active_.load())
        {
            asio::post(io_, [this]() {
                if (abort_requested_.load())
                    close_socket_noexcept();
            });
            if (!operation_active_.load())
                close_socket_noexcept();
            return;
        }
        close_socket_noexcept();
    }

    void close() noexcept
    {
        abort();
        std::scoped_lock lock(operation_mutex_);
        close_socket_noexcept();
    }

    [[nodiscard]] bool opened() const noexcept
    {
        return opened_.load();
    }

    [[nodiscard]] std::uint16_t local_port() const noexcept
    {
        std::error_code ec;
        const auto endpoint = socket_.local_endpoint(ec);
        return ec ? 0U : endpoint.port();
    }

    [[nodiscard]] std::chrono::milliseconds effective_io_timeout() const noexcept
    {
        return completed_receive_frames_.load() >= config_.established_after_frames
                   ? config_.established_io_timeout
                   : config_.startup_io_timeout;
    }

    void lock_peer()
    {
        if (config_.endpoint_role != role::server || !peer_known_.load())
            throw transport_error(error_code::configuration_error,
                                  "A UDP server can lock its peer only after a valid session hello.");
        peer_locked_.store(true);
    }

    void send(const protocol::frame& outgoing)
    {
        std::scoped_lock lock(operation_mutex_);
        send_unlocked(outgoing);
    }

    protocol::frame receive()
    {
        std::scoped_lock lock(operation_mutex_);
        return receive_unlocked();
    }

    protocol::frame transact(protocol::frame_kind request_kind, protocol::frame_kind response_kind,
                             const std::vector<std::uint8_t>& payload, std::uint32_t flags = 0U)
    {
        std::scoped_lock lock(operation_mutex_);
        protocol::frame request;
        request.header.kind = request_kind;
        request.header.flags = flags;
        request.header.sequence = next_sequence_.fetch_add(1U);
        request.payload = payload;
        send_unlocked(request);
        auto response = receive_unlocked();
        if (response.header.sequence != request.header.sequence)
        {
            close_socket_noexcept();
            throw transport_error(error_code::sequence_mismatch, "UDP response sequence does not match request.");
        }
        if (response.header.kind != response_kind)
        {
            close_socket_noexcept();
            throw transport_error(error_code::unexpected_frame, "UDP response kind does not match request.");
        }
        return response;
    }

  private:
    template <typename StartOperation>
    std::size_t run_timed(StartOperation start_operation, const char* operation_name)
    {
        std::error_code operation_error = asio::error::would_block;
        std::size_t transferred = 0U;
        bool completed = false;
        bool timed_out = false;
        const bool timeout_enabled = completed_receive_frames_.load() >= config_.established_after_frames ||
                                     config_.startup_timeout_enabled;
        std::unique_ptr<asio::steady_timer> timer;
        if (timeout_enabled)
        {
            timer = std::make_unique<asio::steady_timer>(io_);
            timer->expires_after(effective_io_timeout());
        }
        operation_active_.store(true);
        try
        {
            start_operation([&](const std::error_code& ec, std::size_t count) {
                operation_error = ec;
                transferred = count;
                completed = true;
                std::error_code ignored;
                if (timer)
                    timer->cancel(ignored);
            });
            if (timer)
                timer->async_wait([&](const std::error_code& ec) {
                    if (!ec && !completed)
                    {
                        timed_out = true;
                        std::error_code ignored;
                        socket_.cancel(ignored);
                    }
                });
            io_.restart();
            io_.run();
        }
        catch (...)
        {
            operation_active_.store(false);
            throw;
        }
        operation_active_.store(false);
        if (abort_requested_.load())
            throw transport_error(error_code::aborted, std::string(operation_name) + " was actively aborted.");
        if (timed_out)
        {
            close_socket_noexcept();
            throw transport_error(error_code::timeout, std::string(operation_name) + " timed out.");
        }
        if (operation_error)
        {
            close_socket_noexcept();
            throw transport_error(error_code::io_error,
                                  std::string(operation_name) + " failed: " + operation_error.message());
        }
        return transferred;
    }

    void send_unlocked(const protocol::frame& outgoing)
    {
        require_open();
        if (!peer_known_.load())
            throw transport_error(error_code::disconnected, "UDP peer is unknown until the first request arrives.");
        if (outgoing.payload.size() > config_.max_payload)
            throw transport_error(error_code::payload_too_large, "UDP payload exceeds configured maximum.");

        protocol::frame_header header = outgoing.header;
        header.payload_size = static_cast<std::uint32_t>(outgoing.payload.size());
        const auto encoded = protocol::encode_header(header);
        std::vector<std::uint8_t> datagram;
        datagram.reserve(encoded.size() + outgoing.payload.size());
        datagram.insert(datagram.end(), encoded.begin(), encoded.end());
        datagram.insert(datagram.end(), outgoing.payload.begin(), outgoing.payload.end());
        const auto transferred = run_timed(
            [this, &datagram](auto handler) {
                socket_.async_send_to(asio::buffer(datagram), peer_, std::move(handler));
            },
            "UDP frame write");
        if (transferred != datagram.size())
            throw transport_error(error_code::io_error, "UDP frame write was incomplete.");
    }

    protocol::frame receive_unlocked()
    {
        require_open();
        std::vector<std::uint8_t> datagram(protocol::wire_header_size + config_.max_payload);
        asio::ip::udp::endpoint sender;
        const auto transferred = run_timed(
            [this, &datagram, &sender](auto handler) {
                socket_.async_receive_from(asio::buffer(datagram), sender, std::move(handler));
            },
            "UDP frame read");
        if (transferred < protocol::wire_header_size)
            throw transport_error(error_code::protocol_error, "UDP datagram is shorter than the GMP SIL header.");

        protocol::frame incoming;
        try
        {
            incoming.header = protocol::decode_header(datagram.data(), protocol::wire_header_size);
        }
        catch (const std::exception& exception)
        {
            throw transport_error(error_code::protocol_error, exception.what());
        }
        const auto actual_payload = transferred - protocol::wire_header_size;
        if (incoming.header.payload_size != actual_payload)
            throw transport_error(error_code::protocol_error, "UDP frame payload length does not match its header.");
        incoming.payload.assign(datagram.begin() + protocol::wire_header_size, datagram.begin() + transferred);
        if (config_.endpoint_role == role::server)
        {
            if (peer_locked_.load() && sender != peer_)
                throw transport_error(error_code::protocol_error,
                                      "UDP datagram does not belong to the locked GMP SIL session.");
            peer_ = sender;
            peer_known_.store(true);
        }
        else if (sender != peer_)
        {
            throw transport_error(error_code::protocol_error, "UDP response arrived from an unexpected endpoint.");
        }
        completed_receive_frames_.fetch_add(1U);
        return incoming;
    }

    void require_open() const
    {
        if (!opened_.load() || !socket_.is_open())
            throw transport_error(error_code::disconnected, "GMP SIL UDP socket is not open.");
    }

    void close_socket_noexcept() noexcept
    {
        opened_.store(false);
        std::error_code ignored;
        if (socket_.is_open())
        {
            socket_.cancel(ignored);
            socket_.close(ignored);
        }
    }

    configuration config_;
    mutable std::mutex operation_mutex_;
    asio::io_context io_;
    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint peer_;
    std::atomic<bool> peer_known_{false};
    std::atomic<bool> peer_locked_{false};
    std::atomic<bool> opened_{false};
    std::atomic<bool> abort_requested_{false};
    std::atomic<bool> operation_active_{false};
    std::atomic<std::uint64_t> completed_receive_frames_{0U};
    std::atomic<std::uint64_t> next_sequence_;
};

} // namespace gmp::sil::udp_transport
