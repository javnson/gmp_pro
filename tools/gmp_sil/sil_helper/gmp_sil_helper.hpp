/**
 * @file gmp_sil_helper.hpp
 * @brief One public GMP SIL connection object for TCP and UDP transports.
 *
 * Applications, Simulink S-functions and command-line tools should depend on
 * this header instead of directly selecting a transport helper.
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <tools/gmp_sil/sil_helper/gmp_sil_protocol.hpp>
#include <tools/gmp_sil/sil_helper/asio_tcp_helper.hpp>
#include <tools/gmp_sil/sil_helper/framed_udp_helper.hpp>

namespace gmp::sil
{

enum class transport_kind
{
    udp,
    tcp
};

enum class endpoint_role
{
    client,
    server
};

class sil_error : public std::runtime_error
{
  public:
    using std::runtime_error::runtime_error;
};

struct configuration
{
    transport_kind transport = transport_kind::udp;
    endpoint_role role = endpoint_role::client;
    std::string target_address = "127.0.0.1";
    std::string bind_address = "0.0.0.0";
    std::uint16_t transmit_port = 12510U; //!< Remote receive port.
    std::uint16_t receive_port = 12511U;  //!< Local receive port (UDP) or listen port (TCP server).
    std::chrono::milliseconds connect_timeout{5000};
    std::chrono::milliseconds startup_io_timeout{5000};
    std::chrono::milliseconds established_io_timeout{2000000};
    bool startup_timeout_enabled = true; //!< First-frame timeout; abort remains available when false.
    std::uint64_t established_after_frames = protocol::startup_qualification_frames;
    std::uint32_t max_payload = 16U * 1024U * 1024U;
    protocol::session_descriptor session;
};

struct received_event
{
    protocol::frame_kind kind = protocol::frame_kind::error;
    std::uint64_t sequence = 0U;
    std::uint32_t flags = 0U;
    std::vector<std::uint8_t> payload;
};

inline configuration parse_configuration(const std::string& path)
{
    std::ifstream stream(path);
    if (!stream.is_open())
        throw sil_error("Cannot open GMP SIL configuration: " + path);
    nlohmann::json document;
    try
    {
        stream >> document;
        if (document.value("schema_version", 1U) != 1U)
            throw sil_error("Unsupported GMP SIL JSON schema version.");

        configuration result;
        const auto transport = document.value("transport", std::string("udp"));
        if (transport == "udp")
            result.transport = transport_kind::udp;
        else if (transport == "tcp")
            result.transport = transport_kind::tcp;
        else
            throw sil_error("GMP SIL transport must be 'udp' or 'tcp'.");

        const auto role = document.value("role", std::string("server"));
        if (role == "client")
            result.role = endpoint_role::client;
        else if (role == "server")
            result.role = endpoint_role::server;
        else
            throw sil_error("GMP SIL role must be 'client' or 'server'.");

        result.target_address = document.value("target_address", result.target_address);
        result.bind_address = document.value("bind_address", result.bind_address);
        const auto transmit_port = document.value("transmit_port", static_cast<unsigned>(result.transmit_port));
        const auto receive_port = document.value("receive_port", static_cast<unsigned>(result.receive_port));
        if (transmit_port == 0U || transmit_port > 65535U || receive_port == 0U || receive_port > 65535U)
            throw sil_error("GMP SIL transmit_port and receive_port must be in [1, 65535].");
        result.transmit_port = static_cast<std::uint16_t>(transmit_port);
        result.receive_port = static_cast<std::uint16_t>(receive_port);

        result.connect_timeout =
            std::chrono::milliseconds(document.value("connect_timeout_ms", result.connect_timeout.count()));
        result.startup_io_timeout = std::chrono::milliseconds(
            document.value("startup_io_timeout_ms", result.startup_io_timeout.count()));
        result.established_io_timeout = std::chrono::milliseconds(
            document.value("established_io_timeout_ms", result.established_io_timeout.count()));
        result.established_after_frames =
            document.value("established_after_frames", result.established_after_frames);
        result.startup_timeout_enabled =
            document.value("startup_timeout_enabled", result.startup_timeout_enabled);
        result.max_payload = document.value("max_payload", result.max_payload);
        if (result.connect_timeout.count() <= 0 || result.startup_io_timeout.count() <= 0 ||
            result.established_io_timeout.count() <= 0 || result.max_payload == 0U)
            throw sil_error("GMP SIL timeouts and max_payload must be positive.");

        result.session.id = protocol::parse_session_id(document.at("connection_id").get<std::string>());
        result.session.request_payload_size = document.at("simulink_to_controller_bytes").get<std::uint32_t>();
        result.session.response_payload_size = document.at("controller_to_simulink_bytes").get<std::uint32_t>();
        (void)protocol::encode_session(result.session);
        if (result.session.request_payload_size > result.max_payload ||
            result.session.response_payload_size > result.max_payload)
            throw sil_error("GMP SIL ABI payload size exceeds max_payload.");
        return result;
    }
    catch (const sil_error&)
    {
        throw;
    }
    catch (const std::exception& exception)
    {
        throw sil_error("Invalid GMP SIL configuration: " + std::string(exception.what()));
    }
}

class gmp_sil_helper
{
  public:
    explicit gmp_sil_helper(configuration config) : config_(std::move(config))
    {
        (void)protocol::encode_session(config_.session);
        if (config_.session.request_payload_size > config_.max_payload ||
            config_.session.response_payload_size > config_.max_payload)
            throw sil_error("GMP SIL ABI payload size exceeds max_payload.");
    }

    static std::unique_ptr<gmp_sil_helper> from_json(const std::string& path)
    {
        return std::make_unique<gmp_sil_helper>(parse_configuration(path));
    }

    gmp_sil_helper(const gmp_sil_helper&) = delete;
    gmp_sil_helper& operator=(const gmp_sil_helper&) = delete;

    ~gmp_sil_helper()
    {
        close();
    }

    /** Open the selected transport and complete the unique-session handshake. */
    void connect()
    {
        session_established_ = false;
        if (config_.transport == transport_kind::tcp)
            connect_tcp();
        else
            connect_udp();
        if (config_.role == endpoint_role::client)
            client_handshake();
        else
            server_handshake();
        session_established_ = true;
    }

    std::vector<std::uint8_t> exchange(const void* data, std::size_t size)
    {
        require_client_session();
        if (data == nullptr || size != config_.session.request_payload_size)
            throw sil_error("Simulink request size does not match the JSON session ABI.");
        const auto* bytes = static_cast<const std::uint8_t*>(data);
        std::vector<std::uint8_t> payload(bytes, bytes + size);
        protocol::frame response;
        if (config_.transport == transport_kind::tcp)
        {
            const auto tcp_response = tcp_->transact(tcp_transport::frame_kind::data_request,
                                                     tcp_transport::frame_kind::data_response, payload);
            response = from_tcp(tcp_response);
        }
        else
        {
            response = udp_->transact(protocol::frame_kind::data_request, protocol::frame_kind::data_response,
                                      payload);
        }
        if (response.payload.size() != config_.session.response_payload_size)
            throw sil_error("Controller response size does not match the JSON session ABI.");
        return response.payload;
    }

    received_event receive_event()
    {
        require_server_session();
        const auto frame = receive_frame();
        if (frame.header.kind == protocol::frame_kind::data_request &&
            frame.payload.size() != config_.session.request_payload_size)
            throw sil_error("Received request size does not match the JSON session ABI.");
        if (frame.header.kind != protocol::frame_kind::data_request &&
            frame.header.kind != protocol::frame_kind::simulation_state &&
            frame.header.kind != protocol::frame_kind::heartbeat)
            throw sil_error("Unexpected GMP SIL frame after session establishment.");
        return {frame.header.kind, frame.header.sequence, frame.header.flags, frame.payload};
    }

    void respond(const received_event& request, const void* data, std::size_t size)
    {
        require_server_session();
        if (request.kind != protocol::frame_kind::data_request || data == nullptr ||
            size != config_.session.response_payload_size)
            throw sil_error("Controller response does not match the JSON session ABI.");
        protocol::frame response;
        response.header.kind = protocol::frame_kind::data_response;
        response.header.sequence = request.sequence;
        const auto* bytes = static_cast<const std::uint8_t*>(data);
        response.payload.assign(bytes, bytes + size);
        send_frame(response);
    }

    void notify_state(protocol::simulation_state state, std::uint64_t major_step, std::uint32_t flags = 0U)
    {
        require_client_session();
        protocol::simulation_status status{state, flags, major_step};
        const auto payload = protocol::make_status_payload(status);
        if (config_.transport == transport_kind::tcp)
        {
            (void)tcp_->transact(tcp_transport::frame_kind::simulation_state,
                                 tcp_transport::frame_kind::simulation_state_response, payload);
            return;
        }

        protocol::frame frame;
        frame.header.kind = protocol::frame_kind::simulation_state;
        frame.header.sequence = state_sequence_++;
        frame.payload = payload;
        const unsigned copies = state == protocol::simulation_state::completed ||
                                        state == protocol::simulation_state::aborted ||
                                        state == protocol::simulation_state::faulted
                                    ? 3U
                                    : 1U;
        for (unsigned index = 0U; index < copies; ++index)
            udp_->send(frame);
    }

    protocol::simulation_status decode_state(const received_event& event) const
    {
        if (event.kind != protocol::frame_kind::simulation_state)
            throw sil_error("GMP SIL event is not a simulation-state frame.");
        return protocol::decode_status(event.payload);
    }

    void acknowledge_state(const received_event& event)
    {
        require_server_session();
        if (event.kind != protocol::frame_kind::simulation_state)
            throw sil_error("Cannot acknowledge a non-state GMP SIL event.");
        if (config_.transport == transport_kind::udp)
            return; // UDP state frames are one-way and terminal states are sent redundantly.
        protocol::frame response;
        response.header.kind = protocol::frame_kind::simulation_state_response;
        response.header.sequence = event.sequence;
        send_frame(response);
    }

    void abort() noexcept
    {
        session_established_ = false;
        if (tcp_)
            tcp_->abort();
        if (udp_)
            udp_->abort();
    }

    void close() noexcept
    {
        session_established_ = false;
        if (tcp_)
            tcp_->close();
        if (udp_)
            udp_->close();
    }

    [[nodiscard]] bool connected() const noexcept
    {
        return session_established_;
    }

    [[nodiscard]] const configuration& config() const noexcept
    {
        return config_;
    }

  private:
    void connect_tcp()
    {
        tcp_transport::configuration config;
        config.endpoint_role = config_.role == endpoint_role::client ? tcp_transport::role::client
                                                                     : tcp_transport::role::server;
        config.target_address = config_.target_address;
        config.bind_address = config_.bind_address;
        config.port = config_.role == endpoint_role::client ? config_.transmit_port : config_.receive_port;
        // The controller is normally launched before MATLAB.  A server must
        // therefore remain blocked waiting for the first session_hello long
        // enough for MATLAB/model start-up and debugger pauses.  Clients keep
        // the short start-up timeout so a bad address fails promptly.
        const bool is_server = config_.role == endpoint_role::server;
        config.connect_timeout = is_server ? config_.established_io_timeout : config_.connect_timeout;
        config.startup_io_timeout = is_server ? config_.established_io_timeout : config_.startup_io_timeout;
        config.established_io_timeout = config_.established_io_timeout;
        config.established_after_frames = config_.established_after_frames;
        config.startup_timeout_enabled = config_.startup_timeout_enabled;
        config.max_payload = config_.max_payload;
        tcp_ = std::make_unique<tcp_transport::asio_tcp_helper>(config);
        tcp_->connect();
    }

    void connect_udp()
    {
        udp_transport::configuration config;
        config.endpoint_role = config_.role == endpoint_role::client ? udp_transport::role::client
                                                                     : udp_transport::role::server;
        config.target_address = config_.target_address;
        config.bind_address = config_.bind_address;
        config.port = config_.role == endpoint_role::client ? config_.transmit_port : config_.receive_port;
        config.local_port = config_.role == endpoint_role::client ? config_.receive_port : 0U;
        config.startup_io_timeout = config_.role == endpoint_role::server ? config_.established_io_timeout
                                                                          : config_.startup_io_timeout;
        config.established_io_timeout = config_.established_io_timeout;
        config.established_after_frames = config_.established_after_frames;
        config.startup_timeout_enabled = config_.startup_timeout_enabled;
        config.max_payload = std::min(config_.max_payload, udp_transport::default_max_payload);
        udp_ = std::make_unique<udp_transport::framed_udp_helper>(config);
        udp_->open();
    }

    void client_handshake()
    {
        const auto payload = protocol::make_session_payload(config_.session);
        protocol::frame response;
        if (config_.transport == transport_kind::tcp)
        {
            response = from_tcp(tcp_->transact(tcp_transport::frame_kind::session_hello,
                                               tcp_transport::frame_kind::session_hello_response, payload));
        }
        else
        {
            response = udp_->transact(protocol::frame_kind::session_hello,
                                      protocol::frame_kind::session_hello_response, payload);
        }
        validate_session(protocol::decode_session(response.payload));
    }

    void server_handshake()
    {
        const auto hello = receive_frame();
        if (hello.header.kind != protocol::frame_kind::session_hello)
            throw sil_error("First GMP SIL frame must be session_hello.");
        validate_session(protocol::decode_session(hello.payload));
        protocol::frame response;
        response.header.kind = protocol::frame_kind::session_hello_response;
        response.header.sequence = hello.header.sequence;
        response.payload = protocol::make_session_payload(config_.session);
        send_frame(response);
        if (udp_)
            udp_->lock_peer();
    }

    void validate_session(const protocol::session_descriptor& remote) const
    {
        if (remote.id != config_.session.id)
            throw sil_error("GMP SIL connection_id does not match (expected " +
                            protocol::format_session_id(config_.session.id) + ", received " +
                            protocol::format_session_id(remote.id) + "); refusing cross-session traffic.");
        if (remote.request_payload_size != config_.session.request_payload_size ||
            remote.response_payload_size != config_.session.response_payload_size)
            throw sil_error("GMP SIL packed-data ABI does not match the JSON session contract.");
    }

    protocol::frame receive_frame()
    {
        return tcp_ ? from_tcp(tcp_->receive()) : udp_->receive();
    }

    void send_frame(const protocol::frame& frame)
    {
        if (tcp_)
            tcp_->send(to_tcp(frame));
        else
            udp_->send(frame);
    }

    static protocol::frame from_tcp(const tcp_transport::frame& source)
    {
        protocol::frame target;
        target.header.kind = static_cast<protocol::frame_kind>(source.header.kind);
        target.header.flags = source.header.flags;
        target.header.sequence = source.header.sequence;
        target.header.payload_size = source.header.payload_size;
        target.payload = source.payload;
        return target;
    }

    static tcp_transport::frame to_tcp(const protocol::frame& source)
    {
        tcp_transport::frame target;
        target.header.kind = static_cast<tcp_transport::frame_kind>(source.header.kind);
        target.header.flags = source.header.flags;
        target.header.sequence = source.header.sequence;
        target.payload = source.payload;
        return target;
    }

    void require_client_session() const
    {
        if (config_.role != endpoint_role::client || !session_established_)
            throw sil_error("GMP SIL client session is not established.");
    }

    void require_server_session() const
    {
        if (config_.role != endpoint_role::server || !session_established_)
            throw sil_error("GMP SIL server session is not established.");
    }

    configuration config_;
    std::unique_ptr<tcp_transport::asio_tcp_helper> tcp_;
    std::unique_ptr<udp_transport::framed_udp_helper> udp_;
    bool session_established_ = false;
    std::uint64_t state_sequence_ = 0x8000000000000000ULL;
};

} // namespace gmp::sil
