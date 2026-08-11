/**
 * @file gmp_sil_protocol.hpp
 * @brief Transport-independent wire protocol for GMP SIL communication.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace gmp::sil::protocol
{

constexpr std::uint32_t frame_magic = 0x474D5054U; // ASCII "GMPT".
constexpr std::uint16_t protocol_version = 1U;
constexpr std::size_t wire_header_size = 24U;
constexpr std::size_t state_payload_size = 16U;
constexpr std::size_t session_id_size = 16U;
constexpr std::size_t session_payload_size = 28U;
// Do not trust a link on one plausible packet alone.  Ten consecutive,
// fully validated receive frames must pass before the transport switches
// from its short startup timeout to the long debugger-friendly timeout.
constexpr std::uint64_t startup_qualification_frames = 10U;

enum class frame_kind : std::uint16_t
{
    data_request = 1U,
    data_response = 2U,
    command = 3U,
    command_response = 4U,
    heartbeat = 5U,
    heartbeat_response = 6U,
    error = 7U,
    simulation_state = 8U,
    simulation_state_response = 9U,
    session_hello = 10U,
    session_hello_response = 11U
};

inline const char* frame_kind_name(frame_kind kind) noexcept
{
    switch (kind)
    {
    case frame_kind::data_request: return "data_request";
    case frame_kind::data_response: return "data_response";
    case frame_kind::command: return "command";
    case frame_kind::command_response: return "command_response";
    case frame_kind::heartbeat: return "heartbeat";
    case frame_kind::heartbeat_response: return "heartbeat_response";
    case frame_kind::error: return "error";
    case frame_kind::simulation_state: return "simulation_state";
    case frame_kind::simulation_state_response: return "simulation_state_response";
    case frame_kind::session_hello: return "session_hello";
    case frame_kind::session_hello_response: return "session_hello_response";
    default: return "unknown";
    }
}

enum class simulation_state : std::uint16_t
{
    starting = 1U,
    running = 2U,
    completed = 3U,
    aborted = 4U,
    faulted = 5U
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

struct simulation_status
{
    simulation_state state = simulation_state::starting;
    std::uint32_t flags = 0U;
    std::uint64_t major_step = 0U;
};

using session_id = std::array<std::uint8_t, session_id_size>;

struct session_descriptor
{
    session_id id{};
    std::uint32_t request_payload_size = 0U;  //!< Simulink to controller.
    std::uint32_t response_payload_size = 0U; //!< Controller to Simulink.
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
    for (std::size_t index = 0U; index < 8U; ++index)
        output[index] = static_cast<std::uint8_t>((value >> ((7U - index) * 8U)) & 0xFFU);
}

inline std::uint16_t get_u16(const std::uint8_t* input)
{
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(input[0]) << 8U) | input[1]);
}

inline std::uint32_t get_u32(const std::uint8_t* input)
{
    return (static_cast<std::uint32_t>(input[0]) << 24U) | (static_cast<std::uint32_t>(input[1]) << 16U) |
           (static_cast<std::uint32_t>(input[2]) << 8U) | static_cast<std::uint32_t>(input[3]);
}

inline std::uint64_t get_u64(const std::uint8_t* input)
{
    std::uint64_t value = 0U;
    for (std::size_t index = 0U; index < 8U; ++index)
        value = (value << 8U) | input[index];
    return value;
}

inline bool valid_kind(frame_kind kind)
{
    const auto value = static_cast<std::uint16_t>(kind);
    return value >= static_cast<std::uint16_t>(frame_kind::data_request) &&
           value <= static_cast<std::uint16_t>(frame_kind::session_hello_response);
}

inline bool valid_state(simulation_state state)
{
    const auto value = static_cast<std::uint16_t>(state);
    return value >= static_cast<std::uint16_t>(simulation_state::starting) &&
           value <= static_cast<std::uint16_t>(simulation_state::faulted);
}

} // namespace detail

inline std::array<std::uint8_t, wire_header_size> encode_header(const frame_header& header)
{
    if (!detail::valid_kind(header.kind))
        throw std::invalid_argument("Unknown GMP SIL frame kind.");
    std::array<std::uint8_t, wire_header_size> output{};
    detail::put_u32(output.data(), frame_magic);
    detail::put_u16(output.data() + 4U, protocol_version);
    detail::put_u16(output.data() + 6U, static_cast<std::uint16_t>(header.kind));
    detail::put_u32(output.data() + 8U, header.flags);
    detail::put_u64(output.data() + 12U, header.sequence);
    detail::put_u32(output.data() + 20U, header.payload_size);
    return output;
}

inline frame_header decode_header(const std::uint8_t* data, std::size_t size)
{
    if (data == nullptr || size != wire_header_size)
        throw std::invalid_argument("Invalid GMP SIL frame header size.");
    frame_header header;
    header.magic = detail::get_u32(data);
    header.version = detail::get_u16(data + 4U);
    header.kind = static_cast<frame_kind>(detail::get_u16(data + 6U));
    header.flags = detail::get_u32(data + 8U);
    header.sequence = detail::get_u64(data + 12U);
    header.payload_size = detail::get_u32(data + 20U);
    if (header.magic != frame_magic || header.version != protocol_version || !detail::valid_kind(header.kind))
        throw std::invalid_argument("Invalid or unsupported GMP SIL frame header.");
    return header;
}

inline std::array<std::uint8_t, state_payload_size> encode_status(const simulation_status& status)
{
    if (!detail::valid_state(status.state))
        throw std::invalid_argument("Unknown GMP SIL simulation state.");
    std::array<std::uint8_t, state_payload_size> output{};
    detail::put_u16(output.data(), protocol_version);
    detail::put_u16(output.data() + 2U, static_cast<std::uint16_t>(status.state));
    detail::put_u32(output.data() + 4U, status.flags);
    detail::put_u64(output.data() + 8U, status.major_step);
    return output;
}

inline simulation_status decode_status(const std::vector<std::uint8_t>& payload)
{
    if (payload.size() != state_payload_size || detail::get_u16(payload.data()) != protocol_version)
        throw std::invalid_argument("Invalid GMP SIL simulation-state payload.");
    simulation_status status;
    status.state = static_cast<simulation_state>(detail::get_u16(payload.data() + 2U));
    status.flags = detail::get_u32(payload.data() + 4U);
    status.major_step = detail::get_u64(payload.data() + 8U);
    if (!detail::valid_state(status.state))
        throw std::invalid_argument("Unknown GMP SIL simulation state.");
    return status;
}

inline std::vector<std::uint8_t> make_status_payload(const simulation_status& status)
{
    const auto encoded = encode_status(status);
    return {encoded.begin(), encoded.end()};
}

inline std::array<std::uint8_t, session_payload_size> encode_session(const session_descriptor& descriptor)
{
    if (descriptor.request_payload_size == 0U || descriptor.response_payload_size == 0U)
        throw std::invalid_argument("GMP SIL session payload sizes must be positive.");
    std::array<std::uint8_t, session_payload_size> output{};
    detail::put_u16(output.data(), protocol_version);
    detail::put_u16(output.data() + 2U, 0U);
    detail::put_u32(output.data() + 4U, descriptor.request_payload_size);
    detail::put_u32(output.data() + 8U, descriptor.response_payload_size);
    for (std::size_t index = 0U; index < descriptor.id.size(); ++index)
        output[12U + index] = descriptor.id[index];
    return output;
}

inline session_descriptor decode_session(const std::vector<std::uint8_t>& payload)
{
    if (payload.size() != session_payload_size || detail::get_u16(payload.data()) != protocol_version)
        throw std::invalid_argument("Invalid GMP SIL session payload.");
    session_descriptor descriptor;
    descriptor.request_payload_size = detail::get_u32(payload.data() + 4U);
    descriptor.response_payload_size = detail::get_u32(payload.data() + 8U);
    for (std::size_t index = 0U; index < descriptor.id.size(); ++index)
        descriptor.id[index] = payload[12U + index];
    if (descriptor.request_payload_size == 0U || descriptor.response_payload_size == 0U)
        throw std::invalid_argument("GMP SIL session payload sizes must be positive.");
    return descriptor;
}

inline std::vector<std::uint8_t> make_session_payload(const session_descriptor& descriptor)
{
    const auto encoded = encode_session(descriptor);
    return {encoded.begin(), encoded.end()};
}

inline session_id parse_session_id(const std::string& text)
{
    if (text.size() != session_id_size * 2U)
        throw std::invalid_argument("GMP SIL connection_id must contain 32 hexadecimal characters.");
    session_id result{};
    const auto nibble = [](char value) -> std::uint8_t {
        if (value >= '0' && value <= '9')
            return static_cast<std::uint8_t>(value - '0');
        if (value >= 'a' && value <= 'f')
            return static_cast<std::uint8_t>(value - 'a' + 10);
        if (value >= 'A' && value <= 'F')
            return static_cast<std::uint8_t>(value - 'A' + 10);
        throw std::invalid_argument("GMP SIL connection_id contains a non-hexadecimal character.");
    };
    for (std::size_t index = 0U; index < result.size(); ++index)
        result[index] = static_cast<std::uint8_t>((nibble(text[index * 2U]) << 4U) | nibble(text[index * 2U + 1U]));
    return result;
}

inline std::string format_session_id(const session_id& id)
{
    constexpr char digits[] = "0123456789abcdef";
    std::string output(id.size() * 2U, '0');
    for (std::size_t index = 0U; index < id.size(); ++index)
    {
        output[index * 2U] = digits[(id[index] >> 4U) & 0x0FU];
        output[index * 2U + 1U] = digits[id[index] & 0x0FU];
    }
    return output;
}

} // namespace gmp::sil::protocol
