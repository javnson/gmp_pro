/**
 * @file gmp_router_protocol.c
 * @brief Binary encoding implementation for the GMP peripheral router.
 */

#include "gmp_router_protocol.h"

static void put_u16(uint8_t* output, uint16_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8U);
}

static void put_u32(uint8_t* output, uint32_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8U);
    output[2] = (uint8_t)(value >> 16U);
    output[3] = (uint8_t)(value >> 24U);
}

static uint16_t get_u16(const uint8_t* input)
{
    return (uint16_t)((uint16_t)input[0] | ((uint16_t)input[1] << 8U));
}

static uint32_t get_u32(const uint8_t* input)
{
    return (uint32_t)input[0] | ((uint32_t)input[1] << 8U) |
           ((uint32_t)input[2] << 16U) | ((uint32_t)input[3] << 24U);
}

uint16_t gmp_router_crc16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index;
    uint8_t bit;
    for (index = 0U; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8U;
        for (bit = 0U; bit < 8U; ++bit)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1U) ^ 0x1021U) : (uint16_t)(crc << 1U);
    }
    return crc;
}

int gmp_router_packet_encode(const gmp_router_header_t* header, const uint8_t* payload,
                             uint8_t* output, size_t output_capacity, size_t* output_length)
{
    size_t packet_length;
    uint16_t crc;
    size_t index;
    if (header == NULL || output == NULL || output_length == NULL ||
        header->payload_length > GMP_ROUTER_MAX_PAYLOAD ||
        (header->payload_length != 0U && payload == NULL))
        return GMP_ROUTER_STATUS_INVALID;
    packet_length = GMP_ROUTER_HEADER_SIZE + header->payload_length + GMP_ROUTER_CRC_SIZE;
    if (output_capacity < packet_length)
        return GMP_ROUTER_STATUS_INVALID;
    put_u16(output, GMP_ROUTER_MAGIC);
    output[2] = GMP_ROUTER_VERSION;
    output[3] = header->message_type;
    put_u32(output + 4U, header->sequence);
    output[8] = header->peripheral;
    output[9] = header->operation;
    put_u16(output + 10U, header->endpoint);
    put_u16(output + 12U, header->channel);
    put_u16(output + 14U, (uint16_t)header->status);
    put_u16(output + 16U, header->payload_length);
    for (index = 0U; index < header->payload_length; ++index)
        output[GMP_ROUTER_HEADER_SIZE + index] = payload[index];
    crc = gmp_router_crc16(output, packet_length - GMP_ROUTER_CRC_SIZE);
    put_u16(output + packet_length - GMP_ROUTER_CRC_SIZE, crc);
    *output_length = packet_length;
    return GMP_ROUTER_STATUS_OK;
}

int gmp_router_packet_decode(const uint8_t* packet, size_t packet_length,
                             gmp_router_header_t* header, const uint8_t** payload)
{
    uint16_t payload_length;
    uint16_t expected_crc;
    if (packet == NULL || header == NULL || payload == NULL || packet_length < GMP_ROUTER_HEADER_SIZE + GMP_ROUTER_CRC_SIZE)
        return GMP_ROUTER_STATUS_INVALID;
    if (get_u16(packet) != GMP_ROUTER_MAGIC || packet[2] != GMP_ROUTER_VERSION)
        return GMP_ROUTER_STATUS_INVALID;
    payload_length = get_u16(packet + 16U);
    if (payload_length > GMP_ROUTER_MAX_PAYLOAD || packet_length != GMP_ROUTER_HEADER_SIZE + payload_length + GMP_ROUTER_CRC_SIZE)
        return GMP_ROUTER_STATUS_INVALID;
    expected_crc = get_u16(packet + packet_length - GMP_ROUTER_CRC_SIZE);
    if (gmp_router_crc16(packet, packet_length - GMP_ROUTER_CRC_SIZE) != expected_crc)
        return GMP_ROUTER_STATUS_INVALID;
    header->message_type = packet[3];
    header->sequence = get_u32(packet + 4U);
    header->peripheral = packet[8];
    header->operation = packet[9];
    header->endpoint = get_u16(packet + 10U);
    header->channel = get_u16(packet + 12U);
    header->status = (int16_t)get_u16(packet + 14U);
    header->payload_length = payload_length;
    *payload = packet + GMP_ROUTER_HEADER_SIZE;
    return GMP_ROUTER_STATUS_OK;
}

int gmp_router_cobs_encode(const uint8_t* input, size_t input_length, uint8_t* output,
                           size_t output_capacity, size_t* output_length)
{
    size_t read_index = 0U, write_index = 1U, code_index = 0U;
    uint8_t code = 1U;
    if (output == NULL || output_length == NULL || (input_length != 0U && input == NULL) || output_capacity == 0U)
        return GMP_ROUTER_STATUS_INVALID;
    while (read_index < input_length) {
        if (input[read_index] == 0U) {
            if (code_index >= output_capacity) return GMP_ROUTER_STATUS_INVALID;
            output[code_index] = code;
            code = 1U;
            code_index = write_index++;
        } else {
            if (write_index >= output_capacity) return GMP_ROUTER_STATUS_INVALID;
            output[write_index++] = input[read_index];
            if (++code == 0xFFU) {
                if (code_index >= output_capacity) return GMP_ROUTER_STATUS_INVALID;
                output[code_index] = code;
                code = 1U;
                code_index = write_index++;
            }
        }
        ++read_index;
    }
    if (code_index >= output_capacity) return GMP_ROUTER_STATUS_INVALID;
    output[code_index] = code;
    *output_length = write_index;
    return GMP_ROUTER_STATUS_OK;
}

int gmp_router_cobs_decode(const uint8_t* input, size_t input_length, uint8_t* output,
                           size_t output_capacity, size_t* output_length)
{
    size_t read_index = 0U, write_index = 0U;
    if (input == NULL || output == NULL || output_length == NULL || input_length == 0U)
        return GMP_ROUTER_STATUS_INVALID;
    while (read_index < input_length) {
        uint8_t code = input[read_index++];
        uint8_t index;
        if (code == 0U || read_index + (size_t)code - 1U > input_length)
            return GMP_ROUTER_STATUS_INVALID;
        for (index = 1U; index < code; ++index) {
            if (write_index >= output_capacity) return GMP_ROUTER_STATUS_INVALID;
            output[write_index++] = input[read_index++];
        }
        if (code != 0xFFU && read_index < input_length) {
            if (write_index >= output_capacity) return GMP_ROUTER_STATUS_INVALID;
            output[write_index++] = 0U;
        }
    }
    *output_length = write_index;
    return GMP_ROUTER_STATUS_OK;
}
