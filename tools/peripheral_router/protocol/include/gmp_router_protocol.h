/**
 * @file gmp_router_protocol.h
 * @brief Transport-independent wire protocol for GMP peripheral routers.
 */

#ifndef GMP_ROUTER_PROTOCOL_H
#define GMP_ROUTER_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GMP_ROUTER_MAGIC 0x4752U
#define GMP_ROUTER_VERSION 1U
#define GMP_ROUTER_HEADER_SIZE 18U
#define GMP_ROUTER_CRC_SIZE 2U
#define GMP_ROUTER_MAX_PAYLOAD 512U
#define GMP_ROUTER_MAX_PACKET (GMP_ROUTER_HEADER_SIZE + GMP_ROUTER_MAX_PAYLOAD + GMP_ROUTER_CRC_SIZE)
#define GMP_ROUTER_MAX_WIRE_PACKET (GMP_ROUTER_MAX_PACKET + (GMP_ROUTER_MAX_PACKET / 254U) + 2U)

/** Router message class. */
typedef enum {
    GMP_ROUTER_MESSAGE_REQUEST = 1,
    GMP_ROUTER_MESSAGE_RESPONSE = 2,
    GMP_ROUTER_MESSAGE_EVENT = 3
} gmp_router_message_type_t;

/** Peripheral family identifier. */
typedef enum {
    GMP_ROUTER_PERIPHERAL_SYSTEM = 0,
    GMP_ROUTER_PERIPHERAL_GPIO = 1,
    GMP_ROUTER_PERIPHERAL_UART = 2,
    GMP_ROUTER_PERIPHERAL_I2C = 3,
    GMP_ROUTER_PERIPHERAL_SPI = 4,
    GMP_ROUTER_PERIPHERAL_CAN = 5
} gmp_router_peripheral_t;

/** Common system operations. */
typedef enum {
    GMP_ROUTER_SYSTEM_HELLO = 1,
    GMP_ROUTER_SYSTEM_CAPABILITIES = 2,
    GMP_ROUTER_SYSTEM_PING = 3
} gmp_router_system_operation_t;

/** GPIO operations. */
typedef enum {
    GMP_ROUTER_GPIO_CONFIGURE = 1,
    GMP_ROUTER_GPIO_WRITE = 2,
    GMP_ROUTER_GPIO_READ = 3
} gmp_router_gpio_operation_t;

/** Stream and bus operations shared by UART, I2C and SPI. */
typedef enum {
    GMP_ROUTER_BUS_CONFIGURE = 1,
    GMP_ROUTER_BUS_WRITE = 2,
    GMP_ROUTER_BUS_READ = 3,
    GMP_ROUTER_BUS_TRANSFER = 4
} gmp_router_bus_operation_t;

/** CAN operations and asynchronous event identifiers. */
typedef enum {
    GMP_ROUTER_CAN_GET_CAPABILITIES = 1,
    GMP_ROUTER_CAN_CONFIGURE = 2,
    GMP_ROUTER_CAN_START = 3,
    GMP_ROUTER_CAN_STOP = 4,
    GMP_ROUTER_CAN_TRANSMIT = 5,
    GMP_ROUTER_CAN_SET_FILTER = 6,
    GMP_ROUTER_CAN_GET_STATE = 7,
    GMP_ROUTER_CAN_RECOVER = 8,
    GMP_ROUTER_CAN_RX_EVENT = 0x81,
    GMP_ROUTER_CAN_TX_EVENT = 0x82,
    GMP_ROUTER_CAN_STATE_EVENT = 0x83
} gmp_router_can_operation_t;

/** Protocol-level status values. */
typedef enum {
    GMP_ROUTER_STATUS_OK = 0,
    GMP_ROUTER_STATUS_INVALID = -1,
    GMP_ROUTER_STATUS_UNSUPPORTED = -2,
    GMP_ROUTER_STATUS_BUSY = -3,
    GMP_ROUTER_STATUS_TIMEOUT = -4,
    GMP_ROUTER_STATUS_IO = -5,
    GMP_ROUTER_STATUS_NOT_FOUND = -6
} gmp_router_status_t;

/** Decoded protocol header. */
typedef struct {
    uint8_t message_type;
    uint32_t sequence;
    uint8_t peripheral;
    uint8_t operation;
    uint16_t endpoint;
    uint16_t channel;
    int16_t status;
    uint16_t payload_length;
} gmp_router_header_t;

/** Calculate the CRC-16/CCITT-FALSE checksum used by the protocol. */
uint16_t gmp_router_crc16(const uint8_t* data, size_t length);

/** Encode a header and payload into an unframed protocol packet. */
int gmp_router_packet_encode(const gmp_router_header_t* header, const uint8_t* payload,
                             uint8_t* output, size_t output_capacity, size_t* output_length);

/** Decode and validate an unframed protocol packet. */
int gmp_router_packet_decode(const uint8_t* packet, size_t packet_length,
                             gmp_router_header_t* header, const uint8_t** payload);

/** Encode bytes with Consistent Overhead Byte Stuffing. */
int gmp_router_cobs_encode(const uint8_t* input, size_t input_length, uint8_t* output,
                           size_t output_capacity, size_t* output_length);

/** Decode bytes encoded with Consistent Overhead Byte Stuffing. */
int gmp_router_cobs_decode(const uint8_t* input, size_t input_length, uint8_t* output,
                           size_t output_capacity, size_t* output_length);

#ifdef __cplusplus
}
#endif

#endif /* GMP_ROUTER_PROTOCOL_H */
