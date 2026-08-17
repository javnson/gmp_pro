/**
 * @file can_if.h
 * @brief Transport-neutral classic-CAN frame and CANopen request contracts.
 *
 * @details CAN and CANopen define an octet as exactly eight bits. Some GMP
 * targets, notably TI C28x, have a 16-bit addressable C byte and do not define
 * `uint8_t`. GMP therefore stores one wire octet in one unsigned `byte_gt`.
 * It is eight bits on ordinary MCUs and one 16-bit addressable unit on C28x.
 */

#ifndef _FILE_GMP_CANOPEN_CAN_IF_H_
#define _FILE_GMP_CANOPEN_CAN_IF_H_

#include <gmp_type.h>
#include <limits.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Number of logical wire octets in a classic CAN data field. */
#define GMP_CANOPEN_CLASSIC_MAX_DATA 8U
/** @brief Lowest assignable CANopen node ID. */
#define GMP_CANOPEN_NODE_ID_MIN      1U
/** @brief Highest assignable CANopen node ID. */
#define GMP_CANOPEN_NODE_ID_MAX      127U
/** @brief NMT service base COB-ID. */
#define GMP_CANOPEN_COB_NMT          0x000U
/** @brief SYNC service base COB-ID. */
#define GMP_CANOPEN_COB_SYNC         0x080U
/** @brief First transmit-PDO base COB-ID. */
#define GMP_CANOPEN_COB_TPDO1        0x180U
/** @brief First receive-PDO base COB-ID. */
#define GMP_CANOPEN_COB_RPDO1        0x200U
/** @brief SDO server-to-client base COB-ID. */
#define GMP_CANOPEN_COB_TSDO         0x580U
/** @brief SDO client-to-server base COB-ID. */
#define GMP_CANOPEN_COB_RSDO         0x600U
/** @brief Heartbeat/boot-up base COB-ID. */
#define GMP_CANOPEN_COB_HEARTBEAT    0x700U

/**
 * @brief Vendor-neutral classic CAN frame.
 * @details Each `data` element represents one eight-bit wire octet in its low
 * eight bits. This is an unpacked logical representation, not a native byte
 * buffer, so its layout remains valid when `CHAR_BIT` is 16.
 */
typedef struct
{
    uint32_t id; /**< 11-bit standard or 29-bit extended identifier. */
    byte_gt data[GMP_CANOPEN_CLASSIC_MAX_DATA]; /**< Wire octets. */
    uint16_t dlc; /**< Number of valid logical octets, in the range 0..8. */
    fast_gt is_extended; /**< Non-zero for a 29-bit identifier. */
    fast_gt is_remote; /**< Non-zero for a remote-transmission request. */
} gmp_canopen_frame_t;

/** @brief Decoded CANopen service category. */
typedef enum
{
    GMP_CANOPEN_REQUEST_UNKNOWN = 0, /**< Unrecognized or unsupported frame. */
    GMP_CANOPEN_REQUEST_NMT, /**< NMT command. */
    GMP_CANOPEN_REQUEST_SYNC, /**< SYNC service. */
    GMP_CANOPEN_REQUEST_EMCY, /**< Emergency service. */
    GMP_CANOPEN_REQUEST_TPDO, /**< Transmit PDO. */
    GMP_CANOPEN_REQUEST_RPDO, /**< Receive PDO. */
    GMP_CANOPEN_REQUEST_SDO_REQUEST, /**< Client-to-server SDO. */
    GMP_CANOPEN_REQUEST_SDO_RESPONSE, /**< Server-to-client SDO. */
    GMP_CANOPEN_REQUEST_HEARTBEAT /**< Heartbeat or boot-up. */
} gmp_canopen_request_kind_t;

/** @brief Normalized CANopen request produced by a transport adapter. */
typedef struct
{
    gmp_canopen_request_kind_t kind; /**< Decoded service category. */
    uint16_t node_id; /**< Node ID, or zero for a broadcast service. */
    uint16_t function_number; /**< CANopen function-code field. */
    uint16_t command; /**< Service command specifier when applicable. */
    uint16_t index; /**< Object-dictionary index when applicable. */
    uint16_t subindex; /**< Object-dictionary sub-index when applicable. */
    gmp_canopen_frame_t frame; /**< Original normalized frame. */
} gmp_canopen_request_t;

/**
 * @brief Callback used by protocol engines to publish one normalized frame.
 * @param context User-owned transport context.
 * @param frame Frame valid only for the duration of the callback.
 * @return Non-zero when the transport accepted the frame.
 */
typedef fast_gt (*gmp_canopen_send_fn)(void* context,
                                       const gmp_canopen_frame_t* frame);

/**
 * @brief Reset a normalized frame to a standard, non-remote empty frame.
 * @param frame Frame to reset; `NULL` is accepted and ignored.
 */
GMP_STATIC_INLINE void gmp_canopen_frame_clear(gmp_canopen_frame_t* frame)
{
    uint16_t index;
    if (frame == NULL)
        return;
    frame->id = 0U;
    frame->dlc = 0U;
    frame->is_extended = 0;
    frame->is_remote = 0;
    for (index = 0U; index < GMP_CANOPEN_CLASSIC_MAX_DATA; ++index)
        frame->data[index] = 0U;
}

/**
 * @brief Load a little-endian 16-bit value from two logical wire octets.
 * @param data At least two `byte_gt` elements containing values in 0..255.
 * @return Reconstructed host value.
 */
GMP_STATIC_INLINE uint16_t gmp_canopen_load_le16(const byte_gt* data)
{
    return (uint16_t)((data[0] & 0xFFU) | ((data[1] & 0xFFU) << 8U));
}

/**
 * @brief Load a little-endian 32-bit value from four logical wire octets.
 * @param data At least four `byte_gt` elements containing values in 0..255.
 * @return Reconstructed host value.
 */
GMP_STATIC_INLINE uint32_t gmp_canopen_load_le32(const byte_gt* data)
{
    return (uint32_t)(data[0] & 0xFFU) |
           ((uint32_t)(data[1] & 0xFFU) << 8U) |
           ((uint32_t)(data[2] & 0xFFU) << 16U) |
           ((uint32_t)(data[3] & 0xFFU) << 24U);
}

/**
 * @brief Store a 16-bit value as two little-endian logical wire octets.
 * @param data Destination with room for two `byte_gt` elements.
 * @param value Host value to serialize.
 */
GMP_STATIC_INLINE void gmp_canopen_store_le16(byte_gt* data, uint16_t value)
{
    data[0] = value & 0xFFU;
    data[1] = (value >> 8U) & 0xFFU;
}

/**
 * @brief Store a 32-bit value as four little-endian logical wire octets.
 * @param data Destination with room for four `byte_gt` elements.
 * @param value Host value to serialize.
 */
GMP_STATIC_INLINE void gmp_canopen_store_le32(byte_gt* data, uint32_t value)
{
    data[0] = (byte_gt)(value & 0xFFU);
    data[1] = (byte_gt)((value >> 8U) & 0xFFU);
    data[2] = (byte_gt)((value >> 16U) & 0xFFU);
    data[3] = (byte_gt)((value >> 24U) & 0xFFU);
}

/**
 * @brief Verify that a normalized frame obeys classic-CAN logical-byte rules.
 * @param frame Frame to inspect.
 * @return Non-zero when DLC is at most eight and every valid data cell is at
 * most `0xFF`.
 */
GMP_STATIC_INLINE fast_gt gmp_canopen_frame_validate(
    const gmp_canopen_frame_t* frame)
{
    uint16_t index;
    if (frame == NULL || frame->dlc > GMP_CANOPEN_CLASSIC_MAX_DATA)
        return 0;
    for (index = 0U; index < frame->dlc; ++index)
        if (frame->data[index] > 0xFFU)
            return 0;
    return 1;
}

#if defined(UINT8_MAX) && (CHAR_BIT == 8)
/**
 * @brief Import a packed native 8-bit payload into logical `byte_gt` elements.
 * @param destination Destination logical cells.
 * @param source Packed native bytes.
 * @param count Number of wire octets to copy.
 * @return Non-zero on success; zero for a null pointer.
 * @note This API does not exist on C28x because that ABI has no `uint8_t`.
 */
GMP_STATIC_INLINE fast_gt gmp_canopen_import_u8(byte_gt* destination,
                                                const uint8_t* source,
                                                uint16_t count)
{
    uint16_t index;
    if (destination == NULL || source == NULL)
        return 0;
    for (index = 0U; index < count; ++index)
        destination[index] = source[index];
    return 1;
}

/**
 * @brief Export `byte_gt` wire octets to a packed native 8-bit payload.
 * @param destination Packed native-byte destination.
 * @param source Logical cells; each value must be at most `0xFF`.
 * @param count Number of wire octets to copy.
 * @return Non-zero on success, zero for invalid input or an out-of-range cell.
 * @note This API does not exist on C28x because that ABI has no `uint8_t`.
 */
GMP_STATIC_INLINE fast_gt gmp_canopen_export_u8(uint8_t* destination,
                                                const byte_gt* source,
                                                uint16_t count)
{
    uint16_t index;
    if (destination == NULL || source == NULL)
        return 0;
    for (index = 0U; index < count; ++index)
    {
        if (source[index] > 0xFFU)
            return 0;
        destination[index] = (uint8_t)source[index];
    }
    return 1;
}
#endif

/**
 * @brief Decode a vendor-neutral CAN frame into a normalized CANopen request.
 * @param frame Input frame.
 * @param request Output request.
 * @return Non-zero when the adapter recognizes and decodes the frame.
 * @note Reserved adapter contract; a selected CAN backend provides the body.
 */
fast_gt gmp_canopen_decode_frame(const gmp_canopen_frame_t* frame,
                                 gmp_canopen_request_t* request);

/**
 * @brief Encode a normalized CANopen request as a vendor-neutral CAN frame.
 * @param request Input request.
 * @param frame Output frame.
 * @return Non-zero when the request can be encoded.
 * @note Reserved adapter contract; a selected CAN backend provides the body.
 */
fast_gt gmp_canopen_encode_request(const gmp_canopen_request_t* request,
                                   gmp_canopen_frame_t* frame);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CANOPEN_CAN_IF_H_ */
