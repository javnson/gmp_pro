/**
 * @file can_if.h
 * @brief Transport-neutral CAN frame and decoded CANopen request contract.
 *
 * Hardware-specific frame decoding/encoding is deliberately left behind two
 * declared adapter functions. Protocol engines consume the stable frame and
 * request models defined here and do not depend on a vendor CAN driver.
 */

#ifndef _FILE_GMP_CANOPEN_CAN_IF_H_
#define _FILE_GMP_CANOPEN_CAN_IF_H_

#include <gmp_type.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef uint_least8_t gmp_canopen_octet_t;

#define GMP_CANOPEN_CLASSIC_MAX_DATA 8U
#define GMP_CANOPEN_NODE_ID_MIN      1U
#define GMP_CANOPEN_NODE_ID_MAX      127U
#define GMP_CANOPEN_COB_NMT          0x000U
#define GMP_CANOPEN_COB_SYNC         0x080U
#define GMP_CANOPEN_COB_TPDO1        0x180U
#define GMP_CANOPEN_COB_RPDO1        0x200U
#define GMP_CANOPEN_COB_TSDO         0x580U
#define GMP_CANOPEN_COB_RSDO         0x600U
#define GMP_CANOPEN_COB_HEARTBEAT    0x700U

typedef struct
{
    uint32_t id;
    gmp_canopen_octet_t data[GMP_CANOPEN_CLASSIC_MAX_DATA];
    gmp_canopen_octet_t dlc;
    fast_gt is_extended;
    fast_gt is_remote;
} gmp_canopen_frame_t;

typedef enum
{
    GMP_CANOPEN_REQUEST_UNKNOWN = 0,
    GMP_CANOPEN_REQUEST_NMT,
    GMP_CANOPEN_REQUEST_SYNC,
    GMP_CANOPEN_REQUEST_EMCY,
    GMP_CANOPEN_REQUEST_TPDO,
    GMP_CANOPEN_REQUEST_RPDO,
    GMP_CANOPEN_REQUEST_SDO_REQUEST,
    GMP_CANOPEN_REQUEST_SDO_RESPONSE,
    GMP_CANOPEN_REQUEST_HEARTBEAT
} gmp_canopen_request_kind_t;

typedef struct
{
    gmp_canopen_request_kind_t kind;
    uint16_t node_id;
    uint16_t function_number;
    uint16_t command;
    uint16_t index;
    uint16_t subindex;
    gmp_canopen_frame_t frame;
} gmp_canopen_request_t;

typedef fast_gt (*gmp_canopen_send_fn)(void* context,
                                       const gmp_canopen_frame_t* frame);

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

GMP_STATIC_INLINE uint16_t gmp_canopen_load_le16(
    const gmp_canopen_octet_t* data)
{
    return (uint16_t)((uint16_t)(data[0] & 0xFFU) |
                      ((uint16_t)(data[1] & 0xFFU) << 8U));
}

GMP_STATIC_INLINE uint32_t gmp_canopen_load_le32(
    const gmp_canopen_octet_t* data)
{
    return (uint32_t)(data[0] & 0xFFU) |
           ((uint32_t)(data[1] & 0xFFU) << 8U) |
           ((uint32_t)(data[2] & 0xFFU) << 16U) |
           ((uint32_t)(data[3] & 0xFFU) << 24U);
}

GMP_STATIC_INLINE void gmp_canopen_store_le16(gmp_canopen_octet_t* data,
                                               uint16_t value)
{
    data[0] = (gmp_canopen_octet_t)(value & 0xFFU);
    data[1] = (gmp_canopen_octet_t)((value >> 8U) & 0xFFU);
}

GMP_STATIC_INLINE void gmp_canopen_store_le32(gmp_canopen_octet_t* data,
                                               uint32_t value)
{
    data[0] = (gmp_canopen_octet_t)(value & 0xFFU);
    data[1] = (gmp_canopen_octet_t)((value >> 8U) & 0xFFU);
    data[2] = (gmp_canopen_octet_t)((value >> 16U) & 0xFFU);
    data[3] = (gmp_canopen_octet_t)((value >> 24U) & 0xFFU);
}

/**
 * @brief Convert one vendor-neutral CAN frame to a standard CANopen request.
 * @note Reserved transport adapter contract; implementation follows with the
 *       selected hardware CAN backend.
 */
fast_gt gmp_canopen_decode_frame(const gmp_canopen_frame_t* frame,
                                 gmp_canopen_request_t* request);

/**
 * @brief Convert a standard CANopen request to one vendor-neutral CAN frame.
 * @note Reserved transport adapter contract; implementation follows with the
 *       selected hardware CAN backend.
 */
fast_gt gmp_canopen_encode_request(const gmp_canopen_request_t* request,
                                   gmp_canopen_frame_t* frame);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CANOPEN_CAN_IF_H_ */
