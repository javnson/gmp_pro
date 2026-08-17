/**
 * @file sdo_engine.h
 * @brief Classic-CAN expedited and segmented SDO server.
 * @details Transfer storage uses one unsigned `byte_gt` per wire octet.
 */

#ifndef _FILE_GMP_CANOPEN_SDO_ENGINE_H_
#define _FILE_GMP_CANOPEN_SDO_ENGINE_H_

#include <core/protocol/canopen/od.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Maximum staged segmented transfer in logical octets. */
#ifndef GMP_CANOPEN_SDO_MAX_TRANSFER
#define GMP_CANOPEN_SDO_MAX_TRANSFER 128U
#endif

/** @brief Toggle bit was not alternated. */
#define GMP_CANOPEN_SDO_ABORT_TOGGLE        0x05030000UL
/** @brief Command specifier is invalid or out of sequence. */
#define GMP_CANOPEN_SDO_ABORT_COMMAND       0x05040001UL
/** @brief Transfer exceeds configured staging capacity. */
#define GMP_CANOPEN_SDO_ABORT_OUT_OF_MEMORY 0x05040005UL
/** @brief Requested access is unsupported. */
#define GMP_CANOPEN_SDO_ABORT_UNSUPPORTED   0x06010000UL
/** @brief Attempted to write a read-only object. */
#define GMP_CANOPEN_SDO_ABORT_READ_ONLY     0x06010002UL
/** @brief Object index/sub-index does not exist. */
#define GMP_CANOPEN_SDO_ABORT_NOT_FOUND     0x06020000UL
/** @brief Transfer length or data type does not match. */
#define GMP_CANOPEN_SDO_ABORT_TYPE_LENGTH   0x06070010UL
/** @brief Unclassified server error. */
#define GMP_CANOPEN_SDO_ABORT_GENERAL       0x08000000UL

/** @brief Active segmented-transfer state. */
typedef enum
{
    GMP_CANOPEN_SDO_IDLE = 0, /**< No segmented transaction. */
    GMP_CANOPEN_SDO_SEGMENTED_DOWNLOAD, /**< Receiving client segments. */
    GMP_CANOPEN_SDO_SEGMENTED_UPLOAD /**< Sending server segments. */
} gmp_canopen_sdo_state_t;

/** @brief Stateful SDO server for one node/default SDO channel. */
typedef struct
{
    uint16_t node_id; /**< Node ID used to derive 0x600/0x580 COB-IDs. */
    gmp_canopen_od_t* dictionary; /**< Shared object dictionary. */
    gmp_canopen_sdo_state_t state; /**< Active transfer state. */
    gmp_canopen_od_entry_t* entry; /**< Entry locked by a segmented transfer. */
    uint32_t offset; /**< Completed logical octets. */
    uint32_t total_size; /**< Expected transaction size. */
    uint16_t toggle; /**< Expected segment toggle bit. */
    byte_gt transfer[GMP_CANOPEN_SDO_MAX_TRANSFER]; /**< Staging octets. */
} gmp_canopen_sdo_server_t;

/**
 * @brief Initialize an SDO server for one node and dictionary.
 * @param server Server context.
 * @param node_id Node ID in 1..127.
 * @param dictionary Shared object dictionary.
 * @return Non-zero for valid arguments.
 */
fast_gt gmp_canopen_sdo_server_init(gmp_canopen_sdo_server_t* server,
                                     uint16_t node_id,
                                     gmp_canopen_od_t* dictionary);
/**
 * @brief Abort local transaction state without transmitting a frame.
 * @param server Server context; `NULL` is ignored.
 */
void gmp_canopen_sdo_server_reset(gmp_canopen_sdo_server_t* server);
/**
 * @brief Process one validated client-to-server frame.
 * @param server Initialized server.
 * @param request Candidate request frame.
 * @param response Output frame when the return value is non-zero.
 * @return Non-zero when `response` must be transmitted. A client abort resets
 * local state and returns zero.
 */
fast_gt gmp_canopen_sdo_server_process(gmp_canopen_sdo_server_t* server,
                                       const gmp_canopen_frame_t* request,
                                       gmp_canopen_frame_t* response);
/**
 * @brief Process one request and immediately dispatch the generated response.
 * @param server Initialized server.
 * @param request Candidate request frame.
 * @param send Transport callback.
 * @param send_context Callback context.
 * @return Non-zero only when a response was generated and accepted by `send`.
 */
fast_gt gmp_canopen_sdo_server_receive(gmp_canopen_sdo_server_t* server,
                                       const gmp_canopen_frame_t* request,
                                       gmp_canopen_send_fn send,
                                       void* send_context);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CANOPEN_SDO_ENGINE_H_ */
