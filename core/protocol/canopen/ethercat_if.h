/**
 * @file ethercat_if.h
 * @brief Transport-neutral CAN application protocol over EtherCAT (CoE) API.
 *
 * @details This layer reuses the CANopen object dictionary and precompiled PDO
 * plans without pretending that an EtherCAT mailbox is a CAN frame. An
 * EtherCAT MainDevice/SubDevice stack owns raw mailbox headers, counters,
 * fragmentation and AL-state gating, then supplies the normalized requests
 * defined here. Payloads use one unsigned `byte_gt` per wire octet.
 */

#ifndef _FILE_GMP_CANOPEN_ETHERCAT_IF_H_
#define _FILE_GMP_CANOPEN_ETHERCAT_IF_H_

#include <core/protocol/canopen/pdo_engine.h>
#include <core/protocol/canopen/sdo_engine.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief CoE service categories carried by an EtherCAT mailbox. */
typedef enum
{
    GMP_COE_SERVICE_EMERGENCY = 1, /**< Emergency request. */
    GMP_COE_SERVICE_SDO_REQUEST = 2, /**< SDO request. */
    GMP_COE_SERVICE_SDO_RESPONSE = 3, /**< SDO response. */
    GMP_COE_SERVICE_TXPDO = 4, /**< Transmit PDO service. */
    GMP_COE_SERVICE_RXPDO = 5, /**< Receive PDO service. */
    GMP_COE_SERVICE_TXPDO_REMOTE = 6, /**< Remote TX-PDO request. */
    GMP_COE_SERVICE_RXPDO_REMOTE = 7, /**< Remote RX-PDO request. */
    GMP_COE_SERVICE_SDO_INFORMATION = 8 /**< SDO information service. */
} gmp_coe_service_t;

/** @brief Normalized CoE mailbox envelope supplied by an EtherCAT adapter. */
typedef struct
{
    gmp_coe_service_t service; /**< Decoded CoE service. */
    uint16_t number; /**< Adapter-preserved CoE service number/counter. */
    const byte_gt* payload; /**< Wire-octet payload. */
    uint32_t payload_size; /**< Payload length in logical octets. */
} gmp_coe_mailbox_t;

/** @brief Normalized CoE SDO operation. */
typedef enum
{
    GMP_COE_SDO_UPLOAD = 0, /**< Read from the object dictionary. */
    GMP_COE_SDO_DOWNLOAD = 1 /**< Write to the object dictionary. */
} gmp_coe_sdo_operation_t;

/** @brief Result of a normalized CoE service. */
typedef enum
{
    GMP_COE_OK = 0, /**< Service completed. */
    GMP_COE_INVALID, /**< Malformed arguments or logical octets. */
    GMP_COE_UNSUPPORTED, /**< Unsupported Complete Access/service feature. */
    GMP_COE_OD_ABORT, /**< OD access failed; inspect `abort_code`. */
    GMP_COE_BUFFER_TOO_SMALL /**< Upload response buffer is too small. */
} gmp_coe_result_t;

/** @brief Normalized CoE SDO request after raw mailbox decoding. */
typedef struct
{
    uint16_t number; /**< CoE request number copied to the response. */
    gmp_coe_sdo_operation_t operation; /**< Upload or download. */
    uint16_t index; /**< OD index. */
    uint16_t subindex; /**< OD sub-index. */
    fast_gt complete_access; /**< Non-zero requests Complete Access. */
    const byte_gt* data; /**< Download wire octets; `NULL` for upload. */
    uint32_t data_size; /**< Download length in logical octets. */
} gmp_coe_sdo_request_t;

/** @brief Normalized CoE SDO response ready for mailbox encoding. */
typedef struct
{
    uint16_t number; /**< Request number. */
    gmp_coe_result_t result; /**< Service result. */
    uint32_t abort_code; /**< CANopen SDO abort code for `GMP_COE_OD_ABORT`. */
    byte_gt* data; /**< Caller-owned upload response buffer. */
    uint32_t capacity; /**< Response-buffer capacity in logical octets. */
    uint32_t data_size; /**< Produced upload length. */
} gmp_coe_sdo_response_t;

/** @brief CoE server bound to one shared CANopen object dictionary. */
typedef struct
{
    gmp_canopen_od_t* dictionary; /**< Dictionary reused by CANopen and CoE. */
} gmp_coe_server_t;

/**
 * @brief Bind a CoE server to an object dictionary.
 * @param server Server to initialize.
 * @param dictionary Shared dictionary.
 * @return Non-zero on success.
 */
fast_gt gmp_coe_server_init(gmp_coe_server_t* server,
                            gmp_canopen_od_t* dictionary);

/**
 * @brief Execute one normalized CoE SDO upload or download.
 * @param server Initialized server.
 * @param request Decoded request.
 * @param response Response with `data` and `capacity` configured by the caller.
 * @return Same result stored in `response->result`.
 * @note Complete Access and SDO Information are explicit future extensions.
 */
gmp_coe_result_t gmp_coe_sdo_server_process(
    gmp_coe_server_t* server, const gmp_coe_sdo_request_t* request,
    gmp_coe_sdo_response_t* response);

/**
 * @brief Compile a CoE TX-PDO with an adapter-selected process-image limit.
 * @param pdo TX plan to compile.
 * @param dictionary Shared OD.
 * @param process_image_key Adapter-defined process-image key.
 * @param descriptors Mapping descriptors.
 * @param descriptor_count Descriptor count.
 * @param payload_limit Maximum process-data length.
 * @return PDO compilation result.
 * @details CoE may relax the legacy CANopen eight-octet PDO limit.
 */
gmp_canopen_pdo_result_t gmp_coe_txpdo_compile(
    gmp_canopen_txpdo_t* pdo, gmp_canopen_od_t* dictionary,
    uint32_t process_image_key, const uint32_t* descriptors,
    uint16_t descriptor_count, uint16_t payload_limit);

/**
 * @brief Compile a CoE RX-PDO with an adapter-selected process-image limit.
 * @param pdo RX plan to compile.
 * @param dictionary Shared OD.
 * @param process_image_key Adapter-defined process-image key.
 * @param descriptors Mapping descriptors.
 * @param descriptor_count Descriptor count.
 * @param payload_limit Maximum process-data length.
 * @return PDO compilation result.
 */
gmp_canopen_pdo_result_t gmp_coe_rxpdo_compile(
    gmp_canopen_rxpdo_t* pdo, gmp_canopen_od_t* dictionary,
    uint32_t process_image_key, const uint32_t* descriptors,
    uint16_t descriptor_count, uint16_t payload_limit);

/**
 * @brief Pack a precompiled CoE TX-PDO after the adapter has gated AL state.
 * @param pdo Compiled TX plan.
 * @param output Destination logical cells.
 * @param capacity Destination capacity.
 * @param actual_size Produced process-data length.
 * @return PDO result; no OD lookup occurs.
 */
gmp_canopen_pdo_result_t gmp_coe_txpdo_pack_fast(
    const gmp_canopen_txpdo_t* pdo, byte_gt* output,
    uint16_t capacity, uint16_t* actual_size);

/**
 * @brief Apply a precompiled CoE RX-PDO after the adapter has gated AL state.
 * @param pdo Compiled RX plan.
 * @param input Source logical cells.
 * @param size Exact process-data length.
 * @return PDO result; no OD lookup occurs.
 */
gmp_canopen_pdo_result_t gmp_coe_rxpdo_unpack_fast(
    const gmp_canopen_rxpdo_t* pdo, const byte_gt* input, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CANOPEN_ETHERCAT_IF_H_ */
