/**
 * @file canopen.h
 * @brief Complete allocation-free CANopen node and transport queue facade.
 *
 * @details A `gmp_canopen_t` owns the NMT state machine, object dictionary,
 * SDO server, compiled RX/TX PDO groups, normalized CoE server, CiA 301 base
 * objects, and two single-producer/single-consumer queues. Transport interrupt
 * code only calls `gmp_canopen_input_callback()` and
 * `gmp_canopen_output_callback()`. The application main loop calls
 * `gmp_canopen_background_callback()` to parse requests and prepare replies.
 * No protocol parser or object-dictionary operation executes in the ISR.
 */

#ifndef _FILE_GMP_CANOPEN_H_
#define _FILE_GMP_CANOPEN_H_

#include <core/protocol/canopen/ethercat_if.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Maximum payload copied into one queued CANopen/CoE packet. */
#ifndef GMP_CANOPEN_PACKET_MAX_DATA
#define GMP_CANOPEN_PACKET_MAX_DATA GMP_CANOPEN_SDO_MAX_TRANSFER
#endif

/**
 * @brief Number of physical slots in each SPSC queue.
 * @details One slot is reserved to distinguish full from empty, so the usable
 * capacity is `GMP_CANOPEN_QUEUE_SLOTS - 1`.
 */
#ifndef GMP_CANOPEN_QUEUE_SLOTS
#define GMP_CANOPEN_QUEUE_SLOTS 4U
#endif

/** @brief Maximum input packets parsed by one background callback. */
#ifndef GMP_CANOPEN_BACKGROUND_BUDGET
#define GMP_CANOPEN_BACKGROUND_BUDGET (GMP_CANOPEN_QUEUE_SLOTS - 1U)
#endif

/** @brief Number of CiA 301 entries owned by the node facade. */
#define GMP_CANOPEN_CIA301_ENTRY_CAPACITY 11U

/** @cond CANOPEN_COMPILE_TIME_CONTRACTS */
GMP_STATIC_ASSERT(GMP_CANOPEN_QUEUE_SLOTS >= 2U,
                  "CANopen queues require at least two slots");
GMP_STATIC_ASSERT(GMP_CANOPEN_QUEUE_SLOTS <= UINT16_MAX,
                  "CANopen queue indexes are uint16_t");
GMP_STATIC_ASSERT(GMP_CANOPEN_PACKET_MAX_DATA >= GMP_CANOPEN_CLASSIC_MAX_DATA,
                  "CANopen packet storage must hold one classic CAN frame");
GMP_STATIC_ASSERT(GMP_CANOPEN_PACKET_MAX_DATA <= UINT16_MAX,
                  "CANopen PDO payload capacity is uint16_t");
/** @endcond */

/** @brief Transport that produced or will consume a public packet. */
typedef enum
{
    GMP_CANOPEN_TRANSPORT_CAN = 0, /**< Classic CAN frame transport. */
    GMP_CANOPEN_TRANSPORT_COE = 1 /**< CAN application protocol over EtherCAT. */
} gmp_canopen_transport_t;

/** @brief Semantic kind of a queued public packet. */
typedef enum
{
    GMP_CANOPEN_PACKET_NONE = 0, /**< Cleared or invalid packet. */
    GMP_CANOPEN_PACKET_CAN_FRAME, /**< Encoded classic CAN frame. */
    GMP_CANOPEN_PACKET_COE_SDO_UPLOAD, /**< Normalized CoE SDO upload request. */
    GMP_CANOPEN_PACKET_COE_SDO_DOWNLOAD, /**< Normalized CoE SDO download request. */
    GMP_CANOPEN_PACKET_COE_SDO_RESPONSE, /**< Normalized CoE SDO response. */
    GMP_CANOPEN_PACKET_COE_RXPDO, /**< EtherCAT process data applied to an RX-PDO. */
    GMP_CANOPEN_PACKET_COE_TXPDO_REQUEST, /**< Request to build a CoE TX-PDO. */
    GMP_CANOPEN_PACKET_COE_TXPDO /**< Built CoE TX-PDO process data. */
} gmp_canopen_packet_kind_t;

/** @brief Public-packet flag: classic CAN extended identifier. */
#define GMP_CANOPEN_PACKET_FLAG_EXTENDED       (1U << 0)
/** @brief Public-packet flag: classic CAN remote request. */
#define GMP_CANOPEN_PACKET_FLAG_REMOTE         (1U << 1)
/** @brief Public-packet flag: CoE Complete Access request. */
#define GMP_CANOPEN_PACKET_FLAG_COMPLETE_ACCESS (1U << 2)

/**
 * @brief Transport-neutral owned packet exchanged with a CAN or CoE adapter.
 *
 * @details `key` is a CAN identifier for `CAN_FRAME` and a process-image key
 * for PDO packets. SDO packets use `number`, `index`, and `subindex`.
 * `data_size` and `data` always count/store CANopen wire octets. The queue
 * copies valid payload elements, so the producer may release its source after
 * the input callback returns.
 */
typedef struct
{
    gmp_canopen_transport_t transport; /**< Source/destination transport. */
    gmp_canopen_packet_kind_t kind; /**< Packet semantic kind. */
    uint32_t key; /**< CAN identifier or CoE process-image key. */
    uint32_t abort_code; /**< SDO abort code in a CoE response. */
    uint32_t data_size; /**< Number of valid payload octets. */
    uint16_t number; /**< CoE request/response number. */
    uint16_t index; /**< CoE SDO OD index. */
    byte_gt subindex; /**< CoE SDO OD sub-index. */
    byte_gt flags; /**< Bitwise `GMP_CANOPEN_PACKET_FLAG_*`. */
    gmp_coe_result_t coe_result; /**< Result carried by a CoE SDO response. */
    byte_gt data[GMP_CANOPEN_PACKET_MAX_DATA]; /**< Queue-owned wire payload. */
} gmp_canopen_packet_t;

/** @brief CANopen facade operation result. */
typedef enum
{
    GMP_CANOPEN_NODE_OK = 0, /**< Operation completed. */
    GMP_CANOPEN_NODE_INVALID, /**< Invalid pointer, packet, or configuration. */
    GMP_CANOPEN_NODE_RX_FULL, /**< Input queue has no free slot. */
    GMP_CANOPEN_NODE_TX_FULL, /**< Output queue has no free slot. */
    GMP_CANOPEN_NODE_EMPTY, /**< Output queue contains no prepared packet. */
    GMP_CANOPEN_NODE_NOT_FOUND, /**< Requested PDO is not registered. */
    GMP_CANOPEN_NODE_PROTOCOL_ERROR /**< Selected engine rejected the request. */
} gmp_canopen_node_result_t;

/** @brief CiA 301 identity record (object 0x1018). */
typedef struct
{
    uint32_t vendor_id; /**< 0x1018:01 vendor identifier. */
    uint32_t product_code; /**< 0x1018:02 product code. */
    uint32_t revision_number; /**< 0x1018:03 revision number. */
    uint32_t serial_number; /**< 0x1018:04 serial number. */
} gmp_canopen_cia301_identity_t;

/**
 * @brief Base CiA 301 information exposed through pointer-backed OD entries.
 *
 * @details Scalar fields are copied into the node and remain directly visible
 * to the debugger. The node's standard OD entries point at these fields.
 * Optional string pointers are not copied and must remain valid for the node
 * lifetime.
 */
typedef struct
{
    uint32_t device_type; /**< 0x1000:00 device type. */
    byte_gt error_register; /**< 0x1001:00 error register. */
    uint16_t producer_heartbeat_time_ms; /**< 0x1017:00 heartbeat period. */
    gmp_canopen_cia301_identity_t identity; /**< 0x1018 identity record. */
    const byte_gt* device_name; /**< Optional 0x1008:00 device name. */
    uint32_t device_name_size; /**< Device-name length in wire octets. */
    const byte_gt* hardware_version; /**< Optional 0x1009:00 version. */
    uint32_t hardware_version_size; /**< Hardware-version length. */
    const byte_gt* software_version; /**< Optional 0x100A:00 version. */
    uint32_t software_version_size; /**< Software-version length. */
} gmp_canopen_cia301_info_t;

/** @brief Initialization contract for one complete node. */
typedef struct
{
    byte_gt node_id; /**< CANopen node ID in 1..127. */
    gmp_canopen_cia301_info_t cia301; /**< Initial standard information. */
} gmp_canopen_config_t;

/**
 * @brief One fixed SPSC packet queue for a single-core ISR/main-loop pair.
 * @details `volatile` indexes prevent access elision but are not a multicore
 * memory barrier. Multicore or cache-incoherent adapters must serialize these
 * callbacks or add platform-appropriate fences outside the portable core.
 */
typedef struct
{
    volatile uint16_t write_index; /**< Producer-owned next-write slot. */
    volatile uint16_t read_index; /**< Consumer-owned next-read slot. */
    gmp_canopen_packet_t packets[GMP_CANOPEN_QUEUE_SLOTS]; /**< Owned slots. */
} gmp_canopen_packet_queue_t;

/** @brief Observable node and queue counters. */
typedef struct
{
    volatile uint32_t input_packets; /**< Packets accepted from transport. */
    volatile uint32_t input_dropped; /**< Packets rejected/full at input. */
    volatile uint32_t processed_packets; /**< Packets parsed in background. */
    volatile uint32_t output_packets; /**< Replies placed in the TX queue. */
    volatile uint32_t output_dropped; /**< Replies lost because TX was full. */
    volatile uint32_t output_consumed; /**< Replies consumed by transport. */
    volatile uint32_t protocol_errors; /**< Engine/packet dispatch failures. */
} gmp_canopen_statistics_t;

/**
 * @brief Complete allocation-free CANopen node.
 * @details Applications normally declare exactly one object of this type per
 * CANopen node and initialize it with `gmp_canopen_init()`.
 */
typedef struct
{
    gmp_canopen_od_t dictionary; /**< Shared CANopen/CoE object dictionary. */
    gmp_canopen_nmt_t nmt; /**< NMT and heartbeat producer. */
    gmp_canopen_sdo_server_t sdo; /**< Default SDO server. */
    gmp_coe_server_t coe; /**< CoE access to the same dictionary. */
    gmp_canopen_txpdo_group_t txpdo_group; /**< Compiled TX-PDOs. */
    gmp_canopen_rxpdo_group_t rxpdo_group; /**< Compiled RX-PDOs. */
    gmp_canopen_cia301_info_t cia301; /**< Pointer-backed standard values. */
    byte_gt identity_subindex_count; /**< Value of 0x1018:00. */
    uint16_t cia301_entry_count; /**< Registered standard-entry count. */
    gmp_canopen_od_entry_t cia301_entries[GMP_CANOPEN_CIA301_ENTRY_CAPACITY]; /**< Owned OD leaves. */
    gmp_canopen_packet_queue_t input_queue; /**< ISR/EtherCAT to background. */
    gmp_canopen_packet_queue_t output_queue; /**< Background to ISR/DMA. */
    gmp_canopen_statistics_t statistics; /**< Debugger-visible counters. */
    fast_gt initialized; /**< Non-zero after complete initialization. */
} gmp_canopen_t;

/**
 * @brief Fill a node configuration with safe CiA 301 defaults.
 * @param config Configuration to clear.
 * @param node_id Node ID in 1..127.
 * @return Non-zero for valid arguments.
 */
fast_gt gmp_canopen_config_init(gmp_canopen_config_t* config,
                                byte_gt node_id);

/**
 * @brief Initialize one complete CANopen node and its standard OD entries.
 * @param node Node object to initialize.
 * @param config Configuration copied during this call.
 * @return Node result.
 */
gmp_canopen_node_result_t gmp_canopen_init(
    gmp_canopen_t* node, const gmp_canopen_config_t* config);

/**
 * @brief Initialize a node with zero-valued CiA 301 information.
 * @param node Node object to initialize.
 * @param node_id Node ID in 1..127.
 * @return Node result.
 */
gmp_canopen_node_result_t gmp_canopen_init_default(
    gmp_canopen_t* node, byte_gt node_id);

/**
 * @brief Clear a public packet without changing queue state.
 * @param packet Packet to clear; `NULL` is ignored.
 */
void gmp_canopen_packet_clear(gmp_canopen_packet_t* packet);

/**
 * @brief Convert a normalized classic-CAN frame to the public packet contract.
 * @param frame Source frame.
 * @param packet Destination packet.
 * @return Non-zero when the frame is valid and copied.
 */
fast_gt gmp_canopen_packet_from_can(
    const gmp_canopen_frame_t* frame, gmp_canopen_packet_t* packet);

/**
 * @brief Convert a public CAN packet back to a normalized frame.
 * @param packet Source `CAN_FRAME` packet.
 * @param frame Destination frame.
 * @return Non-zero when the packet is valid and copied.
 */
fast_gt gmp_canopen_packet_to_can(
    const gmp_canopen_packet_t* packet, gmp_canopen_frame_t* frame);

/**
 * @brief Interrupt/EtherCAT callback that copies one input into the RX queue.
 * @param node Initialized node.
 * @param packet CAN or CoE public input packet.
 * @return Queue/validation result.
 * @note This is the sole producer of `input_queue`.
 */
gmp_canopen_node_result_t gmp_canopen_input_callback(
    gmp_canopen_t* node, const gmp_canopen_packet_t* packet);

/**
 * @brief Main-loop callback that parses inputs, advances heartbeat, and replies.
 * @param node Initialized node.
 * @param elapsed_ms Time since the previous background call.
 * @return Number of input packets removed from the RX queue.
 * @note This is the RX consumer and TX producer. It processes at most
 * `GMP_CANOPEN_BACKGROUND_BUDGET` input packets per call.
 */
uint16_t gmp_canopen_background_callback(gmp_canopen_t* node,
                                         uint32_t elapsed_ms);

/**
 * @brief CAN-idle/DMA callback that removes one prepared output packet.
 * @param node Initialized node.
 * @param packet Destination owned by the transport adapter.
 * @return `GMP_CANOPEN_NODE_OK` or `GMP_CANOPEN_NODE_EMPTY`.
 * @note This is the sole consumer of `output_queue`.
 */
gmp_canopen_node_result_t gmp_canopen_output_callback(
    gmp_canopen_t* node, gmp_canopen_packet_t* packet);

/**
 * @brief Register one already-compiled TX-PDO with the complete node.
 * @param node Initialized node.
 * @param pdo Compiled PDO with stable lifetime.
 * @return PDO group result.
 */
gmp_canopen_pdo_result_t gmp_canopen_add_txpdo(
    gmp_canopen_t* node, gmp_canopen_txpdo_t* pdo);

/**
 * @brief Register one already-compiled RX-PDO with the complete node.
 * @param node Initialized node.
 * @param pdo Compiled PDO with stable lifetime.
 * @return PDO group result.
 */
gmp_canopen_pdo_result_t gmp_canopen_add_rxpdo(
    gmp_canopen_t* node, gmp_canopen_rxpdo_t* pdo);

/**
 * @brief Build and enqueue one registered TX-PDO for CAN or CoE.
 * @param node Initialized node.
 * @param transport Output transport.
 * @param key CAN COB-ID or CoE process-image key.
 * @return Node result.
 * @note Call from the background/TX-producer context. Calling this from a
 * second producer violates the output queue's SPSC contract.
 */
gmp_canopen_node_result_t gmp_canopen_publish_tpdo(
    gmp_canopen_t* node, gmp_canopen_transport_t transport, uint32_t key);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CANOPEN_H_ */
