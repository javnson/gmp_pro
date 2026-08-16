/**
 * @file pdo_engine.h
 * @brief Precompiled CANopen RX-PDO/TX-PDO fast paths and PDO groups.
 *
 * @details Compilation performs all object-dictionary lookup, access checks,
 * width checks and storage-address resolution. The online functions walk only
 * a compact transfer plan; they never search the dictionary or call the
 * general OD serializer. Separate RX and TX types prevent direction mistakes.
 */

#ifndef _FILE_GMP_CANOPEN_PDO_ENGINE_H_
#define _FILE_GMP_CANOPEN_PDO_ENGINE_H_

#include <core/protocol/canopen/nmt_sm.h>
#include <core/protocol/canopen/od.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Maximum mapped OD entries in one precompiled PDO. */
#ifndef GMP_CANOPEN_PDO_MAX_MAPPINGS
#define GMP_CANOPEN_PDO_MAX_MAPPINGS 16U
#endif

/** @brief Maximum PDO objects in one RX or TX group. */
#ifndef GMP_CANOPEN_PDO_GROUP_CAPACITY
#define GMP_CANOPEN_PDO_GROUP_CAPACITY 16U
#endif

/**
 * @brief Build one standard 32-bit PDO mapping descriptor.
 * @param index OD index.
 * @param subindex OD sub-index.
 * @param bits Mapped bit width; the current engine requires a multiple of 8.
 */
#define GMP_CANOPEN_PDO_MAP(index, subindex, bits) \
    (((uint32_t)(index) << 16U) | ((uint32_t)(subindex) << 8U) | (uint32_t)(bits))

/** @brief PDO compilation or online-transfer result. */
typedef enum
{
    GMP_CANOPEN_PDO_OK = 0, /**< Operation completed. */
    GMP_CANOPEN_PDO_INVALID, /**< Null, malformed, disabled, or wrong frame. */
    GMP_CANOPEN_PDO_NOT_MAPPABLE, /**< OD entry or access contract is unsuitable. */
    GMP_CANOPEN_PDO_TOO_LARGE, /**< Mapping exceeds configured payload capacity. */
    GMP_CANOPEN_PDO_NOT_OPERATIONAL, /**< CANopen node is not Operational. */
    GMP_CANOPEN_PDO_IO_ERROR, /**< Fast codec rejected an unexpected type/value. */
    GMP_CANOPEN_PDO_DUPLICATE, /**< Group already contains this PDO key. */
    GMP_CANOPEN_PDO_GROUP_FULL, /**< Group capacity is exhausted. */
    GMP_CANOPEN_PDO_NOT_FOUND /**< Requested PDO key is not registered. */
} gmp_canopen_pdo_result_t;

/** @brief One pre-resolved scalar/raw transfer operation. */
typedef struct
{
    void* storage; /**< Stable scalar address or logical-octet buffer. */
    gmp_canopen_od_data_type_t data_type; /**< Specialized online codec. */
    uint16_t byte_offset; /**< Logical-octet offset in the PDO payload. */
    uint16_t byte_length; /**< Logical wire width. */
} gmp_canopen_pdo_mapping_t;

/** @brief Common immutable-after-compile transfer plan. */
typedef struct
{
    uint32_t key; /**< Classic COB-ID or EtherCAT process-image key. */
    uint16_t transmission_type; /**< CiA transmission type metadata. */
    uint16_t mapping_count; /**< Number of valid mappings. */
    uint16_t payload_size; /**< Required logical-octet payload length. */
    fast_gt enabled; /**< Non-zero after successful compilation. */
    gmp_canopen_pdo_mapping_t mappings[GMP_CANOPEN_PDO_MAX_MAPPINGS]; /**< Transfer operations. */
} gmp_canopen_pdo_plan_t;

/** @brief Precompiled transmit PDO. */
typedef struct
{
    gmp_canopen_pdo_plan_t plan; /**< Fast source-to-wire transfer plan. */
} gmp_canopen_txpdo_t;

/** @brief Precompiled receive PDO. */
typedef struct
{
    gmp_canopen_pdo_plan_t plan; /**< Fast wire-to-destination transfer plan. */
} gmp_canopen_rxpdo_t;

/** @brief Sorted collection of precompiled transmit PDOs. */
typedef struct
{
    uint16_t count; /**< Number of registered pointers. */
    gmp_canopen_txpdo_t* items[GMP_CANOPEN_PDO_GROUP_CAPACITY]; /**< Sorted PDO pointers. */
} gmp_canopen_txpdo_group_t;

/** @brief Sorted collection of precompiled receive PDOs. */
typedef struct
{
    uint16_t count; /**< Number of registered pointers. */
    gmp_canopen_rxpdo_t* items[GMP_CANOPEN_PDO_GROUP_CAPACITY]; /**< Sorted PDO pointers. */
} gmp_canopen_rxpdo_group_t;

/**
 * @brief Compile a classic-CAN transmit PDO with an eight-octet limit.
 * @param pdo TX plan to compile.
 * @param dictionary Dictionary used only during compilation.
 * @param cob_id Standard 11-bit COB-ID.
 * @param transmission_type CiA transmission type metadata.
 * @param descriptors Mapping descriptors.
 * @param descriptor_count Descriptor count.
 * @return Compilation result. No partially compiled plan is enabled.
 */
gmp_canopen_pdo_result_t gmp_canopen_txpdo_compile(
    gmp_canopen_txpdo_t* pdo, gmp_canopen_od_t* dictionary,
    uint32_t cob_id, uint16_t transmission_type,
    const uint32_t* descriptors, uint16_t descriptor_count);

/**
 * @brief Compile a classic-CAN receive PDO with an eight-octet limit.
 * @param pdo RX plan to compile.
 * @param dictionary Dictionary used only during compilation.
 * @param cob_id Standard 11-bit COB-ID.
 * @param transmission_type CiA transmission type metadata.
 * @param descriptors Mapping descriptors.
 * @param descriptor_count Descriptor count.
 * @return Compilation result.
 */
gmp_canopen_pdo_result_t gmp_canopen_rxpdo_compile(
    gmp_canopen_rxpdo_t* pdo, gmp_canopen_od_t* dictionary,
    uint32_t cob_id, uint16_t transmission_type,
    const uint32_t* descriptors, uint16_t descriptor_count);

/**
 * @brief Compile a TX plan for a larger logical process-data buffer.
 * @param key Adapter-defined process-image key; it need not be a CAN COB-ID.
 * @param pdo TX plan to compile.
 * @param dictionary Dictionary used only during compilation.
 * @param transmission_type Adapter metadata.
 * @param descriptors Mapping descriptors.
 * @param descriptor_count Descriptor count.
 * @param payload_limit Maximum allowed logical wire octets.
 * @return Compilation result.
 * @note Used by CoE/EtherCAT adapters, where the legacy eight-octet limit may
 * be relaxed.
 */
gmp_canopen_pdo_result_t gmp_canopen_txpdo_compile_buffer(
    gmp_canopen_txpdo_t* pdo, gmp_canopen_od_t* dictionary,
    uint32_t key, uint16_t transmission_type,
    const uint32_t* descriptors, uint16_t descriptor_count,
    uint16_t payload_limit);

/**
 * @brief Compile an RX plan for a larger logical process-data buffer.
 * @param pdo RX plan to compile.
 * @param dictionary Dictionary used only during compilation.
 * @param key Adapter-defined process-image key.
 * @param transmission_type Adapter metadata.
 * @param descriptors Mapping descriptors.
 * @param descriptor_count Descriptor count.
 * @param payload_limit Maximum allowed logical wire octets.
 * @return Compilation result.
 */
gmp_canopen_pdo_result_t gmp_canopen_rxpdo_compile_buffer(
    gmp_canopen_rxpdo_t* pdo, gmp_canopen_od_t* dictionary,
    uint32_t key, uint16_t transmission_type,
    const uint32_t* descriptors, uint16_t descriptor_count,
    uint16_t payload_limit);

/**
 * @brief Execute a precompiled TX plan into logical wire-octet cells.
 * @param pdo Compiled TX PDO.
 * @param nmt_state Current CANopen state.
 * @param output Destination logical cells.
 * @param capacity Destination capacity.
 * @param actual_size Receives produced logical-octet count.
 * @return Online transfer result.
 */
gmp_canopen_pdo_result_t gmp_canopen_txpdo_pack_fast(
    const gmp_canopen_txpdo_t* pdo, gmp_canopen_nmt_state_t nmt_state,
    uint16_t* output, uint16_t capacity, uint16_t* actual_size);

/**
 * @brief Execute a precompiled RX plan from logical wire-octet cells.
 * @param pdo Compiled RX PDO.
 * @param nmt_state Current CANopen state.
 * @param input Source logical cells.
 * @param size Exact source length.
 * @return Online transfer result.
 */
gmp_canopen_pdo_result_t gmp_canopen_rxpdo_unpack_fast(
    const gmp_canopen_rxpdo_t* pdo, gmp_canopen_nmt_state_t nmt_state,
    const uint16_t* input, uint16_t size);

/**
 * @brief Build one classic-CAN frame from a precompiled TX PDO.
 * @param pdo Compiled classic-CAN TX PDO.
 * @param nmt_state Current CANopen state.
 * @param frame Output frame.
 * @return Online transfer result.
 */
gmp_canopen_pdo_result_t gmp_canopen_txpdo_build_frame_fast(
    const gmp_canopen_txpdo_t* pdo, gmp_canopen_nmt_state_t nmt_state,
    gmp_canopen_frame_t* frame);

/**
 * @brief Apply one classic-CAN frame through a precompiled RX PDO.
 * @param pdo Compiled classic-CAN RX PDO.
 * @param nmt_state Current CANopen state.
 * @param frame Received frame.
 * @return Online transfer result.
 */
gmp_canopen_pdo_result_t gmp_canopen_rxpdo_apply_frame_fast(
    const gmp_canopen_rxpdo_t* pdo, gmp_canopen_nmt_state_t nmt_state,
    const gmp_canopen_frame_t* frame);

/** @brief Initialize an empty sorted TX group. @param group Group to reset. */
void gmp_canopen_txpdo_group_init(gmp_canopen_txpdo_group_t* group);
/** @brief Initialize an empty sorted RX group. @param group Group to reset. */
void gmp_canopen_rxpdo_group_init(gmp_canopen_rxpdo_group_t* group);
/**
 * @brief Register a TX PDO by key, preserving sorted order.
 * @param group Destination group.
 * @param pdo Enabled PDO with stable lifetime.
 * @return Registration result.
 */
gmp_canopen_pdo_result_t gmp_canopen_txpdo_group_add(
    gmp_canopen_txpdo_group_t* group, gmp_canopen_txpdo_t* pdo);
/**
 * @brief Register an RX PDO by key, preserving sorted order.
 * @param group Destination group.
 * @param pdo Enabled PDO with stable lifetime.
 * @return Registration result.
 */
gmp_canopen_pdo_result_t gmp_canopen_rxpdo_group_add(
    gmp_canopen_rxpdo_group_t* group, gmp_canopen_rxpdo_t* pdo);
/**
 * @brief Find a TX PDO by key using binary search.
 * @param group Sorted TX group.
 * @param key COB-ID/process key.
 * @return Registered PDO or `NULL`.
 */
gmp_canopen_txpdo_t* gmp_canopen_txpdo_group_find(
    const gmp_canopen_txpdo_group_t* group, uint32_t key);
/**
 * @brief Find an RX PDO by key using binary search.
 * @param group Sorted RX group.
 * @param key COB-ID/process key.
 * @return Registered PDO or `NULL`.
 */
gmp_canopen_rxpdo_t* gmp_canopen_rxpdo_group_find(
    const gmp_canopen_rxpdo_group_t* group, uint32_t key);
/**
 * @brief Build the TX frame at a sorted group slot in constant lookup time.
 * @param group Sorted TX group.
 * @param slot Sorted slot index.
 * @param nmt_state Current CANopen state.
 * @param frame Output frame.
 * @return Online transfer or not-found result.
 * @note Slots follow ascending key order, not insertion order.
 */
gmp_canopen_pdo_result_t gmp_canopen_txpdo_group_build_at_fast(
    const gmp_canopen_txpdo_group_t* group, uint16_t slot,
    gmp_canopen_nmt_state_t nmt_state, gmp_canopen_frame_t* frame);
/**
 * @brief Dispatch a received frame to its RX PDO using binary search.
 * @param group Sorted RX group.
 * @param nmt_state Current CANopen state.
 * @param frame Received frame.
 * @return Online transfer or not-found result.
 */
gmp_canopen_pdo_result_t gmp_canopen_rxpdo_group_dispatch_fast(
    const gmp_canopen_rxpdo_group_t* group,
    gmp_canopen_nmt_state_t nmt_state, const gmp_canopen_frame_t* frame);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CANOPEN_PDO_ENGINE_H_ */
