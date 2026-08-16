/**
 * @file od.h
 * @brief CANopen object dictionary backed by an intrusive red-black tree.
 * @details Dictionary values use either inline typed storage or a pointer to
 * application-owned storage. Serialized buffers contain one eight-bit wire
 * octet per `uint16_t` cell; only the low eight bits are significant.
 */

#ifndef _FILE_GMP_CANOPEN_OD_H_
#define _FILE_GMP_CANOPEN_OD_H_

#include <core/base/ds/rbtree.h>
#include <core/protocol/canopen/can_if.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Supported CiA 301 object-dictionary data-type codes. */
typedef enum
{
    GMP_CANOPEN_OD_BOOLEAN = 0x0001,
    GMP_CANOPEN_OD_INTEGER8 = 0x0002,
    GMP_CANOPEN_OD_INTEGER16 = 0x0003,
    GMP_CANOPEN_OD_INTEGER32 = 0x0004,
    GMP_CANOPEN_OD_UNSIGNED8 = 0x0005,
    GMP_CANOPEN_OD_UNSIGNED16 = 0x0006,
    GMP_CANOPEN_OD_UNSIGNED32 = 0x0007,
    GMP_CANOPEN_OD_REAL32 = 0x0008,
    GMP_CANOPEN_OD_VISIBLE_STRING = 0x0009,
    GMP_CANOPEN_OD_OCTET_STRING = 0x000A,
    GMP_CANOPEN_OD_DOMAIN = 0x000F,
    GMP_CANOPEN_OD_REAL64 = 0x0011,
    GMP_CANOPEN_OD_INTEGER64 = 0x0015,
    GMP_CANOPEN_OD_UNSIGNED64 = 0x001B
} gmp_canopen_od_data_type_t;

/** @brief Access and PDO-mapping attributes for one OD entry. */
typedef enum
{
    GMP_CANOPEN_OD_ACCESS_NONE = 0,
    GMP_CANOPEN_OD_ACCESS_READ = 1U << 0,
    GMP_CANOPEN_OD_ACCESS_WRITE = 1U << 1,
    GMP_CANOPEN_OD_ACCESS_CONST = 1U << 2,
    GMP_CANOPEN_OD_ACCESS_PDO = 1U << 3
} gmp_canopen_od_access_t;

/** @brief Storage ownership model for one OD entry. */
typedef enum
{
    GMP_CANOPEN_OD_STORAGE_VALUE = 0,
    GMP_CANOPEN_OD_STORAGE_POINTER = 1
} gmp_canopen_od_storage_t;

/** @brief Object-dictionary operation result. */
typedef enum
{
    GMP_CANOPEN_OD_OK = 0,
    GMP_CANOPEN_OD_NOT_FOUND,
    GMP_CANOPEN_OD_READ_DENIED,
    GMP_CANOPEN_OD_WRITE_DENIED,
    GMP_CANOPEN_OD_TYPE_MISMATCH,
    GMP_CANOPEN_OD_LENGTH_MISMATCH,
    GMP_CANOPEN_OD_DUPLICATE,
    GMP_CANOPEN_OD_INVALID
} gmp_canopen_od_result_t;

/** @brief Inline typed value storage owned by an OD entry. */
typedef union
{
    fast_gt boolean; /**< Boolean storage. */
    int_least8_t i8; /**< INTEGER8 storage; may occupy 16 bits on C28x. */
    uint_least8_t u8; /**< UNSIGNED8 storage; may occupy 16 bits on C28x. */
    int16_t i16; /**< INTEGER16 storage. */
    uint16_t u16; /**< UNSIGNED16 storage. */
    int32_t i32; /**< INTEGER32 storage. */
    uint32_t u32; /**< UNSIGNED32 storage. */
    int64_t i64; /**< INTEGER64 storage. */
    uint64_t u64; /**< UNSIGNED64 storage. */
    float real32; /**< REAL32 storage. */
    double real64; /**< REAL64 storage. */
    uint16_t octets[8]; /**< Up to eight logical wire octets. */
} gmp_canopen_od_value_t;

/** @brief Typed pointers to application-owned OD storage. */
typedef union
{
    void* raw; /**< Untyped pointer used during initialization/compilation. */
    fast_gt* boolean; /**< Boolean pointer. */
    int_least8_t* i8; /**< INTEGER8 pointer. */
    uint_least8_t* u8; /**< UNSIGNED8 pointer. */
    int16_t* i16; /**< INTEGER16 pointer. */
    uint16_t* u16; /**< UNSIGNED16 pointer. */
    int32_t* i32; /**< INTEGER32 pointer. */
    uint32_t* u32; /**< UNSIGNED32 pointer. */
    int64_t* i64; /**< INTEGER64 pointer. */
    uint64_t* u64; /**< UNSIGNED64 pointer. */
    float* real32; /**< REAL32 pointer. */
    double* real64; /**< REAL64 pointer. */
    uint16_t* octets; /**< Logical wire-octet string/domain storage. */
} gmp_canopen_od_pointer_t;

/** @brief Union of inline-value and pointer-backed storage models. */
typedef union
{
    gmp_canopen_od_value_t value; /**< Inline owned value. */
    gmp_canopen_od_pointer_t pointer; /**< Application-owned pointer. */
} gmp_canopen_od_storage_value_t;

/** @brief One intrusive object-dictionary leaf entry. */
typedef struct
{
    gmp_rb_node rb_node; /**< First member: intrusive dictionary node. */
    uint16_t index; /**< CiA object index. */
    uint16_t subindex; /**< CiA sub-index; valid range is 0..255. */
    gmp_canopen_od_data_type_t data_type; /**< CiA data type. */
    uint16_t access; /**< Bitwise `gmp_canopen_od_access_t`. */
    gmp_canopen_od_storage_t storage_mode; /**< Active storage union member. */
    uint32_t size; /**< Serialized size in logical wire octets. */
    const char* name; /**< Optional diagnostic name; not owned. */
    gmp_canopen_od_storage_value_t storage; /**< Value or pointer storage. */
} gmp_canopen_od_entry_t;

/** @brief Object dictionary root. */
typedef struct
{
    gmp_rb_root entries; /**< Intrusive entry tree ordered by index/sub-index. */
} gmp_canopen_od_t;

/**
 * @brief Initialize an empty dictionary.
 * @param dictionary Root to reset; `NULL` is ignored.
 */
void gmp_canopen_od_init(gmp_canopen_od_t* dictionary);
/**
 * @brief Initialize an entry that owns its value.
 * @param entry Entry to initialize.
 * @param index Object index.
 * @param subindex Object sub-index.
 * @param data_type CiA data type.
 * @param access Access flags.
 * @param size Serialized size, or zero to infer a scalar size.
 * @param name Optional static diagnostic name.
 */
void gmp_canopen_od_entry_init_value(gmp_canopen_od_entry_t* entry,
                                     uint16_t index, uint16_t subindex,
                                     gmp_canopen_od_data_type_t data_type,
                                     uint16_t access, uint32_t size,
                                     const char* name);
/**
 * @brief Initialize an entry backed by application-owned storage.
 * @param entry Entry to initialize.
 * @param index Object index.
 * @param subindex Object sub-index.
 * @param data_type CiA data type.
 * @param access Access flags.
 * @param value Typed scalar pointer or `uint16_t` logical-octet buffer.
 * @param size Serialized size, or zero to infer a scalar size.
 * @param name Optional static diagnostic name.
 */
void gmp_canopen_od_entry_init_pointer(gmp_canopen_od_entry_t* entry,
                                       uint16_t index, uint16_t subindex,
                                       gmp_canopen_od_data_type_t data_type,
                                       uint16_t access, void* value,
                                       uint32_t size, const char* name);
/**
 * @brief Insert a detached entry.
 * @param dictionary Destination dictionary.
 * @param entry Initialized detached entry.
 * @return `GMP_CANOPEN_OD_OK` on success.
 */
gmp_canopen_od_result_t gmp_canopen_od_insert(
    gmp_canopen_od_t* dictionary, gmp_canopen_od_entry_t* entry);
/**
 * @brief Remove an entry owned by this dictionary.
 * @param dictionary Owning dictionary.
 * @param entry Linked entry to detach.
 * @return `GMP_CANOPEN_OD_OK` or `GMP_CANOPEN_OD_NOT_FOUND`.
 */
gmp_canopen_od_result_t gmp_canopen_od_remove(
    gmp_canopen_od_t* dictionary, gmp_canopen_od_entry_t* entry);
/**
 * @brief Find one entry in logarithmic time.
 * @param dictionary Dictionary to search.
 * @param index OD index.
 * @param subindex OD sub-index.
 * @return Matching entry or `NULL`.
 */
gmp_canopen_od_entry_t* gmp_canopen_od_find(const gmp_canopen_od_t* dictionary,
                                            uint16_t index,
                                            uint16_t subindex);
/**
 * @brief Return the lowest ordered entry.
 * @param dictionary Dictionary to inspect.
 * @return First entry or `NULL`.
 */
gmp_canopen_od_entry_t* gmp_canopen_od_first(const gmp_canopen_od_t* dictionary);
/**
 * @brief Return the next ordered entry.
 * @param entry Current linked entry.
 * @return Successor or `NULL`.
 */
gmp_canopen_od_entry_t* gmp_canopen_od_next(const gmp_canopen_od_entry_t* entry);
/**
 * @brief Serialize an entry into logical wire-octet cells.
 * @param entry Readable entry.
 * @param output Output `uint16_t` cells.
 * @param capacity Output capacity in logical octets.
 * @param actual_size Receives serialized logical-octet count.
 * @return OD access/length result.
 */
gmp_canopen_od_result_t gmp_canopen_od_read(
    const gmp_canopen_od_entry_t* entry, uint16_t* output,
    uint32_t capacity, uint32_t* actual_size);
/**
 * @brief Deserialize logical wire-octet cells into a writable entry.
 * @param entry Writable entry.
 * @param input Input cells; every value must be in 0..255.
 * @param size Input length in logical octets.
 * @return OD access/length/value result.
 */
gmp_canopen_od_result_t gmp_canopen_od_write(
    gmp_canopen_od_entry_t* entry, const uint16_t* input,
    uint32_t size);
/**
 * @brief Validate RB-tree ownership, ordering and invariants.
 * @param dictionary Dictionary to audit.
 * @param max_entries Trusted traversal bound.
 * @return Non-zero only when the dictionary is valid.
 */
fast_gt gmp_canopen_od_validate(const gmp_canopen_od_t* dictionary,
                                size_gt max_entries);
/**
 * @brief Return the CiA scalar width in wire octets.
 * @param data_type CiA data type.
 * @return Scalar width, or zero for variable/unsupported types.
 */
uint32_t gmp_canopen_od_type_size(gmp_canopen_od_data_type_t data_type);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CANOPEN_OD_H_ */
