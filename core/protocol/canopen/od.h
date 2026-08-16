/** @file od.h @brief CANopen object dictionary backed by an intrusive RB tree. */

#ifndef _FILE_GMP_CANOPEN_OD_H_
#define _FILE_GMP_CANOPEN_OD_H_

#include <core/base/ds/rbtree.h>
#include <core/protocol/canopen/can_if.h>

#ifdef __cplusplus
extern "C"
{
#endif

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

typedef enum
{
    GMP_CANOPEN_OD_ACCESS_NONE = 0,
    GMP_CANOPEN_OD_ACCESS_READ = 1U << 0,
    GMP_CANOPEN_OD_ACCESS_WRITE = 1U << 1,
    GMP_CANOPEN_OD_ACCESS_CONST = 1U << 2,
    GMP_CANOPEN_OD_ACCESS_PDO = 1U << 3
} gmp_canopen_od_access_t;

typedef enum
{
    GMP_CANOPEN_OD_STORAGE_VALUE = 0,
    GMP_CANOPEN_OD_STORAGE_POINTER = 1
} gmp_canopen_od_storage_t;

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

typedef union
{
    fast_gt boolean;
    int_least8_t i8;
    uint_least8_t u8;
    int16_t i16;
    uint16_t u16;
    int32_t i32;
    uint32_t u32;
    int64_t i64;
    uint64_t u64;
    float real32;
    double real64;
    gmp_canopen_octet_t octets[8];
} gmp_canopen_od_value_t;

typedef union
{
    void* raw;
    fast_gt* boolean;
    int_least8_t* i8;
    uint_least8_t* u8;
    int16_t* i16;
    uint16_t* u16;
    int32_t* i32;
    uint32_t* u32;
    int64_t* i64;
    uint64_t* u64;
    float* real32;
    double* real64;
    char* visible_string;
    gmp_canopen_octet_t* octets;
} gmp_canopen_od_pointer_t;

typedef union
{
    gmp_canopen_od_value_t value;
    gmp_canopen_od_pointer_t pointer;
} gmp_canopen_od_storage_value_t;

typedef struct
{
    gmp_rb_node rb_node; /* First member: intrusive dictionary node. */
    uint16_t index;
    uint16_t subindex;
    gmp_canopen_od_data_type_t data_type;
    uint16_t access;
    gmp_canopen_od_storage_t storage_mode;
    uint32_t size;
    const char* name;
    gmp_canopen_od_storage_value_t storage;
} gmp_canopen_od_entry_t;

typedef struct
{
    gmp_rb_root entries;
} gmp_canopen_od_t;

void gmp_canopen_od_init(gmp_canopen_od_t* dictionary);
void gmp_canopen_od_entry_init_value(gmp_canopen_od_entry_t* entry,
                                     uint16_t index, uint16_t subindex,
                                     gmp_canopen_od_data_type_t data_type,
                                     uint16_t access, uint32_t size,
                                     const char* name);
void gmp_canopen_od_entry_init_pointer(gmp_canopen_od_entry_t* entry,
                                       uint16_t index, uint16_t subindex,
                                       gmp_canopen_od_data_type_t data_type,
                                       uint16_t access, void* value,
                                       uint32_t size, const char* name);
gmp_canopen_od_result_t gmp_canopen_od_insert(
    gmp_canopen_od_t* dictionary, gmp_canopen_od_entry_t* entry);
gmp_canopen_od_result_t gmp_canopen_od_remove(
    gmp_canopen_od_t* dictionary, gmp_canopen_od_entry_t* entry);
gmp_canopen_od_entry_t* gmp_canopen_od_find(const gmp_canopen_od_t* dictionary,
                                            uint16_t index,
                                            uint16_t subindex);
gmp_canopen_od_entry_t* gmp_canopen_od_first(const gmp_canopen_od_t* dictionary);
gmp_canopen_od_entry_t* gmp_canopen_od_next(const gmp_canopen_od_entry_t* entry);
gmp_canopen_od_result_t gmp_canopen_od_read(
    const gmp_canopen_od_entry_t* entry, gmp_canopen_octet_t* output,
    uint32_t capacity, uint32_t* actual_size);
gmp_canopen_od_result_t gmp_canopen_od_write(
    gmp_canopen_od_entry_t* entry, const gmp_canopen_octet_t* input,
    uint32_t size);
fast_gt gmp_canopen_od_validate(const gmp_canopen_od_t* dictionary,
                                size_gt max_entries);
uint32_t gmp_canopen_od_type_size(gmp_canopen_od_data_type_t data_type);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CANOPEN_OD_H_ */
