/** @file pdo_engine.h @brief Compile and execute classic CANopen PDO maps. */

#ifndef _FILE_GMP_CANOPEN_PDO_ENGINE_H_
#define _FILE_GMP_CANOPEN_PDO_ENGINE_H_

#include <core/protocol/canopen/nmt_sm.h>
#include <core/protocol/canopen/od.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define GMP_CANOPEN_PDO_MAX_MAPPINGS 8U
#define GMP_CANOPEN_PDO_MAP(index, subindex, bits) \
    (((uint32_t)(index) << 16U) | ((uint32_t)(subindex) << 8U) | (uint32_t)(bits))

typedef enum
{
    GMP_CANOPEN_PDO_RECEIVE = 0,
    GMP_CANOPEN_PDO_TRANSMIT = 1
} gmp_canopen_pdo_direction_t;

typedef enum
{
    GMP_CANOPEN_PDO_OK = 0,
    GMP_CANOPEN_PDO_INVALID,
    GMP_CANOPEN_PDO_NOT_MAPPABLE,
    GMP_CANOPEN_PDO_TOO_LARGE,
    GMP_CANOPEN_PDO_NOT_OPERATIONAL,
    GMP_CANOPEN_PDO_IO_ERROR
} gmp_canopen_pdo_result_t;

typedef struct
{
    gmp_canopen_od_entry_t* entry;
    uint16_t byte_offset;
    uint16_t byte_length;
} gmp_canopen_pdo_mapping_t;

typedef struct
{
    uint32_t cob_id;
    gmp_canopen_pdo_direction_t direction;
    uint16_t transmission_type;
    uint16_t mapping_count;
    uint16_t payload_size;
    fast_gt enabled;
    gmp_canopen_pdo_mapping_t mappings[GMP_CANOPEN_PDO_MAX_MAPPINGS];
} gmp_canopen_pdo_t;

gmp_canopen_pdo_result_t gmp_canopen_pdo_compile(
    gmp_canopen_pdo_t* pdo, gmp_canopen_od_t* dictionary,
    gmp_canopen_pdo_direction_t direction, uint32_t cob_id,
    uint16_t transmission_type, const uint32_t* descriptors,
    uint16_t descriptor_count);
gmp_canopen_pdo_result_t gmp_canopen_pdo_build(
    const gmp_canopen_pdo_t* pdo, gmp_canopen_nmt_state_t nmt_state,
    gmp_canopen_frame_t* frame);
gmp_canopen_pdo_result_t gmp_canopen_pdo_apply(
    const gmp_canopen_pdo_t* pdo, gmp_canopen_nmt_state_t nmt_state,
    const gmp_canopen_frame_t* frame);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CANOPEN_PDO_ENGINE_H_ */
