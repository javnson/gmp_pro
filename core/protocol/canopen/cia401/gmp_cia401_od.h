/** @file gmp_cia401_od.h @brief Generated from an EDS file; do not edit. */
#ifndef _FILE_GMP_CIA401_OD_H_
#define _FILE_GMP_CIA401_OD_H_

#include <core/protocol/canopen/od.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define GMP_CIA401_OD_ENTRY_COUNT 25U

extern gmp_canopen_od_entry_t gmp_cia401_od_entries[GMP_CIA401_OD_ENTRY_COUNT];
extern uint32_t gmp_cia401_od_storage_1000_00;
extern uint_least8_t gmp_cia401_od_storage_1001_00;
extern uint16_t gmp_cia401_od_storage_1017_00;
extern uint_least8_t gmp_cia401_od_storage_1018_00;
extern uint32_t gmp_cia401_od_storage_1018_01;
extern uint32_t gmp_cia401_od_storage_1018_02;
extern uint32_t gmp_cia401_od_storage_1018_03;
extern uint32_t gmp_cia401_od_storage_1018_04;
extern uint_least8_t gmp_cia401_od_storage_1200_00;
extern uint32_t gmp_cia401_od_storage_1200_01;
extern uint32_t gmp_cia401_od_storage_1200_02;
extern uint_least8_t gmp_cia401_od_storage_1400_00;
extern uint32_t gmp_cia401_od_storage_1400_01;
extern uint_least8_t gmp_cia401_od_storage_1400_02;
extern uint_least8_t gmp_cia401_od_storage_1600_00;
extern uint32_t gmp_cia401_od_storage_1600_01;
extern uint_least8_t gmp_cia401_od_storage_1800_00;
extern uint32_t gmp_cia401_od_storage_1800_01;
extern uint_least8_t gmp_cia401_od_storage_1800_02;
extern uint_least8_t gmp_cia401_od_storage_1a00_00;
extern uint32_t gmp_cia401_od_storage_1a00_01;
extern uint_least8_t gmp_cia401_od_storage_6000_01;
extern uint_least8_t gmp_cia401_od_storage_6200_01;
extern int16_t gmp_cia401_od_storage_6401_01;
extern int16_t gmp_cia401_od_storage_6411_01;
fast_gt gmp_cia401_od_init(gmp_canopen_od_t* dictionary);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CIA401_OD_H_ */
