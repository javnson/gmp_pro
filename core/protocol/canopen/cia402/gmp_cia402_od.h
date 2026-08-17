/** @file gmp_cia402_od.h @brief Generated from an EDS file; do not edit. */
#ifndef _FILE_GMP_CIA402_OD_H_
#define _FILE_GMP_CIA402_OD_H_

#include <core/protocol/canopen/od.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Generated OD entry count. */
#define GMP_CIA402_OD_ENTRY_COUNT 31U

/** @brief Generated intrusive entry storage. */
extern gmp_canopen_od_entry_t gmp_cia402_od_entries[GMP_CIA402_OD_ENTRY_COUNT];
/** @brief Storage for 0x1000:00 (Device type). */
extern uint32_t gmp_cia402_od_storage_1000_00;
/** @brief Storage for 0x1001:00 (Error register). */
extern byte_gt gmp_cia402_od_storage_1001_00;
/** @brief Storage for 0x1017:00 (Producer heartbeat time). */
extern uint16_t gmp_cia402_od_storage_1017_00;
/** @brief Storage for 0x1018:00 (Identity highest sub-index). */
extern byte_gt gmp_cia402_od_storage_1018_00;
/** @brief Storage for 0x1018:01 (Vendor ID). */
extern uint32_t gmp_cia402_od_storage_1018_01;
/** @brief Storage for 0x1018:02 (Product code). */
extern uint32_t gmp_cia402_od_storage_1018_02;
/** @brief Storage for 0x1018:03 (Revision number). */
extern uint32_t gmp_cia402_od_storage_1018_03;
/** @brief Storage for 0x1018:04 (Serial number). */
extern uint32_t gmp_cia402_od_storage_1018_04;
/** @brief Storage for 0x1200:00 (SDO server highest sub-index). */
extern byte_gt gmp_cia402_od_storage_1200_00;
/** @brief Storage for 0x1200:01 (SDO client-to-server COB-ID). */
extern uint32_t gmp_cia402_od_storage_1200_01;
/** @brief Storage for 0x1200:02 (SDO server-to-client COB-ID). */
extern uint32_t gmp_cia402_od_storage_1200_02;
/** @brief Storage for 0x1400:00 (RPDO1 communication highest sub-index). */
extern byte_gt gmp_cia402_od_storage_1400_00;
/** @brief Storage for 0x1400:01 (RPDO1 COB-ID). */
extern uint32_t gmp_cia402_od_storage_1400_01;
/** @brief Storage for 0x1400:02 (RPDO1 transmission type). */
extern byte_gt gmp_cia402_od_storage_1400_02;
/** @brief Storage for 0x1600:00 (RPDO1 mapped object count). */
extern byte_gt gmp_cia402_od_storage_1600_00;
/** @brief Storage for 0x1600:01 (RPDO1 mapping controlword). */
extern uint32_t gmp_cia402_od_storage_1600_01;
/** @brief Storage for 0x1600:02 (RPDO1 mapping target velocity). */
extern uint32_t gmp_cia402_od_storage_1600_02;
/** @brief Storage for 0x1800:00 (TPDO1 communication highest sub-index). */
extern byte_gt gmp_cia402_od_storage_1800_00;
/** @brief Storage for 0x1800:01 (TPDO1 COB-ID). */
extern uint32_t gmp_cia402_od_storage_1800_01;
/** @brief Storage for 0x1800:02 (TPDO1 transmission type). */
extern byte_gt gmp_cia402_od_storage_1800_02;
/** @brief Storage for 0x1A00:00 (TPDO1 mapped object count). */
extern byte_gt gmp_cia402_od_storage_1a00_00;
/** @brief Storage for 0x1A00:01 (TPDO1 mapping statusword). */
extern uint32_t gmp_cia402_od_storage_1a00_01;
/** @brief Storage for 0x1A00:02 (TPDO1 mapping velocity actual value). */
extern uint32_t gmp_cia402_od_storage_1a00_02;
/** @brief Storage for 0x6040:00 (Controlword). */
extern uint16_t gmp_cia402_od_storage_6040_00;
/** @brief Storage for 0x6041:00 (Statusword). */
extern uint16_t gmp_cia402_od_storage_6041_00;
/** @brief Storage for 0x6060:00 (Modes of operation). */
extern int_least8_t gmp_cia402_od_storage_6060_00;
/** @brief Storage for 0x6061:00 (Modes of operation display). */
extern int_least8_t gmp_cia402_od_storage_6061_00;
/** @brief Storage for 0x6064:00 (Position actual value). */
extern int32_t gmp_cia402_od_storage_6064_00;
/** @brief Storage for 0x606C:00 (Velocity actual value). */
extern int32_t gmp_cia402_od_storage_606c_00;
/** @brief Storage for 0x607A:00 (Target position). */
extern int32_t gmp_cia402_od_storage_607a_00;
/** @brief Storage for 0x60FF:00 (Target velocity). */
extern int32_t gmp_cia402_od_storage_60ff_00;
/**
 * @brief Initialize the generated object dictionary exactly once.
 * @param dictionary Empty dictionary that receives all generated entries.
 * @return Non-zero when every entry is inserted and the RB tree validates.
 */
fast_gt gmp_cia402_od_init(gmp_canopen_od_t* dictionary);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CIA402_OD_H_ */
