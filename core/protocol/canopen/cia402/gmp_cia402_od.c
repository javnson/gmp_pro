/** @file gmp_cia402_od.c @brief Generated from cia402.eds; do not edit. */
#include "gmp_cia402_od.h"

gmp_canopen_od_entry_t gmp_cia402_od_entries[GMP_CIA402_OD_ENTRY_COUNT];
uint32_t gmp_cia402_od_storage_1000_00 = (uint32_t)402;
byte_gt gmp_cia402_od_storage_1001_00 = (byte_gt)0;
uint16_t gmp_cia402_od_storage_1017_00 = (uint16_t)1000;
byte_gt gmp_cia402_od_storage_1018_00 = (byte_gt)4;
uint32_t gmp_cia402_od_storage_1018_01 = (uint32_t)0;
uint32_t gmp_cia402_od_storage_1018_02 = (uint32_t)0;
uint32_t gmp_cia402_od_storage_1018_03 = (uint32_t)1;
uint32_t gmp_cia402_od_storage_1018_04 = (uint32_t)0;
byte_gt gmp_cia402_od_storage_1200_00 = (byte_gt)2;
uint32_t gmp_cia402_od_storage_1200_01 = (uint32_t)1537;
uint32_t gmp_cia402_od_storage_1200_02 = (uint32_t)1409;
byte_gt gmp_cia402_od_storage_1400_00 = (byte_gt)2;
uint32_t gmp_cia402_od_storage_1400_01 = (uint32_t)513;
byte_gt gmp_cia402_od_storage_1400_02 = (byte_gt)255;
byte_gt gmp_cia402_od_storage_1600_00 = (byte_gt)2;
uint32_t gmp_cia402_od_storage_1600_01 = (uint32_t)1614807056;
uint32_t gmp_cia402_od_storage_1600_02 = (uint32_t)1627324448;
byte_gt gmp_cia402_od_storage_1800_00 = (byte_gt)2;
uint32_t gmp_cia402_od_storage_1800_01 = (uint32_t)385;
byte_gt gmp_cia402_od_storage_1800_02 = (byte_gt)255;
byte_gt gmp_cia402_od_storage_1a00_00 = (byte_gt)2;
uint32_t gmp_cia402_od_storage_1a00_01 = (uint32_t)1614872592;
uint32_t gmp_cia402_od_storage_1a00_02 = (uint32_t)1617690656;
uint16_t gmp_cia402_od_storage_6040_00 = (uint16_t)0;
uint16_t gmp_cia402_od_storage_6041_00 = (uint16_t)0;
int_least8_t gmp_cia402_od_storage_6060_00 = (int_least8_t)0;
int_least8_t gmp_cia402_od_storage_6061_00 = (int_least8_t)0;
int32_t gmp_cia402_od_storage_6064_00 = (int32_t)0;
int32_t gmp_cia402_od_storage_606c_00 = (int32_t)0;
int32_t gmp_cia402_od_storage_607a_00 = (int32_t)0;
int32_t gmp_cia402_od_storage_60ff_00 = (int32_t)0;

fast_gt gmp_cia402_od_init(gmp_canopen_od_t* dictionary)
{
    size_gt entry_index;
    if (dictionary == NULL)
        return 0;
    for (entry_index = 0U; entry_index < GMP_CIA402_OD_ENTRY_COUNT; ++entry_index)
        if (!gmp_rb_node_is_detached(&gmp_cia402_od_entries[entry_index].rb_node))
            return 0;
    gmp_canopen_od_init(dictionary);
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[0U],
        0x1000U, 0x00U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia402_od_storage_1000_00, 4U, "Device type");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[0U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[1U],
        0x1001U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia402_od_storage_1001_00, 1U, "Error register");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[1U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[2U],
        0x1017U, 0x00U, GMP_CANOPEN_OD_UNSIGNED16,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia402_od_storage_1017_00, 2U, "Producer heartbeat time");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[2U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[3U],
        0x1018U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia402_od_storage_1018_00, 1U, "Identity highest sub-index");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[3U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[4U],
        0x1018U, 0x01U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia402_od_storage_1018_01, 4U, "Vendor ID");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[4U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[5U],
        0x1018U, 0x02U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia402_od_storage_1018_02, 4U, "Product code");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[5U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[6U],
        0x1018U, 0x03U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia402_od_storage_1018_03, 4U, "Revision number");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[6U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[7U],
        0x1018U, 0x04U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ, &gmp_cia402_od_storage_1018_04, 4U, "Serial number");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[7U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[8U],
        0x1200U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia402_od_storage_1200_00, 1U, "SDO server highest sub-index");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[8U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[9U],
        0x1200U, 0x01U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ, &gmp_cia402_od_storage_1200_01, 4U, "SDO client-to-server COB-ID");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[9U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[10U],
        0x1200U, 0x02U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ, &gmp_cia402_od_storage_1200_02, 4U, "SDO server-to-client COB-ID");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[10U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[11U],
        0x1400U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia402_od_storage_1400_00, 1U, "RPDO1 communication highest sub-index");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[11U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[12U],
        0x1400U, 0x01U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia402_od_storage_1400_01, 4U, "RPDO1 COB-ID");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[12U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[13U],
        0x1400U, 0x02U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia402_od_storage_1400_02, 1U, "RPDO1 transmission type");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[13U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[14U],
        0x1600U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia402_od_storage_1600_00, 1U, "RPDO1 mapped object count");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[14U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[15U],
        0x1600U, 0x01U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia402_od_storage_1600_01, 4U, "RPDO1 mapping controlword");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[15U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[16U],
        0x1600U, 0x02U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia402_od_storage_1600_02, 4U, "RPDO1 mapping target velocity");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[16U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[17U],
        0x1800U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia402_od_storage_1800_00, 1U, "TPDO1 communication highest sub-index");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[17U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[18U],
        0x1800U, 0x01U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia402_od_storage_1800_01, 4U, "TPDO1 COB-ID");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[18U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[19U],
        0x1800U, 0x02U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia402_od_storage_1800_02, 1U, "TPDO1 transmission type");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[19U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[20U],
        0x1A00U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia402_od_storage_1a00_00, 1U, "TPDO1 mapped object count");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[20U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[21U],
        0x1A00U, 0x01U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia402_od_storage_1a00_01, 4U, "TPDO1 mapping statusword");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[21U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[22U],
        0x1A00U, 0x02U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia402_od_storage_1a00_02, 4U, "TPDO1 mapping velocity actual value");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[22U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[23U],
        0x6040U, 0x00U, GMP_CANOPEN_OD_UNSIGNED16,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia402_od_storage_6040_00, 2U, "Controlword");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[23U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[24U],
        0x6041U, 0x00U, GMP_CANOPEN_OD_UNSIGNED16,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia402_od_storage_6041_00, 2U, "Statusword");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[24U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[25U],
        0x6060U, 0x00U, GMP_CANOPEN_OD_INTEGER8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia402_od_storage_6060_00, 1U, "Modes of operation");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[25U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[26U],
        0x6061U, 0x00U, GMP_CANOPEN_OD_INTEGER8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia402_od_storage_6061_00, 1U, "Modes of operation display");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[26U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[27U],
        0x6064U, 0x00U, GMP_CANOPEN_OD_INTEGER32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia402_od_storage_6064_00, 4U, "Position actual value");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[27U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[28U],
        0x606CU, 0x00U, GMP_CANOPEN_OD_INTEGER32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia402_od_storage_606c_00, 4U, "Velocity actual value");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[28U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[29U],
        0x607AU, 0x00U, GMP_CANOPEN_OD_INTEGER32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia402_od_storage_607a_00, 4U, "Target position");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[29U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia402_od_entries[30U],
        0x60FFU, 0x00U, GMP_CANOPEN_OD_INTEGER32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia402_od_storage_60ff_00, 4U, "Target velocity");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia402_od_entries[30U]) != GMP_CANOPEN_OD_OK)
        return 0;
    return gmp_canopen_od_validate(dictionary, GMP_CIA402_OD_ENTRY_COUNT);
}
