/** @file gmp_cia401_od.c @brief Generated from cia401.eds; do not edit. */
#include "gmp_cia401_od.h"

gmp_canopen_od_entry_t gmp_cia401_od_entries[GMP_CIA401_OD_ENTRY_COUNT];
uint32_t gmp_cia401_od_storage_1000_00 = (uint32_t)401;
uint_least8_t gmp_cia401_od_storage_1001_00 = (uint_least8_t)0;
uint16_t gmp_cia401_od_storage_1017_00 = (uint16_t)1000;
uint_least8_t gmp_cia401_od_storage_1018_00 = (uint_least8_t)4;
uint32_t gmp_cia401_od_storage_1018_01 = (uint32_t)0;
uint32_t gmp_cia401_od_storage_1018_02 = (uint32_t)0;
uint32_t gmp_cia401_od_storage_1018_03 = (uint32_t)1;
uint32_t gmp_cia401_od_storage_1018_04 = (uint32_t)0;
uint_least8_t gmp_cia401_od_storage_1200_00 = (uint_least8_t)2;
uint32_t gmp_cia401_od_storage_1200_01 = (uint32_t)1537;
uint32_t gmp_cia401_od_storage_1200_02 = (uint32_t)1409;
uint_least8_t gmp_cia401_od_storage_1400_00 = (uint_least8_t)2;
uint32_t gmp_cia401_od_storage_1400_01 = (uint32_t)513;
uint_least8_t gmp_cia401_od_storage_1400_02 = (uint_least8_t)255;
uint_least8_t gmp_cia401_od_storage_1600_00 = (uint_least8_t)1;
uint32_t gmp_cia401_od_storage_1600_01 = (uint32_t)1644167432;
uint_least8_t gmp_cia401_od_storage_1800_00 = (uint_least8_t)2;
uint32_t gmp_cia401_od_storage_1800_01 = (uint32_t)385;
uint_least8_t gmp_cia401_od_storage_1800_02 = (uint_least8_t)255;
uint_least8_t gmp_cia401_od_storage_1a00_00 = (uint_least8_t)1;
uint32_t gmp_cia401_od_storage_1a00_01 = (uint32_t)1610613000;
uint_least8_t gmp_cia401_od_storage_6000_01 = (uint_least8_t)0;
uint_least8_t gmp_cia401_od_storage_6200_01 = (uint_least8_t)0;
int16_t gmp_cia401_od_storage_6401_01 = (int16_t)0;
int16_t gmp_cia401_od_storage_6411_01 = (int16_t)0;

fast_gt gmp_cia401_od_init(gmp_canopen_od_t* dictionary)
{
    size_gt entry_index;
    if (dictionary == NULL)
        return 0;
    for (entry_index = 0U; entry_index < GMP_CIA401_OD_ENTRY_COUNT; ++entry_index)
        if (!gmp_rb_node_is_detached(&gmp_cia401_od_entries[entry_index].rb_node))
            return 0;
    gmp_canopen_od_init(dictionary);
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[0U],
        0x1000U, 0x00U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia401_od_storage_1000_00, 4U, "Device type");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[0U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[1U],
        0x1001U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia401_od_storage_1001_00, 1U, "Error register");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[1U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[2U],
        0x1017U, 0x00U, GMP_CANOPEN_OD_UNSIGNED16,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia401_od_storage_1017_00, 2U, "Producer heartbeat time");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[2U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[3U],
        0x1018U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia401_od_storage_1018_00, 1U, "Identity highest sub-index");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[3U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[4U],
        0x1018U, 0x01U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia401_od_storage_1018_01, 4U, "Vendor ID");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[4U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[5U],
        0x1018U, 0x02U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia401_od_storage_1018_02, 4U, "Product code");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[5U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[6U],
        0x1018U, 0x03U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia401_od_storage_1018_03, 4U, "Revision number");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[6U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[7U],
        0x1018U, 0x04U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ, &gmp_cia401_od_storage_1018_04, 4U, "Serial number");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[7U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[8U],
        0x1200U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia401_od_storage_1200_00, 1U, "SDO server highest sub-index");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[8U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[9U],
        0x1200U, 0x01U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ, &gmp_cia401_od_storage_1200_01, 4U, "SDO client-to-server COB-ID");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[9U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[10U],
        0x1200U, 0x02U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ, &gmp_cia401_od_storage_1200_02, 4U, "SDO server-to-client COB-ID");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[10U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[11U],
        0x1400U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia401_od_storage_1400_00, 1U, "RPDO1 communication highest sub-index");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[11U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[12U],
        0x1400U, 0x01U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia401_od_storage_1400_01, 4U, "RPDO1 COB-ID");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[12U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[13U],
        0x1400U, 0x02U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia401_od_storage_1400_02, 1U, "RPDO1 transmission type");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[13U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[14U],
        0x1600U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia401_od_storage_1600_00, 1U, "RPDO1 mapped object count");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[14U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[15U],
        0x1600U, 0x01U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia401_od_storage_1600_01, 4U, "RPDO1 mapping digital outputs");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[15U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[16U],
        0x1800U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST, &gmp_cia401_od_storage_1800_00, 1U, "TPDO1 communication highest sub-index");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[16U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[17U],
        0x1800U, 0x01U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia401_od_storage_1800_01, 4U, "TPDO1 COB-ID");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[17U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[18U],
        0x1800U, 0x02U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia401_od_storage_1800_02, 1U, "TPDO1 transmission type");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[18U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[19U],
        0x1A00U, 0x00U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia401_od_storage_1a00_00, 1U, "TPDO1 mapped object count");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[19U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[20U],
        0x1A00U, 0x01U, GMP_CANOPEN_OD_UNSIGNED32,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE, &gmp_cia401_od_storage_1a00_01, 4U, "TPDO1 mapping digital inputs");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[20U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[21U],
        0x6000U, 0x01U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia401_od_storage_6000_01, 1U, "Read input 8-bit channel 1");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[21U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[22U],
        0x6200U, 0x01U, GMP_CANOPEN_OD_UNSIGNED8,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia401_od_storage_6200_01, 1U, "Write output 8-bit channel 1");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[22U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[23U],
        0x6401U, 0x01U, GMP_CANOPEN_OD_INTEGER16,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia401_od_storage_6401_01, 2U, "Read analog input 16-bit channel 1");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[23U]) != GMP_CANOPEN_OD_OK)
        return 0;
    gmp_canopen_od_entry_init_pointer(&gmp_cia401_od_entries[24U],
        0x6411U, 0x01U, GMP_CANOPEN_OD_INTEGER16,
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE | GMP_CANOPEN_OD_ACCESS_PDO, &gmp_cia401_od_storage_6411_01, 2U, "Write analog output 16-bit channel 1");
    if (gmp_canopen_od_insert(dictionary, &gmp_cia401_od_entries[24U]) != GMP_CANOPEN_OD_OK)
        return 0;
    return gmp_canopen_od_validate(dictionary, GMP_CIA401_OD_ENTRY_COUNT);
}
