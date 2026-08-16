/** @file pdo_engine.c @brief Compiled classic CANopen PDO mapping. */

#include <core/protocol/canopen/pdo_engine.h>

gmp_canopen_pdo_result_t gmp_canopen_pdo_compile(
    gmp_canopen_pdo_t* pdo, gmp_canopen_od_t* dictionary,
    gmp_canopen_pdo_direction_t direction, uint32_t cob_id,
    uint16_t transmission_type, const uint32_t* descriptors,
    uint16_t descriptor_count)
{
    uint16_t mapping_index;
    uint16_t offset = 0U;
    if (pdo == NULL || dictionary == NULL || descriptors == NULL ||
        descriptor_count == 0U || descriptor_count > GMP_CANOPEN_PDO_MAX_MAPPINGS ||
        cob_id > 0x7FFU)
        return GMP_CANOPEN_PDO_INVALID;
    pdo->enabled = 0;
    pdo->mapping_count = 0U;
    pdo->payload_size = 0U;
    for (mapping_index = 0U; mapping_index < descriptor_count; ++mapping_index)
    {
        uint32_t descriptor = descriptors[mapping_index];
        uint16_t index = (uint16_t)(descriptor >> 16U);
        uint16_t subindex = (uint16_t)((descriptor >> 8U) & 0xFFU);
        uint16_t bits = (uint16_t)(descriptor & 0xFFU);
        uint16_t bytes;
        gmp_canopen_od_entry_t* entry;
        if (bits == 0U || (bits & 7U) != 0U)
            return GMP_CANOPEN_PDO_INVALID;
        bytes = bits / 8U;
        if (offset + bytes > GMP_CANOPEN_CLASSIC_MAX_DATA)
            return GMP_CANOPEN_PDO_TOO_LARGE;
        entry = gmp_canopen_od_find(dictionary, index, subindex);
        if (entry == NULL || !(entry->access & GMP_CANOPEN_OD_ACCESS_PDO) ||
            entry->size != bytes ||
            (direction == GMP_CANOPEN_PDO_TRANSMIT &&
             !(entry->access & GMP_CANOPEN_OD_ACCESS_READ)) ||
            (direction == GMP_CANOPEN_PDO_RECEIVE &&
             !(entry->access & GMP_CANOPEN_OD_ACCESS_WRITE)))
            return GMP_CANOPEN_PDO_NOT_MAPPABLE;
        pdo->mappings[mapping_index].entry = entry;
        pdo->mappings[mapping_index].byte_offset = offset;
        pdo->mappings[mapping_index].byte_length = bytes;
        offset = (uint16_t)(offset + bytes);
    }
    pdo->cob_id = cob_id;
    pdo->direction = direction;
    pdo->transmission_type = transmission_type;
    pdo->mapping_count = descriptor_count;
    pdo->payload_size = offset;
    pdo->enabled = 1;
    return GMP_CANOPEN_PDO_OK;
}

gmp_canopen_pdo_result_t gmp_canopen_pdo_build(
    const gmp_canopen_pdo_t* pdo, gmp_canopen_nmt_state_t nmt_state,
    gmp_canopen_frame_t* frame)
{
    uint16_t mapping_index;
    if (pdo == NULL || frame == NULL || !pdo->enabled ||
        pdo->direction != GMP_CANOPEN_PDO_TRANSMIT)
        return GMP_CANOPEN_PDO_INVALID;
    if (nmt_state != GMP_CANOPEN_NMT_OPERATIONAL)
        return GMP_CANOPEN_PDO_NOT_OPERATIONAL;
    gmp_canopen_frame_clear(frame);
    frame->id = pdo->cob_id;
    frame->dlc = (gmp_canopen_octet_t)pdo->payload_size;
    for (mapping_index = 0U; mapping_index < pdo->mapping_count; ++mapping_index)
    {
        const gmp_canopen_pdo_mapping_t* mapping = &pdo->mappings[mapping_index];
        uint32_t actual_size = 0U;
        if (gmp_canopen_od_read(mapping->entry,
                                &frame->data[mapping->byte_offset],
                                mapping->byte_length, &actual_size) != GMP_CANOPEN_OD_OK ||
            actual_size != mapping->byte_length)
            return GMP_CANOPEN_PDO_IO_ERROR;
    }
    return GMP_CANOPEN_PDO_OK;
}

gmp_canopen_pdo_result_t gmp_canopen_pdo_apply(
    const gmp_canopen_pdo_t* pdo, gmp_canopen_nmt_state_t nmt_state,
    const gmp_canopen_frame_t* frame)
{
    uint16_t mapping_index;
    if (pdo == NULL || frame == NULL || !pdo->enabled ||
        pdo->direction != GMP_CANOPEN_PDO_RECEIVE || frame->is_extended ||
        frame->is_remote || frame->id != pdo->cob_id ||
        frame->dlc != pdo->payload_size)
        return GMP_CANOPEN_PDO_INVALID;
    if (nmt_state != GMP_CANOPEN_NMT_OPERATIONAL)
        return GMP_CANOPEN_PDO_NOT_OPERATIONAL;
    for (mapping_index = 0U; mapping_index < pdo->mapping_count; ++mapping_index)
    {
        const gmp_canopen_pdo_mapping_t* mapping = &pdo->mappings[mapping_index];
        if (gmp_canopen_od_write(mapping->entry,
                                 &frame->data[mapping->byte_offset],
                                 mapping->byte_length) != GMP_CANOPEN_OD_OK)
            return GMP_CANOPEN_PDO_IO_ERROR;
    }
    return GMP_CANOPEN_PDO_OK;
}
