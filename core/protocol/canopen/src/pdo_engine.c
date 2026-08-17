/** @file pdo_engine.c @brief Precompiled CANopen PDO fast paths. */

#include <core/protocol/canopen/pdo_engine.h>

#include <string.h>

typedef enum
{
    GMP_CANOPEN_PDO_COMPILE_RX = 0,
    GMP_CANOPEN_PDO_COMPILE_TX = 1
} gmp_canopen_pdo_compile_direction_t;

static void* gmp_canopen_pdo_resolve_storage(gmp_canopen_od_entry_t* entry)
{
    if (entry->storage_mode == GMP_CANOPEN_OD_STORAGE_POINTER)
        return entry->storage.pointer.raw;
    switch (entry->data_type)
    {
    case GMP_CANOPEN_OD_BOOLEAN: return &entry->storage.value.boolean;
    case GMP_CANOPEN_OD_INTEGER8: return &entry->storage.value.i8;
    case GMP_CANOPEN_OD_UNSIGNED8: return &entry->storage.value.u8;
    case GMP_CANOPEN_OD_INTEGER16: return &entry->storage.value.i16;
    case GMP_CANOPEN_OD_UNSIGNED16: return &entry->storage.value.u16;
    case GMP_CANOPEN_OD_INTEGER32: return &entry->storage.value.i32;
    case GMP_CANOPEN_OD_UNSIGNED32: return &entry->storage.value.u32;
    case GMP_CANOPEN_OD_INTEGER64: return &entry->storage.value.i64;
    case GMP_CANOPEN_OD_UNSIGNED64: return &entry->storage.value.u64;
    case GMP_CANOPEN_OD_REAL32: return &entry->storage.value.real32;
    case GMP_CANOPEN_OD_REAL64: return &entry->storage.value.real64;
    case GMP_CANOPEN_OD_VISIBLE_STRING:
    case GMP_CANOPEN_OD_OCTET_STRING:
    case GMP_CANOPEN_OD_DOMAIN: return entry->storage.value.octets;
    default: return NULL;
    }
}

static void gmp_canopen_pdo_store_unsigned(byte_gt* output,
                                           uint16_t length,
                                           uint64_t value)
{
    uint16_t index;
    for (index = 0U; index < length; ++index)
        output[index] = (byte_gt)((value >> (8U * index)) & 0xFFU);
}

static uint64_t gmp_canopen_pdo_load_unsigned(const byte_gt* input,
                                               uint16_t length)
{
    uint16_t index;
    uint64_t value = 0U;
    for (index = 0U; index < length; ++index)
        value |= (uint64_t)(input[index] & 0xFFU) << (8U * index);
    return value;
}

static int_least8_t gmp_canopen_pdo_decode_integer8(uint64_t value)
{
    uint16_t octet = (uint16_t)(value & 0xFFU);
    return octet < 0x80U ? (int_least8_t)octet :
        (int_least8_t)((int16_t)octet - 0x100);
}

static fast_gt gmp_canopen_pdo_encode_mapping(
    const gmp_canopen_pdo_mapping_t* mapping, byte_gt* output)
{
    uint64_t value;
    uint16_t index;
    switch (mapping->data_type)
    {
    case GMP_CANOPEN_OD_BOOLEAN:
        value = *(const fast_gt*)mapping->storage != 0;
        break;
    case GMP_CANOPEN_OD_INTEGER8:
    {
        int_least8_t signed_value =
            *(const int_least8_t*)mapping->storage;
        if (signed_value < -128 || signed_value > 127)
            return 0;
        value = (uint64_t)signed_value;
        break;
    }
    case GMP_CANOPEN_OD_UNSIGNED8:
        value = *(const byte_gt*)mapping->storage;
        if (value > 0xFFU)
            return 0;
        break;
    case GMP_CANOPEN_OD_INTEGER16:
        value = (uint64_t)*(const int16_t*)mapping->storage;
        break;
    case GMP_CANOPEN_OD_UNSIGNED16:
        value = *(const uint16_t*)mapping->storage;
        break;
    case GMP_CANOPEN_OD_INTEGER32:
        value = (uint64_t)*(const int32_t*)mapping->storage;
        break;
    case GMP_CANOPEN_OD_UNSIGNED32:
        value = *(const uint32_t*)mapping->storage;
        break;
    case GMP_CANOPEN_OD_INTEGER64:
        value = (uint64_t)*(const int64_t*)mapping->storage;
        break;
    case GMP_CANOPEN_OD_UNSIGNED64:
        value = *(const uint64_t*)mapping->storage;
        break;
    case GMP_CANOPEN_OD_REAL32:
    {
        uint32_t bits;
        memcpy(&bits, mapping->storage, sizeof(bits));
        value = bits;
        break;
    }
    case GMP_CANOPEN_OD_REAL64:
        memcpy(&value, mapping->storage, sizeof(value));
        break;
    case GMP_CANOPEN_OD_VISIBLE_STRING:
    case GMP_CANOPEN_OD_OCTET_STRING:
    case GMP_CANOPEN_OD_DOMAIN:
        for (index = 0U; index < mapping->byte_length; ++index)
        {
            value = ((const byte_gt*)mapping->storage)[index];
            if (value > 0xFFU)
                return 0;
            output[index] = (byte_gt)value;
        }
        return 1;
    default:
        return 0;
    }
    gmp_canopen_pdo_store_unsigned(output, mapping->byte_length, value);
    return 1;
}

static fast_gt gmp_canopen_pdo_decode_mapping(
    const gmp_canopen_pdo_mapping_t* mapping, const byte_gt* input)
{
    uint64_t value;
    uint16_t index;
    for (index = 0U; index < mapping->byte_length; ++index)
        if (input[index] > 0xFFU)
            return 0;
    value = gmp_canopen_pdo_load_unsigned(input, mapping->byte_length);
    switch (mapping->data_type)
    {
    case GMP_CANOPEN_OD_BOOLEAN:
        *(fast_gt*)mapping->storage = value != 0U;
        break;
    case GMP_CANOPEN_OD_INTEGER8:
        *(int_least8_t*)mapping->storage =
            gmp_canopen_pdo_decode_integer8(value);
        break;
    case GMP_CANOPEN_OD_UNSIGNED8:
        *(byte_gt*)mapping->storage = (byte_gt)value;
        break;
    case GMP_CANOPEN_OD_INTEGER16:
        *(int16_t*)mapping->storage = (int16_t)value;
        break;
    case GMP_CANOPEN_OD_UNSIGNED16:
        *(uint16_t*)mapping->storage = (uint16_t)value;
        break;
    case GMP_CANOPEN_OD_INTEGER32:
        *(int32_t*)mapping->storage = (int32_t)value;
        break;
    case GMP_CANOPEN_OD_UNSIGNED32:
        *(uint32_t*)mapping->storage = (uint32_t)value;
        break;
    case GMP_CANOPEN_OD_INTEGER64:
        *(int64_t*)mapping->storage = (int64_t)value;
        break;
    case GMP_CANOPEN_OD_UNSIGNED64:
        *(uint64_t*)mapping->storage = value;
        break;
    case GMP_CANOPEN_OD_REAL32:
    {
        uint32_t bits = (uint32_t)value;
        memcpy(mapping->storage, &bits, sizeof(bits));
        break;
    }
    case GMP_CANOPEN_OD_REAL64:
        memcpy(mapping->storage, &value, sizeof(value));
        break;
    case GMP_CANOPEN_OD_VISIBLE_STRING:
    case GMP_CANOPEN_OD_OCTET_STRING:
    case GMP_CANOPEN_OD_DOMAIN:
        for (index = 0U; index < mapping->byte_length; ++index)
            ((byte_gt*)mapping->storage)[index] = input[index];
        break;
    default:
        return 0;
    }
    return 1;
}

static gmp_canopen_pdo_result_t gmp_canopen_pdo_compile_plan(
    gmp_canopen_pdo_plan_t* plan, gmp_canopen_od_t* dictionary,
    gmp_canopen_pdo_compile_direction_t direction, uint32_t key,
    uint16_t transmission_type, const uint32_t* descriptors,
    uint16_t descriptor_count, uint16_t payload_limit)
{
    uint16_t mapping_index;
    uint16_t offset = 0U;
    if (plan == NULL || dictionary == NULL || descriptors == NULL ||
        descriptor_count == 0U ||
        descriptor_count > GMP_CANOPEN_PDO_MAX_MAPPINGS || payload_limit == 0U)
        return GMP_CANOPEN_PDO_INVALID;
    plan->enabled = 0;
    plan->mapping_count = 0U;
    plan->payload_size = 0U;
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
        if ((uint32_t)offset + bytes > payload_limit)
            return GMP_CANOPEN_PDO_TOO_LARGE;
        entry = gmp_canopen_od_find(dictionary, index, subindex);
        if (entry == NULL || !(entry->access & GMP_CANOPEN_OD_ACCESS_PDO) ||
            entry->size != bytes ||
            (direction == GMP_CANOPEN_PDO_COMPILE_TX &&
             !(entry->access & GMP_CANOPEN_OD_ACCESS_READ)) ||
            (direction == GMP_CANOPEN_PDO_COMPILE_RX &&
             (!(entry->access & GMP_CANOPEN_OD_ACCESS_WRITE) ||
              (entry->access & GMP_CANOPEN_OD_ACCESS_CONST))))
            return GMP_CANOPEN_PDO_NOT_MAPPABLE;
        plan->mappings[mapping_index].storage =
            gmp_canopen_pdo_resolve_storage(entry);
        if (plan->mappings[mapping_index].storage == NULL)
            return GMP_CANOPEN_PDO_NOT_MAPPABLE;
        plan->mappings[mapping_index].data_type = entry->data_type;
        plan->mappings[mapping_index].byte_offset = offset;
        plan->mappings[mapping_index].byte_length = bytes;
        offset = (uint16_t)(offset + bytes);
    }
    plan->key = key;
    plan->transmission_type = transmission_type;
    plan->mapping_count = descriptor_count;
    plan->payload_size = offset;
    plan->enabled = 1;
    return GMP_CANOPEN_PDO_OK;
}

gmp_canopen_pdo_result_t gmp_canopen_txpdo_compile_buffer(
    gmp_canopen_txpdo_t* pdo, gmp_canopen_od_t* dictionary,
    uint32_t key, uint16_t transmission_type,
    const uint32_t* descriptors, uint16_t descriptor_count,
    uint16_t payload_limit)
{
    if (pdo == NULL)
        return GMP_CANOPEN_PDO_INVALID;
    return gmp_canopen_pdo_compile_plan(&pdo->plan, dictionary,
        GMP_CANOPEN_PDO_COMPILE_TX, key, transmission_type,
        descriptors, descriptor_count, payload_limit);
}

gmp_canopen_pdo_result_t gmp_canopen_rxpdo_compile_buffer(
    gmp_canopen_rxpdo_t* pdo, gmp_canopen_od_t* dictionary,
    uint32_t key, uint16_t transmission_type,
    const uint32_t* descriptors, uint16_t descriptor_count,
    uint16_t payload_limit)
{
    if (pdo == NULL)
        return GMP_CANOPEN_PDO_INVALID;
    return gmp_canopen_pdo_compile_plan(&pdo->plan, dictionary,
        GMP_CANOPEN_PDO_COMPILE_RX, key, transmission_type,
        descriptors, descriptor_count, payload_limit);
}

gmp_canopen_pdo_result_t gmp_canopen_txpdo_compile(
    gmp_canopen_txpdo_t* pdo, gmp_canopen_od_t* dictionary,
    uint32_t cob_id, uint16_t transmission_type,
    const uint32_t* descriptors, uint16_t descriptor_count)
{
    if (cob_id > 0x7FFU)
        return GMP_CANOPEN_PDO_INVALID;
    return gmp_canopen_txpdo_compile_buffer(pdo, dictionary, cob_id,
        transmission_type, descriptors, descriptor_count,
        GMP_CANOPEN_CLASSIC_MAX_DATA);
}

gmp_canopen_pdo_result_t gmp_canopen_rxpdo_compile(
    gmp_canopen_rxpdo_t* pdo, gmp_canopen_od_t* dictionary,
    uint32_t cob_id, uint16_t transmission_type,
    const uint32_t* descriptors, uint16_t descriptor_count)
{
    if (cob_id > 0x7FFU)
        return GMP_CANOPEN_PDO_INVALID;
    return gmp_canopen_rxpdo_compile_buffer(pdo, dictionary, cob_id,
        transmission_type, descriptors, descriptor_count,
        GMP_CANOPEN_CLASSIC_MAX_DATA);
}

gmp_canopen_pdo_result_t gmp_canopen_txpdo_pack_fast(
    const gmp_canopen_txpdo_t* pdo, gmp_canopen_nmt_state_t nmt_state,
    byte_gt* output, uint16_t capacity, uint16_t* actual_size)
{
    uint16_t mapping_index;
    if (pdo == NULL || output == NULL || actual_size == NULL ||
        !pdo->plan.enabled)
        return GMP_CANOPEN_PDO_INVALID;
    if (nmt_state != GMP_CANOPEN_NMT_OPERATIONAL)
        return GMP_CANOPEN_PDO_NOT_OPERATIONAL;
    if (capacity < pdo->plan.payload_size)
        return GMP_CANOPEN_PDO_TOO_LARGE;
    for (mapping_index = 0U; mapping_index < pdo->plan.mapping_count;
         ++mapping_index)
    {
        const gmp_canopen_pdo_mapping_t* mapping =
            &pdo->plan.mappings[mapping_index];
        if (!gmp_canopen_pdo_encode_mapping(
                mapping, &output[mapping->byte_offset]))
            return GMP_CANOPEN_PDO_IO_ERROR;
    }
    *actual_size = pdo->plan.payload_size;
    return GMP_CANOPEN_PDO_OK;
}

gmp_canopen_pdo_result_t gmp_canopen_rxpdo_unpack_fast(
    const gmp_canopen_rxpdo_t* pdo, gmp_canopen_nmt_state_t nmt_state,
    const byte_gt* input, uint16_t size)
{
    uint16_t mapping_index;
    if (pdo == NULL || input == NULL || !pdo->plan.enabled ||
        size != pdo->plan.payload_size)
        return GMP_CANOPEN_PDO_INVALID;
    if (nmt_state != GMP_CANOPEN_NMT_OPERATIONAL)
        return GMP_CANOPEN_PDO_NOT_OPERATIONAL;
    for (mapping_index = 0U; mapping_index < pdo->plan.mapping_count;
         ++mapping_index)
    {
        const gmp_canopen_pdo_mapping_t* mapping =
            &pdo->plan.mappings[mapping_index];
        if (!gmp_canopen_pdo_decode_mapping(
                mapping, &input[mapping->byte_offset]))
            return GMP_CANOPEN_PDO_IO_ERROR;
    }
    return GMP_CANOPEN_PDO_OK;
}

gmp_canopen_pdo_result_t gmp_canopen_txpdo_build_frame_fast(
    const gmp_canopen_txpdo_t* pdo, gmp_canopen_nmt_state_t nmt_state,
    gmp_canopen_frame_t* frame)
{
    uint16_t size;
    gmp_canopen_pdo_result_t result;
    if (pdo == NULL || frame == NULL || pdo->plan.key > 0x7FFU ||
        pdo->plan.payload_size > GMP_CANOPEN_CLASSIC_MAX_DATA)
        return GMP_CANOPEN_PDO_INVALID;
    gmp_canopen_frame_clear(frame);
    result = gmp_canopen_txpdo_pack_fast(pdo, nmt_state, frame->data,
        GMP_CANOPEN_CLASSIC_MAX_DATA, &size);
    if (result != GMP_CANOPEN_PDO_OK)
        return result;
    frame->id = pdo->plan.key;
    frame->dlc = size;
    return GMP_CANOPEN_PDO_OK;
}

gmp_canopen_pdo_result_t gmp_canopen_rxpdo_apply_frame_fast(
    const gmp_canopen_rxpdo_t* pdo, gmp_canopen_nmt_state_t nmt_state,
    const gmp_canopen_frame_t* frame)
{
    if (pdo == NULL || frame == NULL || !gmp_canopen_frame_validate(frame) ||
        frame->is_extended || frame->is_remote ||
        frame->id != pdo->plan.key)
        return GMP_CANOPEN_PDO_INVALID;
    return gmp_canopen_rxpdo_unpack_fast(pdo, nmt_state,
        frame->data, frame->dlc);
}

void gmp_canopen_txpdo_group_init(gmp_canopen_txpdo_group_t* group)
{
    uint16_t index;
    if (group == NULL)
        return;
    group->count = 0U;
    for (index = 0U; index < GMP_CANOPEN_PDO_GROUP_CAPACITY; ++index)
        group->items[index] = NULL;
}

void gmp_canopen_rxpdo_group_init(gmp_canopen_rxpdo_group_t* group)
{
    uint16_t index;
    if (group == NULL)
        return;
    group->count = 0U;
    for (index = 0U; index < GMP_CANOPEN_PDO_GROUP_CAPACITY; ++index)
        group->items[index] = NULL;
}

gmp_canopen_pdo_result_t gmp_canopen_txpdo_group_add(
    gmp_canopen_txpdo_group_t* group, gmp_canopen_txpdo_t* pdo)
{
    uint16_t position;
    if (group == NULL || pdo == NULL || !pdo->plan.enabled)
        return GMP_CANOPEN_PDO_INVALID;
    if (group->count >= GMP_CANOPEN_PDO_GROUP_CAPACITY)
        return GMP_CANOPEN_PDO_GROUP_FULL;
    position = 0U;
    while (position < group->count &&
           group->items[position]->plan.key < pdo->plan.key)
        ++position;
    if (position < group->count &&
        group->items[position]->plan.key == pdo->plan.key)
        return GMP_CANOPEN_PDO_DUPLICATE;
    {
        uint16_t index = group->count;
        while (index > position)
        {
            group->items[index] = group->items[index - 1U];
            --index;
        }
    }
    group->items[position] = pdo;
    ++group->count;
    return GMP_CANOPEN_PDO_OK;
}

gmp_canopen_pdo_result_t gmp_canopen_rxpdo_group_add(
    gmp_canopen_rxpdo_group_t* group, gmp_canopen_rxpdo_t* pdo)
{
    uint16_t position;
    if (group == NULL || pdo == NULL || !pdo->plan.enabled)
        return GMP_CANOPEN_PDO_INVALID;
    if (group->count >= GMP_CANOPEN_PDO_GROUP_CAPACITY)
        return GMP_CANOPEN_PDO_GROUP_FULL;
    position = 0U;
    while (position < group->count &&
           group->items[position]->plan.key < pdo->plan.key)
        ++position;
    if (position < group->count &&
        group->items[position]->plan.key == pdo->plan.key)
        return GMP_CANOPEN_PDO_DUPLICATE;
    {
        uint16_t index = group->count;
        while (index > position)
        {
            group->items[index] = group->items[index - 1U];
            --index;
        }
    }
    group->items[position] = pdo;
    ++group->count;
    return GMP_CANOPEN_PDO_OK;
}

gmp_canopen_txpdo_t* gmp_canopen_txpdo_group_find(
    const gmp_canopen_txpdo_group_t* group, uint32_t key)
{
    uint16_t first = 0U;
    uint16_t last;
    if (group == NULL)
        return NULL;
    last = group->count;
    while (first < last)
    {
        uint16_t middle = (uint16_t)(first + (last - first) / 2U);
        uint32_t present = group->items[middle]->plan.key;
        if (present == key)
            return group->items[middle];
        if (present < key)
            first = (uint16_t)(middle + 1U);
        else
            last = middle;
    }
    return NULL;
}

gmp_canopen_rxpdo_t* gmp_canopen_rxpdo_group_find(
    const gmp_canopen_rxpdo_group_t* group, uint32_t key)
{
    uint16_t first = 0U;
    uint16_t last;
    if (group == NULL)
        return NULL;
    last = group->count;
    while (first < last)
    {
        uint16_t middle = (uint16_t)(first + (last - first) / 2U);
        uint32_t present = group->items[middle]->plan.key;
        if (present == key)
            return group->items[middle];
        if (present < key)
            first = (uint16_t)(middle + 1U);
        else
            last = middle;
    }
    return NULL;
}

gmp_canopen_pdo_result_t gmp_canopen_txpdo_group_build_at_fast(
    const gmp_canopen_txpdo_group_t* group, uint16_t slot,
    gmp_canopen_nmt_state_t nmt_state, gmp_canopen_frame_t* frame)
{
    if (group == NULL || slot >= group->count)
        return GMP_CANOPEN_PDO_NOT_FOUND;
    return gmp_canopen_txpdo_build_frame_fast(
        group->items[slot], nmt_state, frame);
}

gmp_canopen_pdo_result_t gmp_canopen_rxpdo_group_dispatch_fast(
    const gmp_canopen_rxpdo_group_t* group,
    gmp_canopen_nmt_state_t nmt_state, const gmp_canopen_frame_t* frame)
{
    gmp_canopen_rxpdo_t* pdo;
    if (group == NULL || frame == NULL)
        return GMP_CANOPEN_PDO_INVALID;
    pdo = gmp_canopen_rxpdo_group_find(group, frame->id);
    if (pdo == NULL)
        return GMP_CANOPEN_PDO_NOT_FOUND;
    return gmp_canopen_rxpdo_apply_frame_fast(pdo, nmt_state, frame);
}
