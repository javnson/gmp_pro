/** @file od.c @brief CANopen object dictionary implementation. */

#include <core/protocol/canopen/od.h>

#include <string.h>

typedef struct
{
    uint16_t index;
    uint16_t subindex;
} gmp_canopen_od_key_t;

static int gmp_canopen_od_compare_nodes(const gmp_rb_node* lhs,
                                        const gmp_rb_node* rhs,
                                        void* context)
{
    const gmp_canopen_od_entry_t* lhs_entry =
        GMP_CONTAINER_OF(lhs, gmp_canopen_od_entry_t, rb_node);
    const gmp_canopen_od_entry_t* rhs_entry =
        GMP_CONTAINER_OF(rhs, gmp_canopen_od_entry_t, rb_node);
    GMP_UNUSED_VAR(context);
    if (lhs_entry->index != rhs_entry->index)
        return lhs_entry->index < rhs_entry->index ? -1 : 1;
    if (lhs_entry->subindex != rhs_entry->subindex)
        return lhs_entry->subindex < rhs_entry->subindex ? -1 : 1;
    return 0;
}

static int gmp_canopen_od_compare_key(const void* key,
                                      const gmp_rb_node* node,
                                      void* context)
{
    const gmp_canopen_od_key_t* od_key = (const gmp_canopen_od_key_t*)key;
    const gmp_canopen_od_entry_t* entry =
        GMP_CONTAINER_OF(node, gmp_canopen_od_entry_t, rb_node);
    GMP_UNUSED_VAR(context);
    if (od_key->index != entry->index)
        return od_key->index < entry->index ? -1 : 1;
    if (od_key->subindex != entry->subindex)
        return od_key->subindex < entry->subindex ? -1 : 1;
    return 0;
}

uint32_t gmp_canopen_od_type_size(gmp_canopen_od_data_type_t data_type)
{
    switch (data_type)
    {
    case GMP_CANOPEN_OD_BOOLEAN:
    case GMP_CANOPEN_OD_INTEGER8:
    case GMP_CANOPEN_OD_UNSIGNED8:
        return 1U;
    case GMP_CANOPEN_OD_INTEGER16:
    case GMP_CANOPEN_OD_UNSIGNED16:
        return 2U;
    case GMP_CANOPEN_OD_INTEGER32:
    case GMP_CANOPEN_OD_UNSIGNED32:
    case GMP_CANOPEN_OD_REAL32:
        return 4U;
    case GMP_CANOPEN_OD_INTEGER64:
    case GMP_CANOPEN_OD_UNSIGNED64:
    case GMP_CANOPEN_OD_REAL64:
        return 8U;
    default:
        return 0U;
    }
}

void gmp_canopen_od_init(gmp_canopen_od_t* dictionary)
{
    if (dictionary != NULL)
        gmp_rb_root_init(&dictionary->entries);
}

static void gmp_canopen_od_entry_init(gmp_canopen_od_entry_t* entry,
                                      uint16_t index, uint16_t subindex,
                                      gmp_canopen_od_data_type_t data_type,
                                      uint16_t access, uint32_t size,
                                      const char* name)
{
    uint32_t scalar_size;
    if (entry == NULL)
        return;
    memset(entry, 0, sizeof(*entry));
    gmp_rb_node_init(&entry->rb_node);
    entry->index = index;
    entry->subindex = subindex;
    entry->data_type = data_type;
    entry->access = access;
    scalar_size = gmp_canopen_od_type_size(data_type);
    entry->size = size == 0U ? scalar_size : size;
    entry->name = name;
}

void gmp_canopen_od_entry_init_value(gmp_canopen_od_entry_t* entry,
                                     uint16_t index, uint16_t subindex,
                                     gmp_canopen_od_data_type_t data_type,
                                     uint16_t access, uint32_t size,
                                     const char* name)
{
    gmp_canopen_od_entry_init(entry, index, subindex, data_type,
                              access, size, name);
    if (entry != NULL)
        entry->storage_mode = GMP_CANOPEN_OD_STORAGE_VALUE;
}

void gmp_canopen_od_entry_init_pointer(gmp_canopen_od_entry_t* entry,
                                       uint16_t index, uint16_t subindex,
                                       gmp_canopen_od_data_type_t data_type,
                                       uint16_t access, void* value,
                                       uint32_t size, const char* name)
{
    gmp_canopen_od_entry_init(entry, index, subindex, data_type,
                              access, size, name);
    if (entry != NULL)
    {
        entry->storage_mode = GMP_CANOPEN_OD_STORAGE_POINTER;
        entry->storage.pointer.raw = value;
    }
}

gmp_canopen_od_result_t gmp_canopen_od_insert(
    gmp_canopen_od_t* dictionary, gmp_canopen_od_entry_t* entry)
{
    uint32_t scalar_size;
    if (dictionary == NULL || entry == NULL || entry->subindex > 0xFFU ||
        entry->size == 0U ||
        (entry->storage_mode == GMP_CANOPEN_OD_STORAGE_POINTER &&
         entry->storage.pointer.raw == NULL))
        return GMP_CANOPEN_OD_INVALID;
    scalar_size = gmp_canopen_od_type_size(entry->data_type);
    if (scalar_size != 0U && scalar_size != entry->size)
        return GMP_CANOPEN_OD_TYPE_MISMATCH;
    if (entry->storage_mode == GMP_CANOPEN_OD_STORAGE_VALUE && entry->size > 8U)
        return GMP_CANOPEN_OD_LENGTH_MISMATCH;
    if (!gmp_rb_insert(&dictionary->entries, &entry->rb_node,
                       gmp_canopen_od_compare_nodes, NULL))
        return GMP_CANOPEN_OD_DUPLICATE;
    return GMP_CANOPEN_OD_OK;
}

gmp_canopen_od_result_t gmp_canopen_od_remove(
    gmp_canopen_od_t* dictionary, gmp_canopen_od_entry_t* entry)
{
    if (dictionary == NULL || entry == NULL ||
        !gmp_rb_remove(&dictionary->entries, &entry->rb_node))
        return GMP_CANOPEN_OD_NOT_FOUND;
    return GMP_CANOPEN_OD_OK;
}

gmp_canopen_od_entry_t* gmp_canopen_od_find(const gmp_canopen_od_t* dictionary,
                                            uint16_t index,
                                            uint16_t subindex)
{
    gmp_canopen_od_key_t key;
    gmp_rb_node* node;
    if (dictionary == NULL)
        return NULL;
    key.index = index;
    key.subindex = subindex;
    node = gmp_rb_find(&dictionary->entries, &key,
                       gmp_canopen_od_compare_key, NULL);
    return node == NULL ? NULL :
        GMP_CONTAINER_OF(node, gmp_canopen_od_entry_t, rb_node);
}

gmp_canopen_od_entry_t* gmp_canopen_od_first(const gmp_canopen_od_t* dictionary)
{
    gmp_rb_node* node = dictionary == NULL ? NULL :
        gmp_rb_minimum(dictionary->entries.root);
    return node == NULL ? NULL :
        GMP_CONTAINER_OF(node, gmp_canopen_od_entry_t, rb_node);
}

gmp_canopen_od_entry_t* gmp_canopen_od_next(const gmp_canopen_od_entry_t* entry)
{
    gmp_rb_node* node = entry == NULL ? NULL :
        gmp_rb_next((gmp_rb_node*)&entry->rb_node);
    return node == NULL ? NULL :
        GMP_CONTAINER_OF(node, gmp_canopen_od_entry_t, rb_node);
}

static uint64_t gmp_canopen_od_get_unsigned(const gmp_canopen_od_entry_t* entry)
{
    if (entry->storage_mode == GMP_CANOPEN_OD_STORAGE_VALUE)
    {
        switch (entry->data_type)
        {
        case GMP_CANOPEN_OD_BOOLEAN: return entry->storage.value.boolean != 0;
        case GMP_CANOPEN_OD_INTEGER8: return (uint64_t)entry->storage.value.i8;
        case GMP_CANOPEN_OD_UNSIGNED8: return entry->storage.value.u8;
        case GMP_CANOPEN_OD_INTEGER16: return (uint64_t)entry->storage.value.i16;
        case GMP_CANOPEN_OD_UNSIGNED16: return entry->storage.value.u16;
        case GMP_CANOPEN_OD_INTEGER32: return (uint64_t)entry->storage.value.i32;
        case GMP_CANOPEN_OD_UNSIGNED32: return entry->storage.value.u32;
        case GMP_CANOPEN_OD_INTEGER64: return (uint64_t)entry->storage.value.i64;
        case GMP_CANOPEN_OD_UNSIGNED64: return entry->storage.value.u64;
        default: return 0U;
        }
    }
    switch (entry->data_type)
    {
    case GMP_CANOPEN_OD_BOOLEAN: return *entry->storage.pointer.boolean != 0;
    case GMP_CANOPEN_OD_INTEGER8: return (uint64_t)*entry->storage.pointer.i8;
    case GMP_CANOPEN_OD_UNSIGNED8: return *entry->storage.pointer.u8;
    case GMP_CANOPEN_OD_INTEGER16: return (uint64_t)*entry->storage.pointer.i16;
    case GMP_CANOPEN_OD_UNSIGNED16: return *entry->storage.pointer.u16;
    case GMP_CANOPEN_OD_INTEGER32: return (uint64_t)*entry->storage.pointer.i32;
    case GMP_CANOPEN_OD_UNSIGNED32: return *entry->storage.pointer.u32;
    case GMP_CANOPEN_OD_INTEGER64: return (uint64_t)*entry->storage.pointer.i64;
    case GMP_CANOPEN_OD_UNSIGNED64: return *entry->storage.pointer.u64;
    default: return 0U;
    }
}

static fast_gt gmp_canopen_od_narrow_scalar_valid(
    const gmp_canopen_od_entry_t* entry)
{
    if (entry->data_type == GMP_CANOPEN_OD_INTEGER8)
    {
        int_least8_t value = entry->storage_mode == GMP_CANOPEN_OD_STORAGE_VALUE ?
            entry->storage.value.i8 : *entry->storage.pointer.i8;
        return value >= -128 && value <= 127;
    }
    if (entry->data_type == GMP_CANOPEN_OD_UNSIGNED8)
    {
        uint_least8_t value = entry->storage_mode == GMP_CANOPEN_OD_STORAGE_VALUE ?
            entry->storage.value.u8 : *entry->storage.pointer.u8;
        return value <= 0xFFU;
    }
    return 1;
}

static int_least8_t gmp_canopen_od_decode_integer8(uint64_t value)
{
    uint16_t octet = (uint16_t)(value & 0xFFU);
    return octet < 0x80U ? (int_least8_t)octet :
        (int_least8_t)((int16_t)octet - 0x100);
}

static void gmp_canopen_od_set_unsigned(gmp_canopen_od_entry_t* entry,
                                        uint64_t value)
{
    if (entry->storage_mode == GMP_CANOPEN_OD_STORAGE_VALUE)
    {
        switch (entry->data_type)
        {
        case GMP_CANOPEN_OD_BOOLEAN: entry->storage.value.boolean = value != 0U; break;
        case GMP_CANOPEN_OD_INTEGER8: entry->storage.value.i8 = gmp_canopen_od_decode_integer8(value); break;
        case GMP_CANOPEN_OD_UNSIGNED8: entry->storage.value.u8 = (uint_least8_t)value; break;
        case GMP_CANOPEN_OD_INTEGER16: entry->storage.value.i16 = (int16_t)value; break;
        case GMP_CANOPEN_OD_UNSIGNED16: entry->storage.value.u16 = (uint16_t)value; break;
        case GMP_CANOPEN_OD_INTEGER32: entry->storage.value.i32 = (int32_t)value; break;
        case GMP_CANOPEN_OD_UNSIGNED32: entry->storage.value.u32 = (uint32_t)value; break;
        case GMP_CANOPEN_OD_INTEGER64: entry->storage.value.i64 = (int64_t)value; break;
        case GMP_CANOPEN_OD_UNSIGNED64: entry->storage.value.u64 = value; break;
        default: break;
        }
        return;
    }
    switch (entry->data_type)
    {
    case GMP_CANOPEN_OD_BOOLEAN: *entry->storage.pointer.boolean = value != 0U; break;
    case GMP_CANOPEN_OD_INTEGER8: *entry->storage.pointer.i8 = gmp_canopen_od_decode_integer8(value); break;
    case GMP_CANOPEN_OD_UNSIGNED8: *entry->storage.pointer.u8 = (uint_least8_t)value; break;
    case GMP_CANOPEN_OD_INTEGER16: *entry->storage.pointer.i16 = (int16_t)value; break;
    case GMP_CANOPEN_OD_UNSIGNED16: *entry->storage.pointer.u16 = (uint16_t)value; break;
    case GMP_CANOPEN_OD_INTEGER32: *entry->storage.pointer.i32 = (int32_t)value; break;
    case GMP_CANOPEN_OD_UNSIGNED32: *entry->storage.pointer.u32 = (uint32_t)value; break;
    case GMP_CANOPEN_OD_INTEGER64: *entry->storage.pointer.i64 = (int64_t)value; break;
    case GMP_CANOPEN_OD_UNSIGNED64: *entry->storage.pointer.u64 = value; break;
    default: break;
    }
}

gmp_canopen_od_result_t gmp_canopen_od_read(
    const gmp_canopen_od_entry_t* entry, uint16_t* output,
    uint32_t capacity, uint32_t* actual_size)
{
    uint32_t index;
    uint64_t value;
    const uint16_t* raw;
    if (entry == NULL || output == NULL || actual_size == NULL)
        return GMP_CANOPEN_OD_INVALID;
    if (!(entry->access & GMP_CANOPEN_OD_ACCESS_READ))
        return GMP_CANOPEN_OD_READ_DENIED;
    if (capacity < entry->size)
        return GMP_CANOPEN_OD_LENGTH_MISMATCH;
    if (!gmp_canopen_od_narrow_scalar_valid(entry))
        return GMP_CANOPEN_OD_TYPE_MISMATCH;
    *actual_size = entry->size;
    if (entry->data_type == GMP_CANOPEN_OD_REAL32)
    {
        uint32_t bits;
        float real = entry->storage_mode == GMP_CANOPEN_OD_STORAGE_VALUE ?
            entry->storage.value.real32 : *entry->storage.pointer.real32;
        memcpy(&bits, &real, sizeof(bits));
        gmp_canopen_store_le32(output, bits);
        return GMP_CANOPEN_OD_OK;
    }
    if (entry->data_type == GMP_CANOPEN_OD_REAL64)
    {
        double real = entry->storage_mode == GMP_CANOPEN_OD_STORAGE_VALUE ?
            entry->storage.value.real64 : *entry->storage.pointer.real64;
        memcpy(&value, &real, sizeof(value));
    }
    else if (gmp_canopen_od_type_size(entry->data_type) != 0U)
    {
        value = gmp_canopen_od_get_unsigned(entry);
    }
    else
    {
        raw = entry->storage_mode == GMP_CANOPEN_OD_STORAGE_VALUE ?
            entry->storage.value.octets : entry->storage.pointer.octets;
        for (index = 0U; index < entry->size; ++index)
            output[index] = raw[index] & 0xFFU;
        return GMP_CANOPEN_OD_OK;
    }
    for (index = 0U; index < entry->size; ++index)
        output[index] = (uint16_t)((value >> (8U * index)) & 0xFFU);
    return GMP_CANOPEN_OD_OK;
}

gmp_canopen_od_result_t gmp_canopen_od_write(
    gmp_canopen_od_entry_t* entry, const uint16_t* input,
    uint32_t size)
{
    uint32_t index;
    uint64_t value = 0U;
    uint16_t* raw;
    if (entry == NULL || input == NULL)
        return GMP_CANOPEN_OD_INVALID;
    if (!(entry->access & GMP_CANOPEN_OD_ACCESS_WRITE) ||
        (entry->access & GMP_CANOPEN_OD_ACCESS_CONST))
        return GMP_CANOPEN_OD_WRITE_DENIED;
    if (size != entry->size)
        return GMP_CANOPEN_OD_LENGTH_MISMATCH;
    for (index = 0U; index < size; ++index)
        if (input[index] > 0xFFU)
            return GMP_CANOPEN_OD_INVALID;
    if (entry->data_type == GMP_CANOPEN_OD_VISIBLE_STRING ||
        entry->data_type == GMP_CANOPEN_OD_OCTET_STRING ||
        entry->data_type == GMP_CANOPEN_OD_DOMAIN)
    {
        raw = entry->storage_mode == GMP_CANOPEN_OD_STORAGE_VALUE ?
            entry->storage.value.octets : entry->storage.pointer.octets;
        for (index = 0U; index < size; ++index)
            raw[index] = input[index];
        return GMP_CANOPEN_OD_OK;
    }
    for (index = 0U; index < size; ++index)
        value |= (uint64_t)(input[index] & 0xFFU) << (8U * index);
    if (entry->data_type == GMP_CANOPEN_OD_REAL32)
    {
        uint32_t bits = (uint32_t)value;
        if (entry->storage_mode == GMP_CANOPEN_OD_STORAGE_VALUE)
            memcpy(&entry->storage.value.real32, &bits, sizeof(bits));
        else
            memcpy(entry->storage.pointer.real32, &bits, sizeof(bits));
    }
    else if (entry->data_type == GMP_CANOPEN_OD_REAL64)
    {
        if (entry->storage_mode == GMP_CANOPEN_OD_STORAGE_VALUE)
            memcpy(&entry->storage.value.real64, &value, sizeof(value));
        else
            memcpy(entry->storage.pointer.real64, &value, sizeof(value));
    }
    else
    {
        gmp_canopen_od_set_unsigned(entry, value);
    }
    return GMP_CANOPEN_OD_OK;
}

fast_gt gmp_canopen_od_validate(const gmp_canopen_od_t* dictionary,
                                size_gt max_entries)
{
    if (dictionary == NULL)
        return 0;
    return gmp_rb_validate(&dictionary->entries, gmp_canopen_od_compare_nodes,
                           NULL, max_entries);
}
