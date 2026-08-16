/** @file ethercat_if.c @brief Normalized CoE services over the CANopen OD/PDO core. */

#include <core/protocol/canopen/ethercat_if.h>

static uint32_t gmp_coe_abort_for_od(gmp_canopen_od_result_t result,
                                     fast_gt writing)
{
    switch (result)
    {
    case GMP_CANOPEN_OD_NOT_FOUND:
        return GMP_CANOPEN_SDO_ABORT_NOT_FOUND;
    case GMP_CANOPEN_OD_READ_DENIED:
        return GMP_CANOPEN_SDO_ABORT_UNSUPPORTED;
    case GMP_CANOPEN_OD_WRITE_DENIED:
        return writing ? GMP_CANOPEN_SDO_ABORT_READ_ONLY :
                         GMP_CANOPEN_SDO_ABORT_UNSUPPORTED;
    case GMP_CANOPEN_OD_TYPE_MISMATCH:
    case GMP_CANOPEN_OD_LENGTH_MISMATCH:
        return GMP_CANOPEN_SDO_ABORT_TYPE_LENGTH;
    default:
        return GMP_CANOPEN_SDO_ABORT_GENERAL;
    }
}

fast_gt gmp_coe_server_init(gmp_coe_server_t* server,
                            gmp_canopen_od_t* dictionary)
{
    if (server == NULL || dictionary == NULL)
        return 0;
    server->dictionary = dictionary;
    return 1;
}

gmp_coe_result_t gmp_coe_sdo_server_process(
    gmp_coe_server_t* server, const gmp_coe_sdo_request_t* request,
    gmp_coe_sdo_response_t* response)
{
    gmp_canopen_od_entry_t* entry;
    gmp_canopen_od_result_t od_result;
    if (response != NULL)
    {
        response->result = GMP_COE_INVALID;
        response->abort_code = 0U;
        response->data_size = 0U;
    }
    if (server == NULL || server->dictionary == NULL || request == NULL ||
        response == NULL || request->subindex > 0xFFU)
        return GMP_COE_INVALID;
    response->number = request->number;
    if (request->complete_access)
    {
        response->result = GMP_COE_UNSUPPORTED;
        response->abort_code = GMP_CANOPEN_SDO_ABORT_UNSUPPORTED;
        return response->result;
    }
    entry = gmp_canopen_od_find(server->dictionary,
                                request->index, request->subindex);
    if (entry == NULL)
    {
        response->result = GMP_COE_OD_ABORT;
        response->abort_code = GMP_CANOPEN_SDO_ABORT_NOT_FOUND;
        return response->result;
    }
    if (request->operation == GMP_COE_SDO_UPLOAD)
    {
        if (response->data == NULL || response->capacity < entry->size)
        {
            response->result = GMP_COE_BUFFER_TOO_SMALL;
            return response->result;
        }
        od_result = gmp_canopen_od_read(entry, response->data,
                                        response->capacity,
                                        &response->data_size);
    }
    else if (request->operation == GMP_COE_SDO_DOWNLOAD)
    {
        if (request->data == NULL)
            return GMP_COE_INVALID;
        od_result = gmp_canopen_od_write(entry, request->data,
                                         request->data_size);
    }
    else
    {
        return GMP_COE_INVALID;
    }
    if (od_result != GMP_CANOPEN_OD_OK)
    {
        response->result = GMP_COE_OD_ABORT;
        response->abort_code = gmp_coe_abort_for_od(
            od_result, request->operation == GMP_COE_SDO_DOWNLOAD);
        return response->result;
    }
    response->result = GMP_COE_OK;
    return response->result;
}

gmp_canopen_pdo_result_t gmp_coe_txpdo_compile(
    gmp_canopen_txpdo_t* pdo, gmp_canopen_od_t* dictionary,
    uint32_t process_image_key, const uint32_t* descriptors,
    uint16_t descriptor_count, uint16_t payload_limit)
{
    return gmp_canopen_txpdo_compile_buffer(pdo, dictionary,
        process_image_key, 0U, descriptors, descriptor_count, payload_limit);
}

gmp_canopen_pdo_result_t gmp_coe_rxpdo_compile(
    gmp_canopen_rxpdo_t* pdo, gmp_canopen_od_t* dictionary,
    uint32_t process_image_key, const uint32_t* descriptors,
    uint16_t descriptor_count, uint16_t payload_limit)
{
    return gmp_canopen_rxpdo_compile_buffer(pdo, dictionary,
        process_image_key, 0U, descriptors, descriptor_count, payload_limit);
}

gmp_canopen_pdo_result_t gmp_coe_txpdo_pack_fast(
    const gmp_canopen_txpdo_t* pdo, uint16_t* output,
    uint16_t capacity, uint16_t* actual_size)
{
    return gmp_canopen_txpdo_pack_fast(pdo,
        GMP_CANOPEN_NMT_OPERATIONAL, output, capacity, actual_size);
}

gmp_canopen_pdo_result_t gmp_coe_rxpdo_unpack_fast(
    const gmp_canopen_rxpdo_t* pdo, const uint16_t* input, uint16_t size)
{
    return gmp_canopen_rxpdo_unpack_fast(pdo,
        GMP_CANOPEN_NMT_OPERATIONAL, input, size);
}
