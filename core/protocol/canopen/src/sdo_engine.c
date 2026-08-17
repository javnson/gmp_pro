/** @file sdo_engine.c @brief CANopen expedited and segmented SDO server. */

#include <core/protocol/canopen/sdo_engine.h>

static void gmp_canopen_sdo_response_begin(const gmp_canopen_sdo_server_t* server, gmp_canopen_frame_t* response)
{
    gmp_canopen_frame_clear(response);
    response->id = GMP_CANOPEN_COB_TSDO + server->node_id;
    response->dlc = 8U;
}

static void gmp_canopen_sdo_abort(gmp_canopen_sdo_server_t* server, gmp_canopen_frame_t* response, uint16_t index,
                                  uint16_t subindex, uint32_t abort_code)
{
    gmp_canopen_sdo_response_begin(server, response);
    response->data[0] = 0x80U;
    gmp_canopen_store_le16(&response->data[1], index);
    response->data[3] = (byte_gt)subindex;
    gmp_canopen_store_le32(&response->data[4], abort_code);
    gmp_canopen_sdo_server_reset(server);
}

static uint32_t gmp_canopen_sdo_abort_for_od(gmp_canopen_od_result_t result, fast_gt writing)
{
    switch (result)
    {
    case GMP_CANOPEN_OD_NOT_FOUND:
        return GMP_CANOPEN_SDO_ABORT_NOT_FOUND;
    case GMP_CANOPEN_OD_READ_DENIED:
        return GMP_CANOPEN_SDO_ABORT_UNSUPPORTED;
    case GMP_CANOPEN_OD_WRITE_DENIED:
        return writing ? GMP_CANOPEN_SDO_ABORT_READ_ONLY : GMP_CANOPEN_SDO_ABORT_UNSUPPORTED;
    case GMP_CANOPEN_OD_TYPE_MISMATCH:
    case GMP_CANOPEN_OD_LENGTH_MISMATCH:
        return GMP_CANOPEN_SDO_ABORT_TYPE_LENGTH;
    default:
        return GMP_CANOPEN_SDO_ABORT_GENERAL;
    }
}

fast_gt gmp_canopen_sdo_server_init(gmp_canopen_sdo_server_t* server, uint16_t node_id, gmp_canopen_od_t* dictionary)
{
    if (server == NULL || dictionary == NULL || node_id < GMP_CANOPEN_NODE_ID_MIN || node_id > GMP_CANOPEN_NODE_ID_MAX)
        return 0;
    server->node_id = node_id;
    server->dictionary = dictionary;
    gmp_canopen_sdo_server_reset(server);
    return 1;
}

void gmp_canopen_sdo_server_reset(gmp_canopen_sdo_server_t* server)
{
    if (server == NULL)
        return;
    server->state = GMP_CANOPEN_SDO_IDLE;
    server->entry = NULL;
    server->offset = 0U;
    server->total_size = 0U;
    server->toggle = 0U;
}

static fast_gt gmp_canopen_sdo_initiate_download(gmp_canopen_sdo_server_t* server, const gmp_canopen_frame_t* request,
                                                 gmp_canopen_frame_t* response, uint16_t index, uint16_t subindex)
{
    uint16_t command = (uint16_t)(request->data[0] & 0xFFU);
    fast_gt expedited = (command & 0x02U) != 0U;
    fast_gt size_indicated = (command & 0x01U) != 0U;
    uint32_t size;
    gmp_canopen_od_result_t result;
    if ((command & 0x10U) != 0U || ((!expedited || !size_indicated) && (command & 0x0CU) != 0U))
    {
        gmp_canopen_sdo_abort(server, response, index, subindex, GMP_CANOPEN_SDO_ABORT_COMMAND);
        return 1;
    }
    server->entry = gmp_canopen_od_find(server->dictionary, index, subindex);
    if (server->entry == NULL)
    {
        gmp_canopen_sdo_abort(server, response, index, subindex, GMP_CANOPEN_SDO_ABORT_NOT_FOUND);
        return 1;
    }
    if (expedited)
    {
        size = size_indicated ? 4U - ((command >> 2U) & 0x03U) : 4U;
        result = gmp_canopen_od_write(server->entry, &request->data[4], size);
        if (result != GMP_CANOPEN_OD_OK)
        {
            gmp_canopen_sdo_abort(server, response, index, subindex, gmp_canopen_sdo_abort_for_od(result, 1));
            return 1;
        }
        gmp_canopen_sdo_response_begin(server, response);
        response->data[0] = 0x60U;
        gmp_canopen_store_le16(&response->data[1], index);
        response->data[3] = (byte_gt)subindex;
        gmp_canopen_sdo_server_reset(server);
        return 1;
    }
    if (!size_indicated)
    {
        gmp_canopen_sdo_abort(server, response, index, subindex, GMP_CANOPEN_SDO_ABORT_UNSUPPORTED);
        return 1;
    }
    size = gmp_canopen_load_le32(&request->data[4]);
    if (size > GMP_CANOPEN_SDO_MAX_TRANSFER)
    {
        gmp_canopen_sdo_abort(server, response, index, subindex, GMP_CANOPEN_SDO_ABORT_OUT_OF_MEMORY);
        return 1;
    }
    if (size != server->entry->size)
    {
        gmp_canopen_sdo_abort(server, response, index, subindex, GMP_CANOPEN_SDO_ABORT_TYPE_LENGTH);
        return 1;
    }
    server->state = GMP_CANOPEN_SDO_SEGMENTED_DOWNLOAD;
    server->offset = 0U;
    server->total_size = size;
    server->toggle = 0U;
    gmp_canopen_sdo_response_begin(server, response);
    response->data[0] = 0x60U;
    gmp_canopen_store_le16(&response->data[1], index);
    response->data[3] = (byte_gt)subindex;
    return 1;
}

static fast_gt gmp_canopen_sdo_download_segment(gmp_canopen_sdo_server_t* server, const gmp_canopen_frame_t* request,
                                                gmp_canopen_frame_t* response)
{
    uint16_t command = (uint16_t)(request->data[0] & 0xFFU);
    uint16_t toggle = (command >> 4U) & 0x01U;
    uint32_t unused = (command >> 1U) & 0x07U;
    uint32_t count = 7U - unused;
    fast_gt last = (command & 0x01U) != 0U;
    uint32_t index;
    gmp_canopen_od_result_t result;
    if (server->state != GMP_CANOPEN_SDO_SEGMENTED_DOWNLOAD || toggle != server->toggle)
    {
        index = server->entry == NULL ? 0U : server->entry->index;
        gmp_canopen_sdo_abort(server, response, (uint16_t)index, server->entry == NULL ? 0U : server->entry->subindex,
                              GMP_CANOPEN_SDO_ABORT_TOGGLE);
        return 1;
    }
    if (!last && unused != 0U)
    {
        gmp_canopen_sdo_abort(server, response, server->entry->index, server->entry->subindex,
                              GMP_CANOPEN_SDO_ABORT_COMMAND);
        return 1;
    }
    if (server->offset + count > server->total_size)
    {
        gmp_canopen_sdo_abort(server, response, server->entry->index, server->entry->subindex,
                              GMP_CANOPEN_SDO_ABORT_TYPE_LENGTH);
        return 1;
    }
    for (index = 0U; index < count; ++index)
        server->transfer[server->offset + index] = request->data[index + 1U];
    server->offset += count;
    gmp_canopen_sdo_response_begin(server, response);
    response->data[0] = (byte_gt)(0x20U | (toggle << 4U));
    if (last)
    {
        if (server->offset != server->total_size)
        {
            gmp_canopen_sdo_abort(server, response, server->entry->index, server->entry->subindex,
                                  GMP_CANOPEN_SDO_ABORT_TYPE_LENGTH);
            return 1;
        }
        result = gmp_canopen_od_write(server->entry, server->transfer, server->total_size);
        if (result != GMP_CANOPEN_OD_OK)
        {
            gmp_canopen_sdo_abort(server, response, server->entry->index, server->entry->subindex,
                                  gmp_canopen_sdo_abort_for_od(result, 1));
            return 1;
        }
        gmp_canopen_sdo_server_reset(server);
    }
    else
    {
        server->toggle ^= 1U;
    }
    return 1;
}

static fast_gt gmp_canopen_sdo_initiate_upload(gmp_canopen_sdo_server_t* server, gmp_canopen_frame_t* response,
                                               uint16_t index, uint16_t subindex)
{
    uint32_t size = 0U;
    gmp_canopen_od_result_t result;
    server->entry = gmp_canopen_od_find(server->dictionary, index, subindex);
    if (server->entry == NULL)
    {
        gmp_canopen_sdo_abort(server, response, index, subindex, GMP_CANOPEN_SDO_ABORT_NOT_FOUND);
        return 1;
    }
    if (server->entry->size > GMP_CANOPEN_SDO_MAX_TRANSFER)
    {
        gmp_canopen_sdo_abort(server, response, index, subindex, GMP_CANOPEN_SDO_ABORT_OUT_OF_MEMORY);
        return 1;
    }
    result = gmp_canopen_od_read(server->entry, server->transfer, GMP_CANOPEN_SDO_MAX_TRANSFER, &size);
    if (result != GMP_CANOPEN_OD_OK)
    {
        gmp_canopen_sdo_abort(server, response, index, subindex, gmp_canopen_sdo_abort_for_od(result, 0));
        return 1;
    }
    gmp_canopen_sdo_response_begin(server, response);
    gmp_canopen_store_le16(&response->data[1], index);
    response->data[3] = (byte_gt)subindex;
    if (size <= 4U)
    {
        uint32_t byte_index;
        response->data[0] = (byte_gt)(0x43U | ((4U - size) << 2U));
        for (byte_index = 0U; byte_index < size; ++byte_index)
            response->data[4U + byte_index] = server->transfer[byte_index];
        gmp_canopen_sdo_server_reset(server);
    }
    else
    {
        response->data[0] = 0x41U;
        gmp_canopen_store_le32(&response->data[4], size);
        server->state = GMP_CANOPEN_SDO_SEGMENTED_UPLOAD;
        server->offset = 0U;
        server->total_size = size;
        server->toggle = 0U;
    }
    return 1;
}

static fast_gt gmp_canopen_sdo_upload_segment(gmp_canopen_sdo_server_t* server, const gmp_canopen_frame_t* request,
                                              gmp_canopen_frame_t* response)
{
    uint16_t requested_toggle = ((uint16_t)request->data[0] >> 4U) & 0x01U;
    uint32_t remaining;
    uint32_t count;
    uint32_t unused;
    uint32_t index;
    fast_gt last;
    if (server->state != GMP_CANOPEN_SDO_SEGMENTED_UPLOAD || requested_toggle != server->toggle)
    {
        gmp_canopen_sdo_abort(server, response, server->entry == NULL ? 0U : server->entry->index,
                              server->entry == NULL ? 0U : server->entry->subindex, GMP_CANOPEN_SDO_ABORT_TOGGLE);
        return 1;
    }
    if ((request->data[0] & 0x0FU) != 0U)
    {
        gmp_canopen_sdo_abort(server, response, server->entry->index, server->entry->subindex,
                              GMP_CANOPEN_SDO_ABORT_COMMAND);
        return 1;
    }
    remaining = server->total_size - server->offset;
    count = remaining > 7U ? 7U : remaining;
    unused = 7U - count;
    last = count == remaining;
    gmp_canopen_sdo_response_begin(server, response);
    response->data[0] = (byte_gt)((server->toggle << 4U) |
        (unused << 1U) | (last ? 1U : 0U));
    for (index = 0U; index < count; ++index)
        response->data[index + 1U] = server->transfer[server->offset + index];
    server->offset += count;
    if (last)
        gmp_canopen_sdo_server_reset(server);
    else
        server->toggle ^= 1U;
    return 1;
}

fast_gt gmp_canopen_sdo_server_process(gmp_canopen_sdo_server_t* server, const gmp_canopen_frame_t* request,
                                       gmp_canopen_frame_t* response)
{
    uint16_t command;
    uint16_t client_command;
    uint16_t index;
    uint16_t subindex;
    if (server == NULL || request == NULL || response == NULL ||
        !gmp_canopen_frame_validate(request) || request->is_extended || request->is_remote ||
        request->dlc != 8U || request->id != GMP_CANOPEN_COB_RSDO + server->node_id)
        return 0;
    command = (uint16_t)(request->data[0] & 0xFFU);
    client_command = command >> 5U;
    index = gmp_canopen_load_le16(&request->data[1]);
    subindex = (uint16_t)(request->data[3] & 0xFFU);
    if (command == 0x80U)
    {
        gmp_canopen_sdo_server_reset(server);
        return 0;
    }
    if ((server->state == GMP_CANOPEN_SDO_SEGMENTED_DOWNLOAD && client_command != 0U) ||
        (server->state == GMP_CANOPEN_SDO_SEGMENTED_UPLOAD && client_command != 3U) ||
        (server->state == GMP_CANOPEN_SDO_IDLE && (client_command == 0U || client_command == 3U)))
    {
        gmp_canopen_sdo_abort(server, response, server->entry == NULL ? index : server->entry->index,
                              server->entry == NULL ? subindex : server->entry->subindex,
                              GMP_CANOPEN_SDO_ABORT_COMMAND);
        return 1;
    }
    if (client_command == 1U)
        return gmp_canopen_sdo_initiate_download(server, request, response, index, subindex);
    if (client_command == 0U)
        return gmp_canopen_sdo_download_segment(server, request, response);
    if (client_command == 2U && command == 0x40U)
        return gmp_canopen_sdo_initiate_upload(server, response, index, subindex);
    if (client_command == 3U)
        return gmp_canopen_sdo_upload_segment(server, request, response);
    gmp_canopen_sdo_abort(server, response, index, subindex, GMP_CANOPEN_SDO_ABORT_COMMAND);
    return 1;
}

fast_gt gmp_canopen_sdo_server_receive(gmp_canopen_sdo_server_t* server, const gmp_canopen_frame_t* request,
                                       gmp_canopen_send_fn send, void* send_context)
{
    gmp_canopen_frame_t response;
    if (send == NULL || !gmp_canopen_sdo_server_process(server, request, &response))
        return 0;
    return send(send_context, &response);
}
