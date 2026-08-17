/**
 * @file canopen.c
 * @brief Complete CANopen node facade, queues, and transport dispatch.
 */

#include <core/protocol/canopen/canopen.h>

#include <string.h>

static void gmp_canopen_queue_init(gmp_canopen_packet_queue_t* queue)
{
    queue->write_index = 0U;
    queue->read_index = 0U;
}

static fast_gt gmp_canopen_packet_validate(
    const gmp_canopen_packet_t* packet)
{
    uint32_t index;
    if (packet == NULL || packet->kind == GMP_CANOPEN_PACKET_NONE ||
        packet->data_size > GMP_CANOPEN_PACKET_MAX_DATA ||
        packet->subindex > 0xFFU)
        return 0;
    if (packet->transport == GMP_CANOPEN_TRANSPORT_CAN)
    {
        if (packet->kind != GMP_CANOPEN_PACKET_CAN_FRAME ||
            packet->data_size > GMP_CANOPEN_CLASSIC_MAX_DATA ||
            ((packet->flags & GMP_CANOPEN_PACKET_FLAG_EXTENDED) == 0U &&
             packet->key > 0x7FFU) ||
            packet->key > 0x1FFFFFFFUL)
            return 0;
    }
    else if (packet->transport != GMP_CANOPEN_TRANSPORT_COE ||
             packet->kind == GMP_CANOPEN_PACKET_CAN_FRAME)
    {
        return 0;
    }
    for (index = 0U; index < packet->data_size; ++index)
        if (packet->data[index] > 0xFFU)
            return 0;
    return 1;
}

static fast_gt gmp_canopen_packet_is_input(
    const gmp_canopen_packet_t* packet)
{
    if (packet->transport == GMP_CANOPEN_TRANSPORT_CAN)
        return packet->kind == GMP_CANOPEN_PACKET_CAN_FRAME;
    return packet->kind == GMP_CANOPEN_PACKET_COE_SDO_UPLOAD ||
           packet->kind == GMP_CANOPEN_PACKET_COE_SDO_DOWNLOAD ||
           packet->kind == GMP_CANOPEN_PACKET_COE_RXPDO ||
           packet->kind == GMP_CANOPEN_PACKET_COE_TXPDO_REQUEST;
}

static void gmp_canopen_packet_copy(gmp_canopen_packet_t* destination,
                                     const gmp_canopen_packet_t* source)
{
    uint32_t index;
    destination->transport = source->transport;
    destination->kind = source->kind;
    destination->key = source->key;
    destination->abort_code = source->abort_code;
    destination->data_size = source->data_size;
    destination->number = source->number;
    destination->index = source->index;
    destination->subindex = source->subindex;
    destination->flags = source->flags;
    destination->coe_result = source->coe_result;
    for (index = 0U; index < source->data_size; ++index)
        destination->data[index] = source->data[index];
}

static fast_gt gmp_canopen_queue_push(gmp_canopen_packet_queue_t* queue,
                                      const gmp_canopen_packet_t* packet)
{
    uint16_t next = (uint16_t)(queue->write_index + 1U);
    if (next >= GMP_CANOPEN_QUEUE_SLOTS)
        next = 0U;
    if (next == queue->read_index)
        return 0;
    gmp_canopen_packet_copy(&queue->packets[queue->write_index], packet);
    queue->write_index = next;
    return 1;
}

static fast_gt gmp_canopen_queue_pop(gmp_canopen_packet_queue_t* queue,
                                     gmp_canopen_packet_t* packet)
{
    uint16_t next;
    if (queue->read_index == queue->write_index)
        return 0;
    gmp_canopen_packet_copy(packet, &queue->packets[queue->read_index]);
    next = (uint16_t)(queue->read_index + 1U);
    if (next >= GMP_CANOPEN_QUEUE_SLOTS)
        next = 0U;
    queue->read_index = next;
    return 1;
}

void gmp_canopen_packet_clear(gmp_canopen_packet_t* packet)
{
    uint32_t index;
    if (packet == NULL)
        return;
    packet->transport = GMP_CANOPEN_TRANSPORT_CAN;
    packet->kind = GMP_CANOPEN_PACKET_NONE;
    packet->key = 0U;
    packet->abort_code = 0U;
    packet->data_size = 0U;
    packet->number = 0U;
    packet->index = 0U;
    packet->subindex = 0U;
    packet->flags = 0U;
    packet->coe_result = GMP_COE_OK;
    for (index = 0U; index < GMP_CANOPEN_PACKET_MAX_DATA; ++index)
        packet->data[index] = 0U;
}

fast_gt gmp_canopen_packet_from_can(
    const gmp_canopen_frame_t* frame, gmp_canopen_packet_t* packet)
{
    uint16_t index;
    if (!gmp_canopen_frame_validate(frame) || packet == NULL ||
        frame->id > (frame->is_extended ? 0x1FFFFFFFUL : 0x7FFU))
        return 0;
    gmp_canopen_packet_clear(packet);
    packet->transport = GMP_CANOPEN_TRANSPORT_CAN;
    packet->kind = GMP_CANOPEN_PACKET_CAN_FRAME;
    packet->key = frame->id;
    packet->data_size = frame->dlc;
    if (frame->is_extended)
        packet->flags |= GMP_CANOPEN_PACKET_FLAG_EXTENDED;
    if (frame->is_remote)
        packet->flags |= GMP_CANOPEN_PACKET_FLAG_REMOTE;
    for (index = 0U; index < frame->dlc; ++index)
        packet->data[index] = frame->data[index];
    return 1;
}

fast_gt gmp_canopen_packet_to_can(
    const gmp_canopen_packet_t* packet, gmp_canopen_frame_t* frame)
{
    uint16_t index;
    if (frame == NULL || !gmp_canopen_packet_validate(packet) ||
        packet->transport != GMP_CANOPEN_TRANSPORT_CAN ||
        packet->kind != GMP_CANOPEN_PACKET_CAN_FRAME)
        return 0;
    gmp_canopen_frame_clear(frame);
    frame->id = packet->key;
    frame->dlc = (uint16_t)packet->data_size;
    frame->is_extended =
        (packet->flags & GMP_CANOPEN_PACKET_FLAG_EXTENDED) != 0U;
    frame->is_remote =
        (packet->flags & GMP_CANOPEN_PACKET_FLAG_REMOTE) != 0U;
    for (index = 0U; index < frame->dlc; ++index)
        frame->data[index] = packet->data[index];
    return 1;
}

fast_gt gmp_canopen_config_init(gmp_canopen_config_t* config,
                                byte_gt node_id)
{
    if (config == NULL || node_id < GMP_CANOPEN_NODE_ID_MIN ||
        node_id > GMP_CANOPEN_NODE_ID_MAX)
        return 0;
    memset(config, 0, sizeof(*config));
    config->node_id = node_id;
    return 1;
}

static fast_gt gmp_canopen_add_standard_entry(
    gmp_canopen_t* node, uint16_t index, byte_gt subindex,
    gmp_canopen_od_data_type_t data_type, uint16_t access,
    void* storage, uint32_t size, const char* name)
{
    gmp_canopen_od_entry_t* entry;
    if (node->cia301_entry_count >= GMP_CANOPEN_CIA301_ENTRY_CAPACITY ||
        storage == NULL)
        return 0;
    entry = &node->cia301_entries[node->cia301_entry_count];
    gmp_canopen_od_entry_init_pointer(entry, index, subindex, data_type,
        access, storage, size, name);
    if (gmp_canopen_od_insert(&node->dictionary, entry) != GMP_CANOPEN_OD_OK)
        return 0;
    ++node->cia301_entry_count;
    return 1;
}

static fast_gt gmp_canopen_register_cia301(gmp_canopen_t* node)
{
    const uint16_t read_only =
        GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_CONST;
    node->identity_subindex_count = 4U;
    if (!gmp_canopen_add_standard_entry(node, 0x1000U, 0U,
            GMP_CANOPEN_OD_UNSIGNED32, read_only,
            &node->cia301.device_type, 0U, "Device type") ||
        !gmp_canopen_add_standard_entry(node, 0x1001U, 0U,
            GMP_CANOPEN_OD_UNSIGNED8, read_only,
            &node->cia301.error_register, 0U, "Error register") ||
        !gmp_canopen_add_standard_entry(node, 0x1017U, 0U,
            GMP_CANOPEN_OD_UNSIGNED16,
            GMP_CANOPEN_OD_ACCESS_READ | GMP_CANOPEN_OD_ACCESS_WRITE,
            &node->cia301.producer_heartbeat_time_ms, 0U,
            "Producer heartbeat time") ||
        !gmp_canopen_add_standard_entry(node, 0x1018U, 0U,
            GMP_CANOPEN_OD_UNSIGNED8, read_only,
            &node->identity_subindex_count, 0U,
            "Identity highest sub-index") ||
        !gmp_canopen_add_standard_entry(node, 0x1018U, 1U,
            GMP_CANOPEN_OD_UNSIGNED32, read_only,
            &node->cia301.identity.vendor_id, 0U, "Vendor ID") ||
        !gmp_canopen_add_standard_entry(node, 0x1018U, 2U,
            GMP_CANOPEN_OD_UNSIGNED32, read_only,
            &node->cia301.identity.product_code, 0U, "Product code") ||
        !gmp_canopen_add_standard_entry(node, 0x1018U, 3U,
            GMP_CANOPEN_OD_UNSIGNED32, read_only,
            &node->cia301.identity.revision_number, 0U,
            "Revision number") ||
        !gmp_canopen_add_standard_entry(node, 0x1018U, 4U,
            GMP_CANOPEN_OD_UNSIGNED32, read_only,
            &node->cia301.identity.serial_number, 0U, "Serial number"))
        return 0;
    if (node->cia301.device_name != NULL &&
        node->cia301.device_name_size != 0U &&
        !gmp_canopen_add_standard_entry(node, 0x1008U, 0U,
            GMP_CANOPEN_OD_VISIBLE_STRING, read_only,
            (void*)node->cia301.device_name,
            node->cia301.device_name_size, "Manufacturer device name"))
        return 0;
    if (node->cia301.hardware_version != NULL &&
        node->cia301.hardware_version_size != 0U &&
        !gmp_canopen_add_standard_entry(node, 0x1009U, 0U,
            GMP_CANOPEN_OD_VISIBLE_STRING, read_only,
            (void*)node->cia301.hardware_version,
            node->cia301.hardware_version_size, "Hardware version"))
        return 0;
    if (node->cia301.software_version != NULL &&
        node->cia301.software_version_size != 0U &&
        !gmp_canopen_add_standard_entry(node, 0x100AU, 0U,
            GMP_CANOPEN_OD_VISIBLE_STRING, read_only,
            (void*)node->cia301.software_version,
            node->cia301.software_version_size, "Software version"))
        return 0;
    return 1;
}

gmp_canopen_node_result_t gmp_canopen_init(
    gmp_canopen_t* node, const gmp_canopen_config_t* config)
{
    if (node == NULL || config == NULL ||
        config->node_id < GMP_CANOPEN_NODE_ID_MIN ||
        config->node_id > GMP_CANOPEN_NODE_ID_MAX ||
        config->cia301.error_register > 0xFFU)
        return GMP_CANOPEN_NODE_INVALID;
    memset(node, 0, sizeof(*node));
    node->cia301 = config->cia301;
    gmp_canopen_od_init(&node->dictionary);
    if (!gmp_canopen_register_cia301(node) ||
        !gmp_canopen_nmt_init(&node->nmt, config->node_id,
            node->cia301.producer_heartbeat_time_ms) ||
        !gmp_canopen_sdo_server_init(&node->sdo, config->node_id,
            &node->dictionary) ||
        !gmp_coe_server_init(&node->coe, &node->dictionary))
        return GMP_CANOPEN_NODE_PROTOCOL_ERROR;
    gmp_canopen_txpdo_group_init(&node->txpdo_group);
    gmp_canopen_rxpdo_group_init(&node->rxpdo_group);
    gmp_canopen_queue_init(&node->input_queue);
    gmp_canopen_queue_init(&node->output_queue);
    node->initialized = 1;
    return GMP_CANOPEN_NODE_OK;
}

gmp_canopen_node_result_t gmp_canopen_init_default(
    gmp_canopen_t* node, byte_gt node_id)
{
    gmp_canopen_config_t config;
    if (!gmp_canopen_config_init(&config, node_id))
        return GMP_CANOPEN_NODE_INVALID;
    return gmp_canopen_init(node, &config);
}

gmp_canopen_node_result_t gmp_canopen_input_callback(
    gmp_canopen_t* node, const gmp_canopen_packet_t* packet)
{
    if (node == NULL || !node->initialized)
        return GMP_CANOPEN_NODE_INVALID;
    if (!gmp_canopen_packet_validate(packet) ||
        !gmp_canopen_packet_is_input(packet))
    {
        ++node->statistics.input_dropped;
        return GMP_CANOPEN_NODE_INVALID;
    }
    if (!gmp_canopen_queue_push(&node->input_queue, packet))
    {
        ++node->statistics.input_dropped;
        return GMP_CANOPEN_NODE_RX_FULL;
    }
    ++node->statistics.input_packets;
    return GMP_CANOPEN_NODE_OK;
}

static gmp_canopen_node_result_t gmp_canopen_enqueue_output(
    gmp_canopen_t* node, const gmp_canopen_packet_t* packet)
{
    if (!gmp_canopen_queue_push(&node->output_queue, packet))
    {
        ++node->statistics.output_dropped;
        return GMP_CANOPEN_NODE_TX_FULL;
    }
    ++node->statistics.output_packets;
    return GMP_CANOPEN_NODE_OK;
}

gmp_canopen_node_result_t gmp_canopen_output_callback(
    gmp_canopen_t* node, gmp_canopen_packet_t* packet)
{
    if (node == NULL || packet == NULL || !node->initialized)
        return GMP_CANOPEN_NODE_INVALID;
    if (!gmp_canopen_queue_pop(&node->output_queue, packet))
        return GMP_CANOPEN_NODE_EMPTY;
    ++node->statistics.output_consumed;
    return GMP_CANOPEN_NODE_OK;
}

gmp_canopen_pdo_result_t gmp_canopen_add_txpdo(
    gmp_canopen_t* node, gmp_canopen_txpdo_t* pdo)
{
    if (node == NULL || !node->initialized)
        return GMP_CANOPEN_PDO_INVALID;
    return gmp_canopen_txpdo_group_add(&node->txpdo_group, pdo);
}

gmp_canopen_pdo_result_t gmp_canopen_add_rxpdo(
    gmp_canopen_t* node, gmp_canopen_rxpdo_t* pdo)
{
    if (node == NULL || !node->initialized)
        return GMP_CANOPEN_PDO_INVALID;
    return gmp_canopen_rxpdo_group_add(&node->rxpdo_group, pdo);
}

gmp_canopen_node_result_t gmp_canopen_publish_tpdo(
    gmp_canopen_t* node, gmp_canopen_transport_t transport, uint32_t key)
{
    gmp_canopen_txpdo_t* pdo;
    gmp_canopen_packet_t packet;
    if (node == NULL || !node->initialized)
        return GMP_CANOPEN_NODE_INVALID;
    pdo = gmp_canopen_txpdo_group_find(&node->txpdo_group, key);
    if (pdo == NULL)
        return GMP_CANOPEN_NODE_NOT_FOUND;
    gmp_canopen_packet_clear(&packet);
    if (transport == GMP_CANOPEN_TRANSPORT_CAN)
    {
        gmp_canopen_frame_t frame;
        if (gmp_canopen_txpdo_build_frame_fast(pdo, node->nmt.state,
                &frame) != GMP_CANOPEN_PDO_OK ||
            !gmp_canopen_packet_from_can(&frame, &packet))
            return GMP_CANOPEN_NODE_PROTOCOL_ERROR;
    }
    else if (transport == GMP_CANOPEN_TRANSPORT_COE)
    {
        uint16_t size;
        packet.transport = GMP_CANOPEN_TRANSPORT_COE;
        packet.kind = GMP_CANOPEN_PACKET_COE_TXPDO;
        packet.key = key;
        if (gmp_coe_txpdo_pack_fast(pdo, packet.data,
                GMP_CANOPEN_PACKET_MAX_DATA, &size) != GMP_CANOPEN_PDO_OK)
            return GMP_CANOPEN_NODE_PROTOCOL_ERROR;
        packet.data_size = size;
    }
    else
    {
        return GMP_CANOPEN_NODE_INVALID;
    }
    return gmp_canopen_enqueue_output(node, &packet);
}

static gmp_canopen_node_result_t gmp_canopen_process_can(
    gmp_canopen_t* node, const gmp_canopen_packet_t* packet)
{
    gmp_canopen_frame_t frame;
    if (!gmp_canopen_packet_to_can(packet, &frame))
        return GMP_CANOPEN_NODE_INVALID;
    if (frame.id == GMP_CANOPEN_COB_NMT)
    {
        (void)gmp_canopen_nmt_receive(&node->nmt, &frame);
        return GMP_CANOPEN_NODE_OK;
    }
    if (frame.id == GMP_CANOPEN_COB_RSDO + node->nmt.node_id)
    {
        gmp_canopen_frame_t response;
        gmp_canopen_packet_t output;
        if (!gmp_canopen_sdo_server_process(&node->sdo, &frame, &response))
            return GMP_CANOPEN_NODE_OK;
        if (!gmp_canopen_packet_from_can(&response, &output))
            return GMP_CANOPEN_NODE_PROTOCOL_ERROR;
        return gmp_canopen_enqueue_output(node, &output);
    }
    if (frame.is_remote)
        return gmp_canopen_publish_tpdo(node, GMP_CANOPEN_TRANSPORT_CAN,
                                        frame.id);
    {
        gmp_canopen_pdo_result_t result =
            gmp_canopen_rxpdo_group_dispatch_fast(&node->rxpdo_group,
                node->nmt.state, &frame);
        if (result == GMP_CANOPEN_PDO_OK ||
            result == GMP_CANOPEN_PDO_NOT_FOUND)
            return GMP_CANOPEN_NODE_OK;
    }
    return GMP_CANOPEN_NODE_PROTOCOL_ERROR;
}

static gmp_canopen_node_result_t gmp_canopen_process_coe_sdo(
    gmp_canopen_t* node, const gmp_canopen_packet_t* packet)
{
    gmp_coe_sdo_request_t request;
    gmp_coe_sdo_response_t response;
    gmp_canopen_packet_t output;
    gmp_canopen_packet_clear(&output);
    output.transport = GMP_CANOPEN_TRANSPORT_COE;
    output.kind = GMP_CANOPEN_PACKET_COE_SDO_RESPONSE;
    output.number = packet->number;
    output.index = packet->index;
    output.subindex = packet->subindex;
    request.number = packet->number;
    request.operation = packet->kind == GMP_CANOPEN_PACKET_COE_SDO_UPLOAD ?
        GMP_COE_SDO_UPLOAD : GMP_COE_SDO_DOWNLOAD;
    request.index = packet->index;
    request.subindex = packet->subindex;
    request.complete_access =
        (packet->flags & GMP_CANOPEN_PACKET_FLAG_COMPLETE_ACCESS) != 0U;
    request.data = request.operation == GMP_COE_SDO_DOWNLOAD ?
        packet->data : NULL;
    request.data_size = request.operation == GMP_COE_SDO_DOWNLOAD ?
        packet->data_size : 0U;
    response.number = packet->number;
    response.result = GMP_COE_INVALID;
    response.abort_code = 0U;
    response.data = output.data;
    response.capacity = GMP_CANOPEN_PACKET_MAX_DATA;
    response.data_size = 0U;
    output.coe_result = gmp_coe_sdo_server_process(
        &node->coe, &request, &response);
    output.abort_code = response.abort_code;
    output.data_size = response.data_size;
    return gmp_canopen_enqueue_output(node, &output);
}

static gmp_canopen_node_result_t gmp_canopen_process_coe(
    gmp_canopen_t* node, const gmp_canopen_packet_t* packet)
{
    if (packet->kind == GMP_CANOPEN_PACKET_COE_SDO_UPLOAD ||
        packet->kind == GMP_CANOPEN_PACKET_COE_SDO_DOWNLOAD)
        return gmp_canopen_process_coe_sdo(node, packet);
    if (packet->kind == GMP_CANOPEN_PACKET_COE_RXPDO)
    {
        gmp_canopen_rxpdo_t* pdo = gmp_canopen_rxpdo_group_find(
            &node->rxpdo_group, packet->key);
        if (pdo == NULL)
            return GMP_CANOPEN_NODE_NOT_FOUND;
        return gmp_coe_rxpdo_unpack_fast(pdo, packet->data,
            (uint16_t)packet->data_size) == GMP_CANOPEN_PDO_OK ?
            GMP_CANOPEN_NODE_OK : GMP_CANOPEN_NODE_PROTOCOL_ERROR;
    }
    if (packet->kind == GMP_CANOPEN_PACKET_COE_TXPDO_REQUEST)
        return gmp_canopen_publish_tpdo(node, GMP_CANOPEN_TRANSPORT_COE,
                                        packet->key);
    return GMP_CANOPEN_NODE_INVALID;
}

uint16_t gmp_canopen_background_callback(gmp_canopen_t* node,
                                         uint32_t elapsed_ms)
{
    uint16_t processed = 0U;
    gmp_canopen_packet_t packet;
    if (node == NULL || !node->initialized)
        return 0U;
    node->nmt.heartbeat_period_ms =
        node->cia301.producer_heartbeat_time_ms;
    {
        gmp_canopen_frame_t heartbeat;
        if (gmp_canopen_nmt_tick(&node->nmt, elapsed_ms, &heartbeat))
        {
            gmp_canopen_packet_t output;
            if (gmp_canopen_packet_from_can(&heartbeat, &output))
                (void)gmp_canopen_enqueue_output(node, &output);
        }
    }
    while (processed < GMP_CANOPEN_BACKGROUND_BUDGET &&
           gmp_canopen_queue_pop(&node->input_queue, &packet))
    {
        gmp_canopen_node_result_t result =
            packet.transport == GMP_CANOPEN_TRANSPORT_CAN ?
            gmp_canopen_process_can(node, &packet) :
            gmp_canopen_process_coe(node, &packet);
        ++processed;
        ++node->statistics.processed_packets;
        if (result != GMP_CANOPEN_NODE_OK &&
            result != GMP_CANOPEN_NODE_NOT_FOUND)
            ++node->statistics.protocol_errors;
    }
    return processed;
}
