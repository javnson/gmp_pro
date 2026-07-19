/**
 * @file gmp_can_service.c
 * @brief Static queue and asynchronous event implementation for GMP CAN.
 */

#include <gmp_core.h>
#include <core/dev/can/can.h>

#ifndef GMP_CAN_MAX_NODES
#define GMP_CAN_MAX_NODES 8U
#endif

typedef struct
{
    can_halt hcan;
    gmp_can_storage_t storage;
    uint16_t tx_head;
    uint16_t tx_tail;
    uint16_t tx_count;
    uint16_t rx_head;
    uint16_t rx_tail;
    uint16_t rx_count;
    uint16_t tx_event_head;
    uint16_t tx_event_tail;
    uint16_t tx_event_count;
    uint16_t state_head;
    uint16_t state_tail;
    uint16_t state_count;
    uint32_t next_token;
    fast_gt initialized;
} gmp_can_node_context_t;

static gmp_can_node_context_t g_can_nodes[GMP_CAN_MAX_NODES];

static gmp_can_node_context_t* find_node(can_halt hcan)
{
    uint16_t index;
    for (index = 0U; index < GMP_CAN_MAX_NODES; ++index)
    {
        if (g_can_nodes[index].initialized && g_can_nodes[index].hcan == hcan)
        {
            return &g_can_nodes[index];
        }
    }
    return NULL;
}

static fast_gt storage_is_valid(const gmp_can_storage_t* storage)
{
    return storage != NULL && storage->tx_entries != NULL && storage->tx_capacity != 0U &&
           storage->rx_events != NULL && storage->rx_capacity != 0U && storage->tx_events != NULL &&
           storage->tx_event_capacity != 0U && storage->state_events != NULL &&
           storage->state_event_capacity != 0U;
}

ec_gt gmp_can_node_init(can_halt hcan, const gmp_can_storage_t* storage)
{
    uint16_t index;
    gmp_can_node_context_t* node;

    if (hcan == (can_halt)0 || !storage_is_valid(storage))
    {
        return GMP_EC_INVALID_PARAM;
    }
    if (find_node(hcan) != NULL)
    {
        return GMP_EC_REINIT;
    }

    for (index = 0U; index < GMP_CAN_MAX_NODES; ++index)
    {
        if (!g_can_nodes[index].initialized)
        {
            node = &g_can_nodes[index];
            memset(node, 0, sizeof(*node));
            node->hcan = hcan;
            node->storage = *storage;
            node->next_token = 1U;
            node->initialized = 1;
            return GMP_EC_OK;
        }
    }
    return GMP_EC_OVERFLOW;
}

uint16_t gmp_can_frame_get_u8(const gmp_can_frame_t* frame, uint16_t offset)
{
    uint32_t word;
    if (frame == NULL || offset >= frame->length || offset >= GMP_CAN_MAX_DATA_BYTES)
    {
        return 0U;
    }
    word = frame->data_32[offset / 4U];
    return (uint16_t)((word >> ((offset % 4U) * 8U)) & 0xFFU);
}

ec_gt gmp_can_frame_set_u8(gmp_can_frame_t* frame, uint16_t offset, uint16_t value)
{
    uint32_t shift;
    uint32_t mask;
    if (frame == NULL || offset >= GMP_CAN_MAX_DATA_BYTES || value > 0xFFU)
    {
        return GMP_EC_INVALID_PARAM;
    }
    shift = (uint32_t)(offset % 4U) * 8U;
    mask = 0xFFUL << shift;
    frame->data_32[offset / 4U] = (frame->data_32[offset / 4U] & ~mask) | ((uint32_t)value << shift);
    if (frame->length <= offset)
    {
        frame->length = (uint16_t)(offset + 1U);
    }
    return GMP_EC_OK;
}

fast_gt gmp_can_frame_is_valid(const gmp_can_frame_t* frame)
{
    static const uint16_t fd_lengths[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};
    uint16_t index;

    if (frame == NULL || frame->length > GMP_CAN_MAX_DATA_BYTES)
    {
        return 0;
    }
    if ((frame->flags & GMP_CAN_FRAME_EXTENDED_ID) != 0U)
    {
        if (frame->id > 0x1FFFFFFFUL)
            return 0;
    }
    else if (frame->id > 0x7FFUL)
    {
        return 0;
    }
    if ((frame->flags & GMP_CAN_FRAME_FD) == 0U)
    {
        return frame->length <= 8U;
    }
    if ((frame->flags & GMP_CAN_FRAME_REMOTE) != 0U)
    {
        return 0;
    }
    for (index = 0U; index < (uint16_t)(sizeof(fd_lengths) / sizeof(fd_lengths[0])); ++index)
    {
        if (frame->length == fd_lengths[index])
            return 1;
    }
    return 0;
}

ec_gt gmp_can_get_capabilities(can_halt hcan, gmp_can_capabilities_t* capabilities)
{
    if (hcan == (can_halt)0 || capabilities == NULL)
        return GMP_EC_INVALID_PARAM;
    return gmp_hal_can_get_capabilities(hcan, capabilities);
}

ec_gt gmp_can_configure(can_halt hcan, const gmp_can_config_t* config)
{
    return hcan == (can_halt)0 || config == NULL ? GMP_EC_INVALID_PARAM : gmp_hal_can_configure(hcan, config);
}

ec_gt gmp_can_start(can_halt hcan)
{
    return hcan == (can_halt)0 ? GMP_EC_INVALID_PARAM : gmp_hal_can_start(hcan);
}

ec_gt gmp_can_stop(can_halt hcan)
{
    return hcan == (can_halt)0 ? GMP_EC_INVALID_PARAM : gmp_hal_can_stop(hcan);
}

ec_gt gmp_can_set_filter(can_halt hcan, uint16_t filter_slot, const gmp_can_hw_filter_t* filter)
{
    return hcan == (can_halt)0 || filter == NULL ? GMP_EC_INVALID_PARAM :
           gmp_hal_can_set_filter(hcan, filter_slot, filter);
}

ec_gt gmp_can_get_state(can_halt hcan, gmp_can_state_t* state)
{
    return hcan == (can_halt)0 || state == NULL ? GMP_EC_INVALID_PARAM : gmp_hal_can_get_state(hcan, state);
}

ec_gt gmp_can_recover(can_halt hcan)
{
    return hcan == (can_halt)0 ? GMP_EC_INVALID_PARAM : gmp_hal_can_recover(hcan);
}

ec_gt gmp_can_send(can_halt hcan, const gmp_can_frame_t* frame, const gmp_can_tx_options_t* options)
{
    gmp_can_node_context_t* node = find_node(hcan);
    gmp_can_tx_entry_t entry;

    if (node == NULL || !gmp_can_frame_is_valid(frame))
    {
        return GMP_EC_INVALID_PARAM;
    }
    entry.frame = *frame;
    if (options != NULL)
        entry.options = *options;
    else
        memset(&entry.options, 0, sizeof(entry.options));

    gmp_base_enter_critical();
    if (node->tx_count >= node->storage.tx_capacity)
    {
        gmp_base_leave_critical();
        return GMP_EC_DEQUE_FULL;
    }
    if (entry.options.token == 0U)
    {
        entry.options.token = node->next_token++;
        if (node->next_token == 0U)
            node->next_token = 1U;
    }
    node->storage.tx_entries[node->tx_tail] = entry;
    node->tx_tail = (uint16_t)((node->tx_tail + 1U) % node->storage.tx_capacity);
    ++node->tx_count;
    gmp_base_leave_critical();
    gmp_can_service(hcan);
    return GMP_EC_OK;
}

static ec_gt pop_rx(gmp_can_node_context_t* node, gmp_can_rx_event_t* event)
{
    gmp_base_enter_critical();
    if (node->rx_count == 0U)
    {
        gmp_base_leave_critical();
        return GMP_EC_DEQUE_EMPTY;
    }
    *event = node->storage.rx_events[node->rx_head];
    node->rx_head = (uint16_t)((node->rx_head + 1U) % node->storage.rx_capacity);
    --node->rx_count;
    gmp_base_leave_critical();
    return GMP_EC_OK;
}

ec_gt gmp_can_receive(can_halt hcan, gmp_can_rx_event_t* event)
{
    gmp_can_node_context_t* node = find_node(hcan);
    return node == NULL || event == NULL ? GMP_EC_INVALID_PARAM : pop_rx(node, event);
}

ec_gt gmp_can_get_tx_event(can_halt hcan, gmp_can_tx_event_t* event)
{
    gmp_can_node_context_t* node = find_node(hcan);
    if (node == NULL || event == NULL)
        return GMP_EC_INVALID_PARAM;
    gmp_base_enter_critical();
    if (node->tx_event_count == 0U)
    {
        gmp_base_leave_critical();
        return GMP_EC_DEQUE_EMPTY;
    }
    *event = node->storage.tx_events[node->tx_event_head];
    node->tx_event_head = (uint16_t)((node->tx_event_head + 1U) % node->storage.tx_event_capacity);
    --node->tx_event_count;
    gmp_base_leave_critical();
    return GMP_EC_OK;
}

ec_gt gmp_can_get_state_event(can_halt hcan, gmp_can_state_event_t* event)
{
    gmp_can_node_context_t* node = find_node(hcan);
    if (node == NULL || event == NULL)
        return GMP_EC_INVALID_PARAM;
    gmp_base_enter_critical();
    if (node->state_count == 0U)
    {
        gmp_base_leave_critical();
        return GMP_EC_DEQUE_EMPTY;
    }
    *event = node->storage.state_events[node->state_head];
    node->state_head = (uint16_t)((node->state_head + 1U) % node->storage.state_event_capacity);
    --node->state_count;
    gmp_base_leave_critical();
    return GMP_EC_OK;
}

void gmp_can_service(can_halt hcan)
{
    gmp_can_node_context_t* node = find_node(hcan);
    ec_gt status;
    gmp_can_tx_entry_t* entry;
    uint32_t token;

    if (node == NULL)
        return;
    gmp_hal_can_service(hcan);
    while (node->tx_count != 0U)
    {
        entry = &node->storage.tx_entries[node->tx_head];
        token = entry->options.token;
        status = gmp_hal_can_submit_tx(hcan, &entry->frame, token);
        if (status == GMP_EC_BUSY || status == GMP_EC_NOT_READY)
            return;

        gmp_base_enter_critical();
        node->tx_head = (uint16_t)((node->tx_head + 1U) % node->storage.tx_capacity);
        --node->tx_count;
        gmp_base_leave_critical();
        if (status != GMP_EC_OK)
            (void)gmp_can_tx_complete(hcan, token, status, 0U);
    }
}

ec_gt gmp_can_rx_commit(can_halt hcan, const gmp_can_frame_t* frame, const gmp_can_rx_meta_t* metadata)
{
    gmp_can_node_context_t* node = find_node(hcan);
    gmp_can_rx_event_t event;
    if (node == NULL || !gmp_can_frame_is_valid(frame))
        return GMP_EC_INVALID_PARAM;
    event.frame = *frame;
    if (metadata != NULL)
        event.metadata = *metadata;
    else
        memset(&event.metadata, 0, sizeof(event.metadata));
    gmp_base_enter_critical();
    if (node->rx_count >= node->storage.rx_capacity)
    {
        gmp_base_leave_critical();
        return GMP_EC_DEQUE_FULL;
    }
    node->storage.rx_events[node->rx_tail] = event;
    node->rx_tail = (uint16_t)((node->rx_tail + 1U) % node->storage.rx_capacity);
    ++node->rx_count;
    gmp_base_leave_critical();
    return GMP_EC_OK;
}

ec_gt gmp_can_rx_commit_isr(can_halt hcan, const gmp_can_frame_t* frame, const gmp_can_rx_meta_t* metadata)
{
    return gmp_can_rx_commit(hcan, frame, metadata);
}

ec_gt gmp_can_tx_complete(can_halt hcan, uint32_t token, ec_gt status, uint32_t timestamp)
{
    gmp_can_node_context_t* node = find_node(hcan);
    gmp_can_tx_event_t event;
    if (node == NULL || token == 0U)
        return GMP_EC_INVALID_PARAM;
    event.token = token;
    event.status = status;
    event.timestamp = timestamp;
    gmp_base_enter_critical();
    if (node->tx_event_count >= node->storage.tx_event_capacity)
    {
        gmp_base_leave_critical();
        return GMP_EC_DEQUE_FULL;
    }
    node->storage.tx_events[node->tx_event_tail] = event;
    node->tx_event_tail = (uint16_t)((node->tx_event_tail + 1U) % node->storage.tx_event_capacity);
    ++node->tx_event_count;
    gmp_base_leave_critical();
    return GMP_EC_OK;
}

ec_gt gmp_can_tx_complete_isr(can_halt hcan, uint32_t token, ec_gt status, uint32_t timestamp)
{
    return gmp_can_tx_complete(hcan, token, status, timestamp);
}

ec_gt gmp_can_state_commit(can_halt hcan, const gmp_can_state_t* state, uint32_t timestamp)
{
    gmp_can_node_context_t* node = find_node(hcan);
    gmp_can_state_event_t event;
    if (node == NULL || state == NULL)
        return GMP_EC_INVALID_PARAM;
    event.state = *state;
    event.timestamp = timestamp;
    gmp_base_enter_critical();
    if (node->state_count >= node->storage.state_event_capacity)
    {
        gmp_base_leave_critical();
        return GMP_EC_DEQUE_FULL;
    }
    node->storage.state_events[node->state_tail] = event;
    node->state_tail = (uint16_t)((node->state_tail + 1U) % node->storage.state_event_capacity);
    ++node->state_count;
    gmp_base_leave_critical();
    return GMP_EC_OK;
}

ec_gt gmp_can_state_commit_isr(can_halt hcan, const gmp_can_state_t* state, uint32_t timestamp)
{
    return gmp_can_state_commit(hcan, state, timestamp);
}
