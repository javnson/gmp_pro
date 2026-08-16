/** @file nmt_sm.c @brief CANopen NMT state and heartbeat services. */

#include <core/protocol/canopen/nmt_sm.h>

static uint32_t gmp_canopen_saturating_add_u32(uint32_t lhs, uint32_t rhs)
{
    return UINT32_MAX - lhs < rhs ? UINT32_MAX : lhs + rhs;
}

fast_gt gmp_canopen_nmt_init(gmp_canopen_nmt_t* nmt, uint16_t node_id,
                             uint32_t heartbeat_period_ms)
{
    if (nmt == NULL || node_id < GMP_CANOPEN_NODE_ID_MIN ||
        node_id > GMP_CANOPEN_NODE_ID_MAX)
        return 0;
    nmt->node_id = node_id;
    nmt->state = GMP_CANOPEN_NMT_INITIALIZATION;
    nmt->heartbeat_period_ms = heartbeat_period_ms;
    nmt->heartbeat_elapsed_ms = 0U;
    nmt->bootup_pending = 1;
    return 1;
}

gmp_canopen_nmt_event_t gmp_canopen_nmt_apply_command(
    gmp_canopen_nmt_t* nmt, gmp_canopen_nmt_command_t command)
{
    if (nmt == NULL)
        return GMP_CANOPEN_NMT_EVENT_NONE;
    switch (command)
    {
    case GMP_CANOPEN_NMT_CMD_START:
        nmt->state = GMP_CANOPEN_NMT_OPERATIONAL;
        return GMP_CANOPEN_NMT_EVENT_STATE_CHANGED;
    case GMP_CANOPEN_NMT_CMD_STOP:
        nmt->state = GMP_CANOPEN_NMT_STOPPED;
        return GMP_CANOPEN_NMT_EVENT_STATE_CHANGED;
    case GMP_CANOPEN_NMT_CMD_PRE_OPERATIONAL:
        nmt->state = GMP_CANOPEN_NMT_PRE_OPERATIONAL;
        return GMP_CANOPEN_NMT_EVENT_STATE_CHANGED;
    case GMP_CANOPEN_NMT_CMD_RESET_NODE:
        nmt->state = GMP_CANOPEN_NMT_INITIALIZATION;
        nmt->heartbeat_elapsed_ms = 0U;
        nmt->bootup_pending = 1;
        return GMP_CANOPEN_NMT_EVENT_RESET_NODE;
    case GMP_CANOPEN_NMT_CMD_RESET_COMMUNICATION:
        nmt->state = GMP_CANOPEN_NMT_INITIALIZATION;
        nmt->heartbeat_elapsed_ms = 0U;
        nmt->bootup_pending = 1;
        return GMP_CANOPEN_NMT_EVENT_RESET_COMMUNICATION;
    default:
        return GMP_CANOPEN_NMT_EVENT_NONE;
    }
}

gmp_canopen_nmt_event_t gmp_canopen_nmt_receive(
    gmp_canopen_nmt_t* nmt, const gmp_canopen_frame_t* frame)
{
    uint16_t target;
    if (nmt == NULL || frame == NULL || frame->is_extended ||
        frame->is_remote || frame->id != GMP_CANOPEN_COB_NMT ||
        frame->dlc != 2U)
        return GMP_CANOPEN_NMT_EVENT_NONE;
    if (frame->data[1] > GMP_CANOPEN_NODE_ID_MAX)
        return GMP_CANOPEN_NMT_EVENT_NONE;
    target = (uint16_t)frame->data[1];
    if (target != 0U && target != nmt->node_id)
        return GMP_CANOPEN_NMT_EVENT_NONE;
    return gmp_canopen_nmt_apply_command(
        nmt, (gmp_canopen_nmt_command_t)(frame->data[0] & 0xFFU));
}

fast_gt gmp_canopen_nmt_build_heartbeat(const gmp_canopen_nmt_t* nmt,
                                        gmp_canopen_frame_t* heartbeat)
{
    if (nmt == NULL || heartbeat == NULL)
        return 0;
    gmp_canopen_frame_clear(heartbeat);
    heartbeat->id = GMP_CANOPEN_COB_HEARTBEAT + nmt->node_id;
    heartbeat->dlc = 1U;
    heartbeat->data[0] = (gmp_canopen_octet_t)nmt->state;
    return 1;
}

fast_gt gmp_canopen_nmt_tick(gmp_canopen_nmt_t* nmt, uint32_t elapsed_ms,
                             gmp_canopen_frame_t* heartbeat)
{
    if (nmt == NULL || heartbeat == NULL)
        return 0;
    if (nmt->bootup_pending)
    {
        nmt->state = GMP_CANOPEN_NMT_INITIALIZATION;
        if (!gmp_canopen_nmt_build_heartbeat(nmt, heartbeat))
            return 0;
        nmt->bootup_pending = 0;
        nmt->state = GMP_CANOPEN_NMT_PRE_OPERATIONAL;
        nmt->heartbeat_elapsed_ms = 0U;
        return 1;
    }
    if (nmt->heartbeat_period_ms == 0U)
        return 0;
    nmt->heartbeat_elapsed_ms = gmp_canopen_saturating_add_u32(
        nmt->heartbeat_elapsed_ms, elapsed_ms);
    if (nmt->heartbeat_elapsed_ms < nmt->heartbeat_period_ms)
        return 0;
    nmt->heartbeat_elapsed_ms %= nmt->heartbeat_period_ms;
    return gmp_canopen_nmt_build_heartbeat(nmt, heartbeat);
}

fast_gt gmp_canopen_heartbeat_consumer_init(
    gmp_canopen_heartbeat_consumer_t* consumer, uint16_t node_id,
    uint32_t timeout_ms)
{
    if (consumer == NULL || node_id < GMP_CANOPEN_NODE_ID_MIN ||
        node_id > GMP_CANOPEN_NODE_ID_MAX || timeout_ms == 0U)
        return 0;
    consumer->node_id = node_id;
    consumer->state = GMP_CANOPEN_NMT_INITIALIZATION;
    consumer->timeout_ms = timeout_ms;
    consumer->elapsed_ms = 0U;
    consumer->has_heartbeat = 0;
    consumer->timed_out = 0;
    return 1;
}

fast_gt gmp_canopen_heartbeat_consumer_receive(
    gmp_canopen_heartbeat_consumer_t* consumer,
    const gmp_canopen_frame_t* heartbeat)
{
    uint16_t state;
    if (consumer == NULL || heartbeat == NULL || heartbeat->is_extended ||
        heartbeat->is_remote || heartbeat->dlc != 1U ||
        heartbeat->id != GMP_CANOPEN_COB_HEARTBEAT + consumer->node_id)
        return 0;
    state = (uint16_t)(heartbeat->data[0] & 0xFFU);
    if (state != GMP_CANOPEN_NMT_INITIALIZATION &&
        state != GMP_CANOPEN_NMT_STOPPED &&
        state != GMP_CANOPEN_NMT_OPERATIONAL &&
        state != GMP_CANOPEN_NMT_PRE_OPERATIONAL)
        return 0;
    consumer->state = (gmp_canopen_nmt_state_t)state;
    consumer->elapsed_ms = 0U;
    consumer->has_heartbeat = 1;
    consumer->timed_out = 0;
    return 1;
}

fast_gt gmp_canopen_heartbeat_consumer_tick(
    gmp_canopen_heartbeat_consumer_t* consumer, uint32_t elapsed_ms)
{
    if (consumer == NULL)
        return 0;
    consumer->elapsed_ms = gmp_canopen_saturating_add_u32(
        consumer->elapsed_ms, elapsed_ms);
    if (consumer->elapsed_ms >= consumer->timeout_ms)
        consumer->timed_out = 1;
    return consumer->timed_out;
}
