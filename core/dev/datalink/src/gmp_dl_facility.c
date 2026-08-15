/**
 * @file gmp_dl_facility.c
 * @brief Intrusive Data Link submodule registration and automatic dispatch.
 */

#include <core/dev/datalink/datalink.h>

static fast_gt gmp_dl_command_ranges_overlap(uint16_t lhs_base,
                                              uint16_t lhs_count,
                                              uint16_t rhs_base,
                                              uint16_t rhs_count)
{
    uint32_t lhs_end = (uint32_t)lhs_base + lhs_count;
    uint32_t rhs_end = (uint32_t)rhs_base + rhs_count;
    return ((uint32_t)lhs_base < rhs_end && (uint32_t)rhs_base < lhs_end) ? 1 : 0;
}

static fast_gt gmp_dl_facility_list_is_valid(const gmp_datalink_t* ctx)
{
    size_gt actual_count;
    if (ctx == NULL ||
        !gmp_list_validate(&ctx->facility_list, GMP_DL_MAX_FACILITIES))
        return 0;
    actual_count = gmp_list_count(&ctx->facility_list, GMP_DL_MAX_FACILITIES);
    return actual_count == ctx->facility_count ? 1 : 0;
}

void gmp_dl_facility_init(gmp_dl_facility_t* facility,
                          gmp_dl_facility_type_t type,
                          uint16_t base_cmd, uint16_t command_count,
                          gmp_dl_facility_rx_cb_t dispatch, void* owner)
{
    if (facility == NULL)
        return;
    gmp_list_init(&facility->list);
    facility->type = type;
    facility->base_cmd = base_cmd;
    facility->command_count = command_count;
    facility->dispatch = dispatch;
    facility->owner = owner;
}

static fast_gt gmp_dl_echo_alias_dispatch(gmp_dl_facility_t* facility,
                                           gmp_datalink_t* datalink)
{
    GMP_UNUSED_VAR(facility);
    if (datalink == NULL)
        return 0;
    gmp_dev_dl_tx_request(datalink, datalink->rx_head.seq_id, GMP_DL_CMD_ECHO,
                          datalink->expected_payload_len,
                          datalink->payload_buf);
    gmp_dev_dl_msg_handled(datalink);
    return 1;
}

void gmp_dev_dl_init_echo_alias(gmp_dl_facility_t* facility, uint16_t command)
{
    gmp_dl_facility_init(facility, GMP_DL_FACILITY_USER, command, 1U,
                         gmp_dl_echo_alias_dispatch, NULL);
}

fast_gt gmp_dev_dl_append_facility(gmp_datalink_t* ctx,
                                   gmp_dl_facility_t* facility)
{
    gmp_list* cursor;
    if (ctx == NULL || facility == NULL ||
        facility->type == GMP_DL_FACILITY_INVALID ||
        facility->command_count == 0U || facility->dispatch == NULL ||
        facility->base_cmd > 0xFFU ||
        (uint32_t)facility->base_cmd + facility->command_count > 0x100U ||
        gmp_dl_command_ranges_overlap(facility->base_cmd,
                                      facility->command_count,
                                      GMP_DL_CMD_INFO, 1U) ||
        !gmp_list_is_detached(&facility->list) ||
        ctx->facility_count >= GMP_DL_MAX_FACILITIES ||
        !gmp_dl_facility_list_is_valid(ctx))
        return 0;

    cursor = ctx->facility_list.next;
    while (cursor != &ctx->facility_list)
    {
        gmp_dl_facility_t* registered =
            GMP_CONTAINER_OF(cursor, gmp_dl_facility_t, list);
        if (gmp_dl_command_ranges_overlap(facility->base_cmd,
                                          facility->command_count,
                                          registered->base_cmd,
                                          registered->command_count))
            return 0;
        cursor = cursor->next;
    }

    if (!gmp_list_push_tail(&ctx->facility_list, &facility->list))
        return 0;
    ctx->facility_count++;
    return 1;
}

fast_gt gmp_dev_dl_remove_facility(gmp_datalink_t* ctx,
                                   gmp_dl_facility_t* facility)
{
    gmp_list* cursor;
    if (ctx == NULL || facility == NULL || ctx->facility_count == 0U ||
        !gmp_dl_facility_list_is_valid(ctx))
        return 0;

    cursor = ctx->facility_list.next;
    while (cursor != &ctx->facility_list)
    {
        if (cursor == &facility->list)
        {
            if (!gmp_list_remove(cursor))
                return 0;
            ctx->facility_count--;
            return 1;
        }
        cursor = cursor->next;
    }
    return 0;
}

static void gmp_dev_dl_reply_info(gmp_datalink_t* ctx)
{
    gmp_list* cursor;
    gmp_dev_dl_tx_request_cmd(ctx, ctx->rx_head.seq_id, GMP_DL_CMD_INFO);
    gmp_dev_dl_tx_append_u8(ctx, GMP_DL_INFO_PROTOCOL_VERSION);
    gmp_dev_dl_tx_append_u8(ctx, GMP_PORT_DATA_SIZE_PER_BYTES);
    gmp_dev_dl_tx_append_u8(ctx, sizeof(byte_gt));
    gmp_dev_dl_tx_append_u8(ctx, GMP_PORT_DATA_SIZE_PER_BYTES * 8U);
    gmp_dev_dl_tx_append_u8(ctx, (byte_gt)ctx->facility_count);

    cursor = ctx->facility_list.next;
    while (cursor != &ctx->facility_list)
    {
        gmp_dl_facility_t* facility =
            GMP_CONTAINER_OF(cursor, gmp_dl_facility_t, list);
        gmp_dev_dl_tx_append_u8(ctx, (byte_gt)facility->type);
        gmp_dev_dl_tx_append_u8(ctx, (byte_gt)facility->base_cmd);
        gmp_dev_dl_tx_append_u8(ctx, (byte_gt)facility->command_count);
        cursor = cursor->next;
    }
    gmp_dev_dl_tx_ready(ctx);
    gmp_dev_dl_msg_handled(ctx);
}

fast_gt gmp_dev_dl_dispatch_rx(gmp_datalink_t* ctx)
{
    gmp_list* cursor;
    if (ctx == NULL)
        return 0;

    if (!gmp_dl_facility_list_is_valid(ctx))
    {
        gmp_dev_dl_reply_nack(ctx, 2U);
        return 1;
    }

    if (ctx->rx_head.cmd == GMP_DL_CMD_INFO)
    {
        gmp_dev_dl_reply_info(ctx);
        return 1;
    }

    cursor = ctx->facility_list.next;
    while (cursor != &ctx->facility_list)
    {
        gmp_dl_facility_t* facility =
            GMP_CONTAINER_OF(cursor, gmp_dl_facility_t, list);
        uint32_t end = (uint32_t)facility->base_cmd + facility->command_count;
        if ((uint32_t)ctx->rx_head.cmd >= facility->base_cmd &&
            (uint32_t)ctx->rx_head.cmd < end)
        {
            if (facility->dispatch(facility, ctx))
                return 1;
            break;
        }
        cursor = cursor->next;
    }

    gmp_dev_dl_default_rx_handler(ctx);
    return ctx->flag_reply_handled;
}
