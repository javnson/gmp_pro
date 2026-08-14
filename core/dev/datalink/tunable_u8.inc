/**
 * @file gmp_tunable_u8.inc
 * @brief Tunable parameter service for byte-addressed targets.
 */

static fast_gt tunable_u8_read(const uint8_t* buf, uint16_t len,
                               uint16_t* index, uint8_t* value)
{
    if (*index >= len)
        return 0;
    *value = buf[*index];
    *index = (uint16_t)(*index + 1U);
    return 1;
}

static fast_gt tunable_u8_read_u16(const uint8_t* buf, uint16_t len,
                                   uint16_t* index, uint16_t* value)
{
    if ((uint32_t)*index + 2U > len)
        return 0;
    *value = (uint16_t)buf[*index] | (uint16_t)((uint16_t)buf[*index + 1U] << 8);
    *index = (uint16_t)(*index + 2U);
    return 1;
}

static fast_gt tunable_u8_read_u32(const uint8_t* buf, uint16_t len,
                                   uint16_t* index, uint32_t* value)
{
    if ((uint32_t)*index + 4U > len)
        return 0;
    *value = (uint32_t)buf[*index] |
             ((uint32_t)buf[*index + 1U] << 8) |
             ((uint32_t)buf[*index + 2U] << 16) |
             ((uint32_t)buf[*index + 3U] << 24);
    *index = (uint16_t)(*index + 4U);
    return 1;
}

static size_gt tunable_u8_item_size(gmp_param_type_t type)
{
    return (type == GMP_PARAM_TYPE_U16 || type == GMP_PARAM_TYPE_I16) ? 2U : 4U;
}

/**
 * @brief Reply with one indexed tunable-parameter descriptor.
 * @param ctx Tunable service containing the parameter dictionary.
 */
static void tunable_u8_reply_discovery(gmp_param_tunable_t* ctx)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    uint8_t item_id = 0U;
    uint8_t total = (uint8_t)ctx->dict_size;
    const gmp_param_item_t* item = NULL;
    const char* name = NULL;
    size_t source_name_length = 0U;
    size_gt name_length = 0U;
    size_gt capacity;
    size_gt index;

    if (dl->expected_payload_len > 0U)
        item_id = dl->payload_buf[0];
    if (item_id < total)
        item = &ctx->dict[item_id];

    gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, dl->rx_head.cmd);
    gmp_dev_dl_tx_append_u8(dl, 2U);
    gmp_dev_dl_tx_append_u8(dl, (item == NULL) ? 1U : 0U);
    gmp_dev_dl_tx_append_u8(dl, total);
    gmp_dev_dl_tx_append_u8(dl, item_id);
    if (item != NULL)
    {
        gmp_dev_dl_tx_append_u8(dl, (uint8_t)item->type);
        gmp_dev_dl_tx_append_u8(dl, (uint8_t)item->perm);
        name = item->name;
        capacity = gmp_dev_dl_get_tx_capacity(dl);
        if (name != NULL && capacity > 1U)
        {
            source_name_length = strlen(name);
            if (source_name_length > 63U)
                source_name_length = 63U;
            if (source_name_length > (size_t)(capacity - 1U))
                source_name_length = (size_t)(capacity - 1U);
            name_length = (size_gt)source_name_length;
        }
        gmp_dev_dl_tx_append_u8(dl, (uint8_t)name_length);
        for (index = 0U; index < name_length; ++index)
            gmp_dev_dl_tx_append_u8(dl, (uint8_t)name[index]);
    }
    gmp_dev_dl_tx_ready(dl);
    gmp_dev_dl_msg_handled(dl);
}

void gmp_param_tunable_init(gmp_param_tunable_t* ctx, gmp_datalink_t* dl,
                            uint16_t base_cmd, const gmp_param_item_t* dict,
                            fast16_gt dict_size)
{
    if (ctx == NULL)
        return;
    ctx->dl_ctx = dl;
    ctx->base_cmd = base_cmd;
    ctx->dict = dict;
    ctx->dict_size = (dict_size > 255) ? 255 : dict_size;
}

fast_gt gmp_param_tunable_rx_cb(gmp_param_tunable_t* ctx)
{
    gmp_datalink_t* dl;
    const uint8_t* payload;
    uint16_t index = 0;
    uint16_t len;
    uint8_t request_count;
    size_gt i;

    if (ctx == NULL || ctx->dl_ctx == NULL)
        return 0;
    dl = ctx->dl_ctx;
    if (dl->rx_head.cmd != ctx->base_cmd &&
        dl->rx_head.cmd != (ctx->base_cmd + 1U) &&
        dl->rx_head.cmd != (ctx->base_cmd + GMP_PARAM_DISCOVERY_CMD_OFFSET))
        return 0;

    /* A one-byte payload on base + 1 is an indexed discovery query.
     * A useful write request is longer, so legacy writes remain compatible. */
    if (dl->rx_head.cmd == (ctx->base_cmd + GMP_PARAM_DISCOVERY_CMD_OFFSET) &&
        dl->expected_payload_len == 1U)
    {
        tunable_u8_reply_discovery(ctx);
        return 1;
    }

    payload = dl->payload_buf;
    len = dl->expected_payload_len;
    if (!tunable_u8_read(payload, len, &index, &request_count))
    {
        gmp_dev_dl_reply_nack(dl, 0x0002);
        gmp_dev_dl_msg_handled(dl);
        return 1;
    }

    if (dl->rx_head.cmd == ctx->base_cmd)
    {
        uint8_t valid_count = 0;
        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, dl->rx_head.cmd);
        gmp_dev_dl_tx_append_u8(dl, 0);

        for (i = 0; i < request_count; ++i)
        {
            uint8_t id;
            const gmp_param_item_t* item;
            size_gt item_size;
            uint16_t value16;
            uint32_t value32;
            if (!tunable_u8_read(payload, len, &index, &id))
                break;
            if (id >= ctx->dict_size)
                continue;
            item = &ctx->dict[id];
            item_size = tunable_u8_item_size(item->type);
            if (gmp_dev_dl_get_tx_capacity(dl) < item_size + 1U)
                break;

            gmp_dev_dl_tx_append_u8(dl, id);
            if (item_size == 2U)
            {
                memcpy(&value16, item->addr, sizeof(value16));
                gmp_dev_dl_tx_append_u16(dl, value16);
            }
            else
            {
                memcpy(&value32, item->addr, sizeof(value32));
                gmp_dev_dl_tx_append_u32(dl, value32);
            }
            ++valid_count;
        }
        dl->tx_buf[0] = valid_count;
        gmp_dev_dl_tx_ready(dl);
    }
    else
    {
        uint8_t status = 0;
        for (i = 0; i < request_count; ++i)
        {
            uint8_t id;
            const gmp_param_item_t* item;
            uint16_t value16;
            uint32_t value32;
            if (!tunable_u8_read(payload, len, &index, &id) || id >= ctx->dict_size)
            {
                status = 1;
                break;
            }
            item = &ctx->dict[id];
            if (tunable_u8_item_size(item->type) == 2U)
            {
                if (!tunable_u8_read_u16(payload, len, &index, &value16))
                {
                    status = 1;
                    break;
                }
                if (item->perm == GMP_PARAM_PERM_RW)
                    memcpy(item->addr, &value16, sizeof(value16));
                else
                    status = 1;
            }
            else
            {
                if (!tunable_u8_read_u32(payload, len, &index, &value32))
                {
                    status = 1;
                    break;
                }
                if (item->perm == GMP_PARAM_PERM_RW)
                    memcpy(item->addr, &value32, sizeof(value32));
                else
                    status = 1;
            }
        }
        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, dl->rx_head.cmd);
        gmp_dev_dl_tx_append_u8(dl, status);
        gmp_dev_dl_tx_ready(dl);
    }

    gmp_dev_dl_msg_handled(dl);
    return 1;
}
