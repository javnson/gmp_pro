/**
 * @file gmp_mem_presp_u8.inc
 * @brief Sandboxed memory perspective service for byte-addressed targets.
 */

/** @brief Read an unsigned 8-bit value from a protocol payload. */
static fast_gt mem_u8_read(const gmp_dl_octet_t* buffer, uint16_t length,
                           uint16_t* index, gmp_dl_octet_t* value)
{
    if (*index >= length)
        return 0;
    *value = buffer[*index];
    *index = (uint16_t)(*index + 1U);
    return 1;
}

/** @brief Read a little-endian unsigned 16-bit value from a protocol payload. */
static fast_gt mem_u8_read_u16(const gmp_dl_octet_t* buffer, uint16_t length,
                               uint16_t* index, uint16_t* value)
{
    if ((uint32_t)*index + 2U > length)
        return 0;
    *value = (uint16_t)buffer[*index] |
             (uint16_t)((uint16_t)buffer[*index + 1U] << 8);
    *index = (uint16_t)(*index + 2U);
    return 1;
}

/** @brief Read a little-endian unsigned 32-bit value from a protocol payload. */
static fast_gt mem_u8_read_u32(const gmp_dl_octet_t* buffer, uint16_t length,
                               uint16_t* index, uint32_t* value)
{
    if ((uint32_t)*index + 4U > length)
        return 0;
    *value = (uint32_t)buffer[*index] |
             ((uint32_t)buffer[*index + 1U] << 8) |
             ((uint32_t)buffer[*index + 2U] << 16) |
             ((uint32_t)buffer[*index + 3U] << 24);
    *index = (uint16_t)(*index + 4U);
    return 1;
}

/**
 * @brief Validate a byte-addressed request against the configured sandbox.
 * @param ctx Memory perspective context.
 * @param address First requested byte address.
 * @param total_bytes Number of requested bytes.
 * @param is_write Nonzero for a write request.
 * @return Nonzero when the complete request is inside one permitted region.
 */
static fast_gt mem_u8_sandbox_allows(const gmp_mem_persp_t* ctx, uintptr_t address,
                                     uint32_t total_bytes, fast_gt is_write)
{
    fast16_gt i;
    uintptr_t request_end;
    if (ctx->regions == NULL || ctx->region_count <= 0 || total_bytes == 0U)
        return 0;
    if (address > UINTPTR_MAX - total_bytes)
        return 0;
    request_end = address + total_bytes;

    for (i = 0; i < ctx->region_count; ++i)
    {
        const gmp_mem_region_t* region = &ctx->regions[i];
        uintptr_t region_start = (uintptr_t)region->base_addr;
        uintptr_t region_end;
        if (region_start > UINTPTR_MAX - region->byte_length)
            continue;
        region_end = region_start + region->byte_length;
        if (address >= region_start && request_end <= region_end)
        {
            if (is_write && region->perm != GMP_MEM_PERM_RW)
                return 0;
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Append one native memory item in little-endian wire order.
 * @param dl Bound Data Link context.
 * @param address Native byte address.
 * @param item_size Item size in bytes; supported values are 1, 2, and 4.
 */
static void mem_u8_append_item(gmp_datalink_t* dl, uintptr_t address, gmp_dl_octet_t item_size)
{
    if (item_size == 1U)
    {
        gmp_dl_octet_t value;
        memcpy(&value, (const void*)address, sizeof(value));
        gmp_dev_dl_tx_append_u8(dl, value);
    }
    else if (item_size == 2U)
    {
        uint16_t value;
        memcpy(&value, (const void*)address, sizeof(value));
        gmp_dev_dl_tx_append_u16(dl, value);
    }
    else
    {
        uint32_t value;
        memcpy(&value, (const void*)address, sizeof(value));
        gmp_dev_dl_tx_append_u32(dl, value);
    }
}

void gmp_mem_persp_init(gmp_mem_persp_t* ctx, gmp_datalink_t* dl, uint16_t base_cmd,
                        const gmp_mem_region_t* regions, fast16_gt region_count)
{
    if (ctx == NULL)
        return;
    ctx->dl_ctx = dl;
    ctx->base_cmd = base_cmd;
    ctx->regions = regions;
    ctx->region_count = region_count;
}

fast_gt gmp_mem_persp_rx_cb(gmp_mem_persp_t* ctx)
{
    gmp_datalink_t* dl;
    const gmp_dl_octet_t* payload;
    uint16_t index = 0;
    uint16_t item_count;
    uint32_t address32;
    uint32_t total_bytes;
    uintptr_t address;
    gmp_dl_octet_t item_size;
    gmp_dl_octet_t status = 0;
    uint16_t item;

    if (ctx == NULL || ctx->dl_ctx == NULL)
        return 0;
    dl = ctx->dl_ctx;
    if (dl->rx_head.cmd != ctx->base_cmd && dl->rx_head.cmd != (ctx->base_cmd + 1U))
        return 0;

    payload = dl->payload_buf;
    if (!mem_u8_read_u32(payload, dl->expected_payload_len, &index, &address32) ||
        !mem_u8_read(payload, dl->expected_payload_len, &index, &item_size) ||
        !mem_u8_read_u16(payload, dl->expected_payload_len, &index, &item_count))
    {
        gmp_dev_dl_reply_nack(dl, 0x0002);
        gmp_dev_dl_msg_handled(dl);
        return 1;
    }

    if (item_size != 1U && item_size != 2U && item_size != 4U)
        status = 1;
    if (status == 0U &&
        (item_count == 0U || (uint32_t)item_count > UINT32_MAX / item_size))
        status = 1;
    total_bytes = (status == 0U) ? (uint32_t)item_count * item_size : 0U;
    address = (uintptr_t)address32;

    if (status == 0U && !mem_u8_sandbox_allows(
            ctx, address, total_bytes, dl->rx_head.cmd == (ctx->base_cmd + 1U)))
        status = 1;

    if (dl->rx_head.cmd == ctx->base_cmd)
    {
        if (status == 0U && total_bytes > (uint32_t)(GMP_DL_MTU - 1U))
            status = 2;
        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, dl->rx_head.cmd);
        gmp_dev_dl_tx_append_u8(dl, status);
        if (status == 0U)
        {
            for (item = 0; item < item_count; ++item)
                mem_u8_append_item(dl, address + (uintptr_t)item * item_size, item_size);
        }
        gmp_dev_dl_tx_ready(dl);
    }
    else
    {
        if (status == 0U && (uint32_t)index + total_bytes > dl->expected_payload_len)
            status = 1;
        if (status == 0U)
        {
            for (item = 0; item < item_count; ++item)
            {
                uintptr_t item_address = address + (uintptr_t)item * item_size;
                if (item_size == 1U)
                {
                    gmp_dl_octet_t value;
                    mem_u8_read(payload, dl->expected_payload_len, &index, &value);
                    memcpy((void*)item_address, &value, sizeof(value));
                }
                else if (item_size == 2U)
                {
                    uint16_t value;
                    mem_u8_read_u16(payload, dl->expected_payload_len, &index, &value);
                    memcpy((void*)item_address, &value, sizeof(value));
                }
                else
                {
                    uint32_t value;
                    mem_u8_read_u32(payload, dl->expected_payload_len, &index, &value);
                    memcpy((void*)item_address, &value, sizeof(value));
                }
            }
        }
        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, dl->rx_head.cmd);
        gmp_dev_dl_tx_append_u8(dl, status);
        gmp_dev_dl_tx_ready(dl);
    }

    gmp_dev_dl_msg_handled(dl);
    return 1;
}
