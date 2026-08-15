/**
 * @file gmp_mem_presp_u16.inc
 * @brief Memory Perspective implementation for 16-bit-addressed targets.
 */

/* Internal payload decoding helpers. */

static inline fast16_gt mem_unpack_u8(const byte_gt* buf, uint16_t* idx, uint16_t max_len)
{
    if (*idx >= max_len)
        return 0;
    fast16_gt val = buf[*idx] & 0xFF;
    *idx += 1;
    return val;
}

static inline uint32_t mem_unpack_u32(const byte_gt* buf, uint16_t* idx, uint16_t max_len)
{
    if (*idx + 4 > max_len)
        return 0;
    uint32_t val = ((uint32_t)(buf[*idx] & 0xFF)) | (((uint32_t)(buf[*idx + 1] & 0xFF)) << 8) |
                   (((uint32_t)(buf[*idx + 2] & 0xFF)) << 16) | (((uint32_t)(buf[*idx + 3] & 0xFF)) << 24);
    *idx += 4;
    return val;
}

/* Memory Perspective implementation. */

void gmp_mem_persp_init(gmp_mem_persp_t* ctx, gmp_datalink_t* dl, uint16_t base_cmd, const gmp_mem_region_t* regions,
                        fast16_gt region_count)
{
    if (ctx == NULL)
        return;
    gmp_dl_facility_init(&ctx->facility, GMP_DL_FACILITY_MEMORY,
                         base_cmd, 2U,
                         gmp_mem_persp_facility_dispatch, ctx);
    ctx->dl_ctx = dl;
    ctx->base_cmd = base_cmd;
    ctx->regions = regions;
    ctx->region_count = (region_count > 255) ? 255 : region_count;
}

/**
 * @brief Reply with one indexed memory-region descriptor.
 * @param ctx Memory Perspective service containing the region table.
 */
static void mem_u16_reply_discovery(gmp_mem_persp_t* ctx)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    fast16_gt region_id = 0;
    fast16_gt total = ctx->region_count;
    const gmp_mem_region_t* region = NULL;
    const char* name = NULL;
    size_gt name_length = 0U;
    size_gt capacity;
    size_gt index;

    if (dl->expected_payload_len > 0U)
        region_id = dl->payload_buf[0] & 0xFF;
    if (region_id < total)
        region = &ctx->regions[region_id];

    gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, dl->rx_head.cmd);
    gmp_dev_dl_tx_append_u8(dl, 2U);
    gmp_dev_dl_tx_append_u8(dl, (region == NULL) ? 1U : 0U);
    gmp_dev_dl_tx_append_u8(dl, total);
    gmp_dev_dl_tx_append_u8(dl, region_id);
    if (region != NULL)
    {
        uint32_t wire_address = (uint32_t)region->base_addr * GMP_PORT_DATA_SIZE_PER_BYTES;
        gmp_dev_dl_tx_append_u32(dl, wire_address);
        gmp_dev_dl_tx_append_u32(dl, region->byte_length);
        gmp_dev_dl_tx_append_u8(dl, region->perm);

        name = region->name;
        capacity = gmp_dev_dl_get_tx_capacity(dl);
        if (name != NULL && capacity > 1U)
        {
            name_length = strlen(name);
            if (name_length > capacity - 1U)
                name_length = capacity - 1U;
            if (name_length > 63U)
                name_length = 63U;
        }
        gmp_dev_dl_tx_append_u8(dl, name_length);
        for (index = 0U; index < name_length; ++index)
            gmp_dev_dl_tx_append_u8(dl, name[index]);
    }
    gmp_dev_dl_tx_ready(dl);
    gmp_dev_dl_msg_handled(dl);
}

/**
 * @brief Check whether an access is fully contained in an allowed region.
 * @param ctx Memory Perspective instance containing the region table.
 * @param addr Protocol byte address of the first requested item.
 * @param total_bytes Total requested byte count.
 * @param is_write Nonzero for a write access.
 * @return Nonzero when the complete access is permitted; otherwise zero.
 */
static fast_gt check_sandbox_validity(gmp_mem_persp_t* ctx, uint32_t addr, uint32_t total_bytes, fast16_gt is_write)
{
    size_gt i;

    for (i = 0; i < ctx->region_count; i++)
    {
        const gmp_mem_region_t* reg = &ctx->regions[i];

        /* Convert the native word address into the protocol byte address. */
        uint32_t safe_start_addr = ((uint32_t)reg->base_addr) * GMP_PORT_DATA_SIZE_PER_BYTES;

        /* Require the complete request to remain inside one region. */
        if (addr >= safe_start_addr && (addr + total_bytes) <= (safe_start_addr + reg->byte_length))
        {
            if (is_write && reg->perm != GMP_MEM_PERM_RW)
                return 0;
            return 1;
        }
    }
    return 0;
}

fast_gt gmp_mem_persp_rx_cb(gmp_mem_persp_t* ctx)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    uint16_t cmd = dl->rx_head.cmd;

    if (cmd != ctx->base_cmd && cmd != (ctx->base_cmd + 1) &&
        cmd != (ctx->base_cmd + GMP_MEM_DISCOVERY_CMD_OFFSET))
        return 0;

    /* A one-unit payload on base + 1 is an indexed discovery query.
     * Normal write requests are longer, so legacy writes remain unambiguous
     * without consuming another command identifier. */
    if (cmd == (ctx->base_cmd + GMP_MEM_DISCOVERY_CMD_OFFSET) &&
        dl->expected_payload_len == 1U)
    {
        mem_u16_reply_discovery(ctx);
        return 1;
    }

    uint16_t len = dl->expected_payload_len;
    uint16_t idx = 0;
    const byte_gt* pld = dl->payload_buf;

    /* Decode [byte address:4][item size:1][item count:2]. */
    uint32_t target_addr = mem_unpack_u32(pld, &idx, len);
    fast16_gt item_size = mem_unpack_u8(pld, &idx, len);
    uint16_t item_count = mem_unpack_u8(pld, &idx, len) | (mem_unpack_u8(pld, &idx, len) << 8);

    uint32_t total_bytes = (uint32_t)item_size * item_count;
    byte_gt status = 0;

    size_gt c;

    /* Read request. */
    if (cmd == ctx->base_cmd)
    {
        if (!check_sandbox_validity(ctx, target_addr, total_bytes, 0))
        {
            status = 1;
        }

        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, cmd);
        gmp_dev_dl_tx_append_u8(dl, status);

        if (status == 0)
        {
            for (c = 0; c < item_count; c++)
            {
                uint32_t byte_addr = target_addr + c * item_size;

/* Translate protocol byte addresses into native DSP word addresses. */
#if GMP_PORT_DATA_SIZE_PER_BYTES == 2
                uint32_t native_addr = byte_addr >> 1;
                if (item_size == 1)
                {
                    uint16_t word = *((uint16_t*)native_addr);
                    gmp_dev_dl_tx_append_u8(dl, (byte_addr & 1) ? (word >> 8) : (word & 0xFF));
                }
                else if (item_size == 2)
                {
                    uint16_t word = *((uint16_t*)native_addr);
                    gmp_dev_dl_tx_append_u8(dl, word & 0xFF);
                    gmp_dev_dl_tx_append_u8(dl, (word >> 8) & 0xFF);
                }
                else if (item_size == 4)
                {
                    uint32_t dword = *((uint32_t*)native_addr);
                    gmp_dev_dl_tx_append_u8(dl, dword & 0xFF);
                    gmp_dev_dl_tx_append_u8(dl, (dword >> 8) & 0xFF);
                    gmp_dev_dl_tx_append_u8(dl, (dword >> 16) & 0xFF);
                    gmp_dev_dl_tx_append_u8(dl, (dword >> 24) & 0xFF);
                }
#else /* Byte-addressed compatibility path. */
                uint32_t native_addr = byte_addr;
                if (item_size == 1)
                {
                    gmp_dev_dl_tx_append_u8(dl, *((byte_gt*)native_addr));
                }
                else if (item_size == 2)
                {
                    uint16_t word = *((uint16_t*)native_addr);
                    gmp_dev_dl_tx_append_u8(dl, word & 0xFF);
                    gmp_dev_dl_tx_append_u8(dl, (word >> 8) & 0xFF);
                }
                else if (item_size == 4)
                {
                    uint32_t dword = *((uint32_t*)native_addr);
                    gmp_dev_dl_tx_append_u8(dl, dword & 0xFF);
                    gmp_dev_dl_tx_append_u8(dl, (dword >> 8) & 0xFF);
                    gmp_dev_dl_tx_append_u8(dl, (dword >> 16) & 0xFF);
                    gmp_dev_dl_tx_append_u8(dl, (dword >> 24) & 0xFF);
                }
#endif
            }
        }

        gmp_dev_dl_tx_ready(dl);
        gmp_dev_dl_msg_handled(dl);
        return 1;
    }

    /* Write request. */
    else if (cmd == (ctx->base_cmd + 1))
    {
        if (!check_sandbox_validity(ctx, target_addr, total_bytes, 1))
        {
            status = 1;
        }
        else
        {
            for (c = 0; c < item_count; c++)
            {
                uint32_t byte_addr = target_addr + c * item_size;

/* Translate protocol byte addresses into native DSP word addresses. */
#if GMP_PORT_DATA_SIZE_PER_BYTES == 2
                uint32_t native_addr = byte_addr >> 1;
                if (item_size == 1)
                {
                    byte_gt val = (byte_gt)mem_unpack_u8(pld, &idx, len);
                    uint16_t word = *((uint16_t*)native_addr);
                    /* Update one octet with a native 16-bit read-modify-write. */
                    if (byte_addr & 1)
                        word = (word & 0x00FF) | ((uint16_t)val << 8);
                    else
                        word = (word & 0xFF00) | val;
                    *((uint16_t*)native_addr) = word;
                }
                else if (item_size == 2)
                {
                    uint16_t word = mem_unpack_u8(pld, &idx, len) | (mem_unpack_u8(pld, &idx, len) << 8);
                    *((uint16_t*)native_addr) = word;
                }
                else if (item_size == 4)
                {
                    uint32_t dword = mem_unpack_u8(pld, &idx, len) | (mem_unpack_u8(pld, &idx, len) << 8) |
                                     ((uint32_t)mem_unpack_u8(pld, &idx, len) << 16) |
                                     ((uint32_t)mem_unpack_u8(pld, &idx, len) << 24);
                    *((uint32_t*)native_addr) = dword;
                }
#else /* Byte-addressed compatibility path. */
                uint32_t native_addr = byte_addr;
                if (item_size == 1)
                {
                    *((byte_gt*)native_addr) = (byte_gt)mem_unpack_u8(pld, &idx, len);
                }
                else if (item_size == 2)
                {
                    *((uint16_t*)native_addr) = mem_unpack_u8(pld, &idx, len) | (mem_unpack_u8(pld, &idx, len) << 8);
                }
                else if (item_size == 4)
                {
                    *((uint32_t*)native_addr) = mem_unpack_u8(pld, &idx, len) | (mem_unpack_u8(pld, &idx, len) << 8) |
                                                ((uint32_t)mem_unpack_u8(pld, &idx, len) << 16) |
                                                ((uint32_t)mem_unpack_u8(pld, &idx, len) << 24);
                }
#endif
            }
        }

        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, cmd);
        gmp_dev_dl_tx_append_u8(dl, status);
        gmp_dev_dl_tx_ready(dl);
        gmp_dev_dl_msg_handled(dl);
        return 1;
    }

    return 0;
}
