#include <gmp_core.h>

#include <core/dev/tunable.h>
#include <string.h>

// =========================================================
// MODULE 1: VARIABLE TUNABLE
// =========================================================

void gmp_tunable_var_init(gmp_tunable_var_t* ctx, gmp_datalink_t* dl_ctx, uint16_t base_cmd,
                          const gmp_var_entry_t* dict, uint16_t dict_size)
{
    if (!ctx)
        return;
    ctx->dl_ctx = dl_ctx;
    ctx->base_cmd = base_cmd;
    ctx->static_dict = dict;
    ctx->static_dict_size = dict_size;
}

fast_gt gmp_tunable_var_rx_cb(gmp_tunable_var_t* ctx, uint16_t target_id, uint16_t cmd, const data_gt* payload,
                              size_gt len)
{
    if (!ctx || cmd < ctx->base_cmd)
        return GMP_TUNABLE_PASS;

    uint16_t offset = cmd - ctx->base_cmd;

    // Strict Offset Filtering: Var module only handles 0, 2, 4
    if (offset != GMP_TUNABLE_OFFSET_INFO_REQ && offset != GMP_TUNABLE_OFFSET_VAR_READ_REQ &&
        offset != GMP_TUNABLE_OFFSET_VAR_WRITE_REQ)
    {
        return GMP_TUNABLE_PASS;
    }

    if (len < 2)
        return GMP_TUNABLE_PASS;

    uint16_t var_id = (payload[0] & 0xFF) | ((payload[1] & 0xFF) << 8);
    const gmp_var_entry_t* entry = NULL;

    if (ctx->static_dict && var_id < ctx->static_dict_size)
    {
        entry = &ctx->static_dict[var_id];
    }

    switch (offset)
    {
    case GMP_TUNABLE_OFFSET_INFO_REQ:
        if (entry && entry->var_ptr)
        {
            data_gt tx_payload[8];
            uint32_t addr = (uint32_t)((uintptr_t)entry->var_ptr);
            tx_payload[0] = var_id & 0xFF;
            tx_payload[1] = (var_id >> 8) & 0xFF;
            tx_payload[2] = addr & 0xFF;
            tx_payload[3] = (addr >> 8) & 0xFF;
            tx_payload[4] = (addr >> 16) & 0xFF;
            tx_payload[5] = (addr >> 24) & 0xFF;
            tx_payload[6] = (data_gt)entry->var_type;
            tx_payload[7] = (data_gt)entry->access;
            gmp_datalink_send(ctx->dl_ctx, target_id, ctx->base_cmd + GMP_TUNABLE_OFFSET_INFO_ACK, tx_payload, 8);
        }
        return GMP_TUNABLE_HANDLED;

    case GMP_TUNABLE_OFFSET_VAR_READ_REQ:
        if (entry && entry->var_ptr)
        {
            data_gt tx_payload[6];
            tx_payload[0] = var_id & 0xFF;
            tx_payload[1] = (var_id >> 8) & 0xFF;
            size_gt tx_len = 2;
            switch (entry->var_type)
            {
            case GMP_VAR_TYPE_UINT16:
            case GMP_VAR_TYPE_INT16: {
                uint16_t val = *((uint16_t*)entry->var_ptr);
                tx_payload[tx_len++] = val & 0xFF;
                tx_payload[tx_len++] = (val >> 8) & 0xFF;
                break;
            }
            case GMP_VAR_TYPE_UINT32:
            case GMP_VAR_TYPE_INT32:
            case GMP_VAR_TYPE_FLOAT: {
                uint32_t val = *((uint32_t*)entry->var_ptr);
                tx_payload[tx_len++] = val & 0xFF;
                tx_payload[tx_len++] = (val >> 8) & 0xFF;
                tx_payload[tx_len++] = (val >> 16) & 0xFF;
                tx_payload[tx_len++] = (val >> 24) & 0xFF;
                break;
            }
            }
            gmp_datalink_send(ctx->dl_ctx, target_id, ctx->base_cmd + GMP_TUNABLE_OFFSET_VAR_READ_ACK, tx_payload,
                              tx_len);
        }
        return GMP_TUNABLE_HANDLED;

    case GMP_TUNABLE_OFFSET_VAR_WRITE_REQ: {
        uint32_t status = GMP_EC_OK;
        if (!entry || !entry->var_ptr)
            status = GMP_EC_TUNABLE_ERR_NO_ID;
        else if (entry->access == GMP_TUNABLE_ACCESS_RO)
            status = GMP_EC_TUNABLE_ERR_RO;
        else
        {
            switch (entry->var_type)
            {
            case GMP_VAR_TYPE_UINT16:
            case GMP_VAR_TYPE_INT16:
                if (len < 4)
                    status = GMP_EC_TUNABLE_ERR_BAD_LEN;
                else
                    *((uint16_t*)entry->var_ptr) = (payload[2] & 0xFF) | ((payload[3] & 0xFF) << 8);
                break;
            case GMP_VAR_TYPE_UINT32:
            case GMP_VAR_TYPE_INT32:
            case GMP_VAR_TYPE_FLOAT:
                if (len < 6)
                    status = GMP_EC_TUNABLE_ERR_BAD_LEN;
                else
                    *((uint32_t*)entry->var_ptr) = (payload[2] & 0xFF) | ((payload[3] & 0xFF) << 8) |
                                                   ((payload[4] & 0xFF) << 16) | ((payload[5] & 0xFF) << 24);
                break;
            }
        }
        data_gt tx_payload[6];
        tx_payload[0] = var_id & 0xFF;
        tx_payload[1] = (var_id >> 8) & 0xFF;
        tx_payload[2] = status & 0xFF;
        tx_payload[3] = (status >> 8) & 0xFF;
        tx_payload[4] = (status >> 16) & 0xFF;
        tx_payload[5] = (status >> 24) & 0xFF;

        gmp_datalink_send(ctx->dl_ctx, target_id, ctx->base_cmd + GMP_TUNABLE_OFFSET_VAR_WRITE_ACK, tx_payload, 6);
        return GMP_TUNABLE_HANDLED;
    }
    default:
        return GMP_TUNABLE_PASS;
    }
}

// =========================================================
// MODULE 2: MEMORY TUNABLE
// =========================================================

void gmp_tunable_mem_init(gmp_tunable_mem_t* ctx, gmp_datalink_t* dl_ctx, uint16_t base_cmd,
                          const gmp_mem_region_t* whitelist, uint16_t whitelist_size)
{
    if (!ctx)
        return;
    ctx->dl_ctx = dl_ctx;
    ctx->base_cmd = base_cmd;
    ctx->whitelist = whitelist;
    ctx->whitelist_size = whitelist_size;
}

static uint32_t check_mem_whitelist(gmp_tunable_mem_t* ctx, uint32_t req_addr, uint32_t req_len_mau, fast_gt is_write)
{
    if (!ctx || !ctx->whitelist)
        return GMP_EC_TUNABLE_ERR_OOB;
    for (uint16_t i = 0; i < ctx->whitelist_size; i++)
    {
        uint32_t start = ctx->whitelist[i].start_addr;
        uint32_t end = start + ctx->whitelist[i].size;
        if (req_addr >= start && (req_addr + req_len_mau) <= end)
        {
            if (is_write && ctx->whitelist[i].access == GMP_TUNABLE_ACCESS_RO)
                return GMP_EC_TUNABLE_ERR_RO;
            return GMP_EC_OK;
        }
    }
    return GMP_EC_TUNABLE_ERR_OOB;
}

fast_gt gmp_tunable_mem_rx_cb(gmp_tunable_mem_t* ctx, uint16_t target_id, uint16_t cmd, const data_gt* payload,
                              size_gt len)
{
    if (!ctx || cmd < ctx->base_cmd)
        return GMP_TUNABLE_PASS;

    uint16_t offset = cmd - ctx->base_cmd;

    // Strict Offset Filtering: Mem module only handles 6, 8
    if (offset != GMP_TUNABLE_OFFSET_MEM_READ_REQ && offset != GMP_TUNABLE_OFFSET_MEM_WRITE_REQ)
    {
        return GMP_TUNABLE_PASS;
    }

    if (len < 6)
        return GMP_TUNABLE_PASS; // Require Addr(4) + Len(2)

    uint32_t req_addr =
        (payload[0] & 0xFF) | ((payload[1] & 0xFF) << 8) | ((payload[2] & 0xFF) << 16) | ((payload[3] & 0xFF) << 24);
    uint16_t req_len_mau = (payload[4] & 0xFF) | ((payload[5] & 0xFF) << 8);

    switch (offset)
    {
    case GMP_TUNABLE_OFFSET_MEM_READ_REQ: {
        uint32_t status = check_mem_whitelist(ctx, req_addr, req_len_mau, 0);

        if ((req_len_mau * GMP_PORT_DATA_SIZE_PER_BYTES) > (GMP_DL_MTU - 10))
        {
            status = GMP_EC_TUNABLE_ERR_MTU;
            req_len_mau = 0;
        }

        data_gt tx_payload[GMP_DL_MTU];
        tx_payload[0] = status & 0xFF;
        tx_payload[1] = (status >> 8) & 0xFF;
        tx_payload[2] = (status >> 16) & 0xFF;
        tx_payload[3] = (status >> 24) & 0xFF;
        for (int k = 0; k < 6; k++)
            tx_payload[4 + k] = payload[k]; // Echo Addr & Len

        size_gt tx_idx = 10;

        if (status == GMP_EC_OK)
        {
            data_gt* ptr = (data_gt*)((uintptr_t)req_addr);
            for (uint16_t i = 0; i < req_len_mau; i++)
            {
                data_gt val = ptr[i];
                for (int b = 0; b < GMP_PORT_DATA_SIZE_PER_BYTES; b++)
                {
                    tx_payload[tx_idx++] = (val >> (b * 8)) & 0xFF;
                }
            }
        }
        gmp_datalink_send(ctx->dl_ctx, target_id, ctx->base_cmd + GMP_TUNABLE_OFFSET_MEM_READ_ACK, tx_payload, tx_idx);
        return GMP_TUNABLE_HANDLED;
    }

    case GMP_TUNABLE_OFFSET_MEM_WRITE_REQ: {
        uint32_t status = check_mem_whitelist(ctx, req_addr, req_len_mau, 1);

        if (len < (6 + (req_len_mau * GMP_PORT_DATA_SIZE_PER_BYTES)))
        {
            status = GMP_EC_TUNABLE_ERR_MTU;
        }

        if (status == GMP_EC_OK)
        {
            data_gt* ptr = (data_gt*)((uintptr_t)req_addr);
            size_gt p_idx = 6;
            for (uint16_t i = 0; i < req_len_mau; i++)
            {
                data_gt val = 0;
                for (int b = 0; b < GMP_PORT_DATA_SIZE_PER_BYTES; b++)
                {
                    val |= ((data_gt)(payload[p_idx++] & 0xFF) << (b * 8));
                }
                ptr[i] = val;
            }
        }

        data_gt tx_payload[10];
        tx_payload[0] = status & 0xFF;
        tx_payload[1] = (status >> 8) & 0xFF;
        tx_payload[2] = (status >> 16) & 0xFF;
        tx_payload[3] = (status >> 24) & 0xFF;
        for (int k = 0; k < 6; k++)
            tx_payload[4 + k] = payload[k];

        gmp_datalink_send(ctx->dl_ctx, target_id, ctx->base_cmd + GMP_TUNABLE_OFFSET_MEM_WRITE_ACK, tx_payload, 10);
        return GMP_TUNABLE_HANDLED;
    }
    default:
        return GMP_TUNABLE_PASS;
    }
}

// =========================================================
// MODULE 3: FLEX VARIABLE TUNABLE (Backdoor)
// =========================================================

void gmp_tunable_flex_init(gmp_tunable_flex_t* ctx, gmp_datalink_t* dl_ctx, uint16_t base_cmd,
                           gmp_var_entry_t* ram_dict, uint16_t max_channels)
{
    if (!ctx)
        return;
    ctx->dl_ctx = dl_ctx;
    ctx->base_cmd = base_cmd;
    ctx->backdoor_dict = ram_dict;
    ctx->max_channels = max_channels;

    if (ram_dict)
    {
        memset(ram_dict, 0, sizeof(gmp_var_entry_t) * max_channels);
    }
}

fast_gt gmp_tunable_flex_rx_cb(gmp_tunable_flex_t* ctx, uint16_t target_id, uint16_t cmd, const data_gt* payload,
                               size_gt len)
{
    if (!ctx || cmd < ctx->base_cmd)
        return GMP_TUNABLE_PASS;

    uint16_t offset = cmd - ctx->base_cmd;

    // Strict Offset Filtering: Flex module handles 0, 2, 4
    if (offset != GMP_TUNABLE_OFFSET_INFO_REQ && offset != GMP_TUNABLE_OFFSET_VAR_READ_REQ &&
        offset != GMP_TUNABLE_OFFSET_VAR_WRITE_REQ)
    {
        return GMP_TUNABLE_PASS;
    }

    switch (offset)
    {
    case GMP_TUNABLE_OFFSET_INFO_REQ: {
        if (len < 6)
            return GMP_TUNABLE_HANDLED;
        uint16_t ch = (payload[0] & 0xFF) | ((payload[1] & 0xFF) << 8);

        uint32_t status = GMP_EC_OK;
        if (ch >= ctx->max_channels || !ctx->backdoor_dict)
        {
            status = GMP_EC_TUNABLE_ERR_NO_ID;
        }
        else
        {
            uint32_t addr = (payload[2] & 0xFF) | ((payload[3] & 0xFF) << 8) | ((payload[4] & 0xFF) << 16) |
                            ((payload[5] & 0xFF) << 24);
            uint8_t type = payload[6] & 0xFF;

            ctx->backdoor_dict[ch].var_ptr = (void*)((uintptr_t)addr);
            ctx->backdoor_dict[ch].var_type = (gmp_var_type_t)type;
            ctx->backdoor_dict[ch].access = GMP_TUNABLE_ACCESS_RW;
        }

        data_gt tx_payload[6];
        tx_payload[0] = payload[0];
        tx_payload[1] = payload[1];
        tx_payload[2] = status & 0xFF;
        tx_payload[3] = (status >> 8) & 0xFF;
        tx_payload[4] = (status >> 16) & 0xFF;
        tx_payload[5] = (status >> 24) & 0xFF;

        gmp_datalink_send(ctx->dl_ctx, target_id, ctx->base_cmd + GMP_TUNABLE_OFFSET_INFO_ACK, tx_payload, 6);
        return GMP_TUNABLE_HANDLED;
    }

    case GMP_TUNABLE_OFFSET_VAR_READ_REQ: {
        if (len < 2)
            return GMP_TUNABLE_HANDLED;
        uint16_t ch = (payload[0] & 0xFF) | ((payload[1] & 0xFF) << 8);

        if (ctx->backdoor_dict && ch < ctx->max_channels && ctx->backdoor_dict[ch].var_ptr)
        {
            gmp_var_entry_t* entry = &ctx->backdoor_dict[ch];
            data_gt tx_payload[6];
            tx_payload[0] = ch & 0xFF;
            tx_payload[1] = (ch >> 8) & 0xFF;
            size_gt tx_len = 2;
            switch (entry->var_type)
            {
            case GMP_VAR_TYPE_UINT16:
            case GMP_VAR_TYPE_INT16: {
                uint16_t val = *((uint16_t*)entry->var_ptr);
                tx_payload[tx_len++] = val & 0xFF;
                tx_payload[tx_len++] = (val >> 8) & 0xFF;
                break;
            }
            case GMP_VAR_TYPE_UINT32:
            case GMP_VAR_TYPE_INT32:
            case GMP_VAR_TYPE_FLOAT: {
                uint32_t val = *((uint32_t*)entry->var_ptr);
                tx_payload[tx_len++] = val & 0xFF;
                tx_payload[tx_len++] = (val >> 8) & 0xFF;
                tx_payload[tx_len++] = (val >> 16) & 0xFF;
                tx_payload[tx_len++] = (val >> 24) & 0xFF;
                break;
            }
            }
            gmp_datalink_send(ctx->dl_ctx, target_id, ctx->base_cmd + GMP_TUNABLE_OFFSET_VAR_READ_ACK, tx_payload,
                              tx_len);
        }
        return GMP_TUNABLE_HANDLED;
    }

    case GMP_TUNABLE_OFFSET_VAR_WRITE_REQ: {
        if (len < 2)
            return GMP_TUNABLE_HANDLED;
        uint16_t ch = (payload[0] & 0xFF) | ((payload[1] & 0xFF) << 8);
        uint32_t status = GMP_EC_OK;

        gmp_var_entry_t* entry = NULL;
        if (ctx->backdoor_dict && ch < ctx->max_channels)
            entry = &ctx->backdoor_dict[ch];

        if (!entry || !entry->var_ptr)
            status = GMP_EC_TUNABLE_ERR_NO_ID;
        else
        {
            switch (entry->var_type)
            {
            case GMP_VAR_TYPE_UINT16:
            case GMP_VAR_TYPE_INT16:
                if (len < 4)
                    status = GMP_EC_TUNABLE_ERR_BAD_LEN;
                else
                    *((uint16_t*)entry->var_ptr) = (payload[2] & 0xFF) | ((payload[3] & 0xFF) << 8);
                break;
            case GMP_VAR_TYPE_UINT32:
            case GMP_VAR_TYPE_INT32:
            case GMP_VAR_TYPE_FLOAT:
                if (len < 6)
                    status = GMP_EC_TUNABLE_ERR_BAD_LEN;
                else
                    *((uint32_t*)entry->var_ptr) = (payload[2] & 0xFF) | ((payload[3] & 0xFF) << 8) |
                                                   ((payload[4] & 0xFF) << 16) | ((payload[5] & 0xFF) << 24);
                break;
            }
        }
        data_gt tx_payload[6];
        tx_payload[0] = ch & 0xFF;
        tx_payload[1] = (ch >> 8) & 0xFF;
        tx_payload[2] = status & 0xFF;
        tx_payload[3] = (status >> 8) & 0xFF;
        tx_payload[4] = (status >> 16) & 0xFF;
        tx_payload[5] = (status >> 24) & 0xFF;

        gmp_datalink_send(ctx->dl_ctx, target_id, ctx->base_cmd + GMP_TUNABLE_OFFSET_VAR_WRITE_ACK, tx_payload, 6);
        return GMP_TUNABLE_HANDLED;
    }
    default:
        return GMP_TUNABLE_PASS;
    }
}
