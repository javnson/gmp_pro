#include <gmp_core.h>

#include <core/dev/tunable.h>


// =========================================================
// 内部辅助函数：安全的 Little-Endian 总线提取器
// =========================================================

static inline fast16_gt param_unpack_u8(const data_gt* buf, uint16_t* idx, uint16_t max_len)
{
    if (*idx >= max_len)
        return 0;
    fast16_gt val = buf[*idx] & 0xFF;
    *idx += 1;
    return val;
}

static inline uint16_t param_unpack_u16(const data_gt* buf, uint16_t* idx, uint16_t max_len)
{
    if (*idx + 2 > max_len)
        return 0;
    uint16_t val = ((uint16_t)(buf[*idx] & 0xFF)) | (((uint16_t)(buf[*idx + 1] & 0xFF)) << 8);
    *idx += 2;
    return val;
}

static inline uint32_t param_unpack_u32(const data_gt* buf, uint16_t* idx, uint16_t max_len)
{
    if (*idx + 4 > max_len)
        return 0;
    uint32_t val = ((uint32_t)(buf[*idx] & 0xFF)) | (((uint32_t)(buf[*idx + 1] & 0xFF)) << 8) |
                   (((uint32_t)(buf[*idx + 2] & 0xFF)) << 16) | (((uint32_t)(buf[*idx + 3] & 0xFF)) << 24);
    *idx += 4;
    return val;
}

// =========================================================
// 核心 API 实现
// =========================================================

void gmp_param_tunable_init(gmp_param_tunable_t* ctx, gmp_datalink_t* dl, uint16_t base_cmd,
                            const gmp_param_item_t* dict, fast16_gt dict_size)
{
    ctx->dl_ctx = dl;
    ctx->base_cmd = base_cmd;
    ctx->dict = dict;
    ctx->dict_size = (dict_size > 255) ? 255 : dict_size;
}

fast_gt gmp_param_tunable_rx_cb(gmp_param_tunable_t* ctx)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    uint16_t cmd = dl->rx_head.cmd;

    if (cmd != ctx->base_cmd && cmd != (ctx->base_cmd + 1))
    {
        return 0;
    }

    uint16_t len = dl->expected_payload_len;
    uint16_t idx = 0;
    const data_gt* pld = dl->payload_buf;
    gmp_safe_pun_t pun;

    // ==========================================
    // 读变量处理 (Base CMD)
    // ==========================================
    if (cmd == ctx->base_cmd)
    {
        fast16_gt req_cnt = param_unpack_u8(pld, &idx, len);
        fast16_gt valid_cnt = 0;

        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, cmd);
        gmp_dev_dl_tx_append_u8(dl, 0);

        for (fast16_gt i = 0; i < req_cnt; i++)
        {
            fast16_gt target_id = param_unpack_u8(pld, &idx, len);

            if (target_id >= ctx->dict_size)
                continue;

            const gmp_param_item_t* item = &ctx->dict[target_id];
            gmp_dev_dl_tx_append_u8(dl, target_id);

            switch (item->type)
            {
            case GMP_PARAM_TYPE_U16:
            case GMP_PARAM_TYPE_I16:
                gmp_dev_dl_tx_append_u16(dl, *((uint16_t*)item->addr));
                break;
            case GMP_PARAM_TYPE_U32:
            case GMP_PARAM_TYPE_I32:
            case GMP_PARAM_TYPE_F32:
                gmp_dev_dl_tx_append_u32(dl, *((uint32_t*)item->addr));
                break;
            }
            valid_cnt++;
        }

        dl->tx_hw_pld_buf[0] = valid_cnt & 0xFF;
        gmp_dev_dl_tx_ready(dl);
        gmp_dev_dl_msg_handled(dl);
        return 1;
    }

    // ==========================================
    // 写变量处理 (Base CMD + 1)
    // ==========================================
    else if (cmd == (ctx->base_cmd + 1))
    {
        fast16_gt req_cnt = param_unpack_u8(pld, &idx, len);
        fast16_gt status = 0;

        for (fast16_gt i = 0; i < req_cnt; i++)
        {
            fast16_gt target_id = param_unpack_u8(pld, &idx, len);

            if (target_id >= ctx->dict_size)
            {
                status = 1;
                break;
            }

            const gmp_param_item_t* item = &ctx->dict[target_id];

            if (item->perm == GMP_PARAM_PERM_RO)
            {
                status = 1;
                idx += (item->type <= GMP_PARAM_TYPE_I16) ? 2 : 4;
                continue;
            }

            switch (item->type)
            {
            case GMP_PARAM_TYPE_U16:
            case GMP_PARAM_TYPE_I16:
                *((uint16_t*)item->addr) = param_unpack_u16(pld, &idx, len);
                break;
            case GMP_PARAM_TYPE_U32:
            case GMP_PARAM_TYPE_I32:
            case GMP_PARAM_TYPE_F32:
                pun.u_val = param_unpack_u32(pld, &idx, len);
                *((uint32_t*)item->addr) = pun.u_val;
                break;
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
