/**
 * @file gmp_tunable.c
 * @brief Tunable Parameter Group Implementation
 * @details Implements the data dictionary parsing, little-endian unpacking,
 * and dynamic read/write routing for the tunable parameter service.
 */

#include <gmp_core.h>

#include <core/dev/tunable.h>

// =========================================================
// Internal Helper Functions: Safe Little-Endian Unpackers
// =========================================================

/**
 * @brief Unpack an 8-bit unsigned integer from the datalink payload buffer.
 * @param buf Pointer to the datalink payload buffer.
 * @param idx Pointer to the current parsing index (automatically incremented).
 * @param max_len Maximum length of the payload buffer to prevent out-of-bounds access.
 * @return fast16_gt The unpacked 8-bit value (returns 0 if out of bounds).
 */
static inline fast16_gt param_unpack_u8(const data_gt* buf, uint16_t* idx, uint16_t max_len)
{
    if (*idx >= max_len)
        return 0;
    fast16_gt val = buf[*idx] & 0xFF;
    *idx += 1;
    return val;
}

/**
 * @brief Unpack a 16-bit unsigned integer (Little-Endian) from the datalink payload buffer.
 * @param buf Pointer to the datalink payload buffer.
 * @param idx Pointer to the current parsing index (automatically incremented).
 * @param max_len Maximum length of the payload buffer to prevent out-of-bounds access.
 * @return uint16_t The unpacked 16-bit value (returns 0 if out of bounds).
 */
static inline uint16_t param_unpack_u16(const data_gt* buf, uint16_t* idx, uint16_t max_len)
{
    if (*idx + 2 > max_len)
        return 0;
    uint16_t val = ((uint16_t)(buf[*idx] & 0xFF)) | (((uint16_t)(buf[*idx + 1] & 0xFF)) << 8);
    *idx += 2;
    return val;
}

/**
 * @brief Unpack a 32-bit unsigned integer (Little-Endian) from the datalink payload buffer.
 * @param buf Pointer to the datalink payload buffer.
 * @param idx Pointer to the current parsing index (automatically incremented).
 * @param max_len Maximum length of the payload buffer to prevent out-of-bounds access.
 * @return uint32_t The unpacked 32-bit value (returns 0 if out of bounds).
 */
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
// Core API Implementation
// =========================================================

void gmp_param_tunable_init(gmp_param_tunable_t* ctx, gmp_datalink_t* dl, uint16_t base_cmd,
                            const gmp_param_item_t* dict, fast16_gt dict_size)
{
    ctx->dl_ctx = dl;
    ctx->base_cmd = base_cmd;
    ctx->dict = dict;
    // Cap dictionary size at 255 to fit within 8-bit ID mapping constraint
    ctx->dict_size = (dict_size > 255) ? 255 : dict_size;
}

fast_gt gmp_param_tunable_rx_cb(gmp_param_tunable_t* ctx)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    uint16_t cmd = dl->rx_head.cmd;

    // Verify if the incoming command targets this specific tunable service instance
    if (cmd != ctx->base_cmd && cmd != (ctx->base_cmd + 1))
    {
        return 0;
    }

    uint16_t len = dl->expected_payload_len;
    uint16_t idx = 0;
    const data_gt* pld = dl->payload_buf;

    size_gt i;

    // ==========================================
    // Handle Parameter Read Request (Base CMD)
    // ==========================================
    if (cmd == ctx->base_cmd)
    {
        fast16_gt req_cnt = param_unpack_u8(pld, &idx, len);
        fast16_gt valid_cnt = 0;

        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, cmd);
        gmp_dev_dl_tx_append_u8(dl, 0); // Reserve byte for valid count

        for (i = 0; i < req_cnt; i++)
        {
            fast16_gt target_id = param_unpack_u8(pld, &idx, len);

            // Skip out-of-bounds dictionary IDs
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

        // Update the reserved count byte with the actual number of parameters packed
        ctx->dl_ctx->tx_buf[0] = valid_cnt & 0xFF;
        gmp_dev_dl_tx_ready(dl);
        gmp_dev_dl_msg_handled(dl);
        return 1;
    }

    // ==========================================
    // Handle Parameter Write Request (Base CMD + 1)
    // ==========================================
    else if (cmd == (ctx->base_cmd + 1))
    {
        fast16_gt req_cnt = param_unpack_u8(pld, &idx, len);
        fast16_gt status = 0;

        for (i = 0; i < req_cnt; i++)
        {
            fast16_gt target_id = param_unpack_u8(pld, &idx, len);

            if (target_id >= ctx->dict_size)
            {
                status = 1; // Mark status as failed due to invalid ID
                break;
            }

            const gmp_param_item_t* item = &ctx->dict[target_id];

            if (item->perm == GMP_PARAM_PERM_RO)
            {
                status = 1; // Mark status as failed due to permission denial
                // Fast-forward parsing index to continue reading the next parameter
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
                // Corrected: Direct pointer casting assignment bypassing the undeclared 'pun' union
                *((uint32_t*)item->addr) = param_unpack_u32(pld, &idx, len);
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
