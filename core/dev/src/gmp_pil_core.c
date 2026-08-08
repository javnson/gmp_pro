#include <gmp_core.h>

#include <core/dev/pil_core.h>

#include <string.h>

/* Little-endian payload decoding helpers. */

/**
 * @brief Decode one little-endian 16-bit value from a Data Link payload.
 * @param buf Payload buffer.
 * @param idx Current offset, advanced on success.
 * @param max_len Number of valid protocol octets in @p buf.
 * @return Decoded value, or zero when fewer than two octets remain.
 */
static inline uint16_t sim_unpack_u16(const gmp_dl_octet_t* buf, uint16_t* idx, uint16_t max_len)
{
    if (*idx + 2 > max_len)
        return 0;

    uint16_t val = ((uint16_t)(buf[*idx] & 0xFF)) | (((uint16_t)(buf[*idx + 1] & 0xFF)) << 8);
    *idx += 2;
    return val;
}

/**
 * @brief Decode one little-endian 32-bit value from a Data Link payload.
 * @param buf Payload buffer.
 * @param idx Current offset, advanced on success.
 * @param max_len Number of valid protocol octets in @p buf.
 * @return Decoded value, or zero when fewer than four octets remain.
 */
static inline uint32_t sim_unpack_u32(const gmp_dl_octet_t* buf, uint16_t* idx, uint16_t max_len)
{
    if (*idx + 4 > max_len)
        return 0;

    uint32_t val = ((uint32_t)(buf[*idx] & 0xFF)) | (((uint32_t)(buf[*idx + 1] & 0xFF)) << 8) |
                   (((uint32_t)(buf[*idx + 2] & 0xFF)) << 16) | (((uint32_t)(buf[*idx + 3] & 0xFF)) << 24);
    *idx += 4;
    return val;
}

/* PIL payload serialization helpers. */

/**
 * @brief Deserialize enabled input fields according to the receive mask.
 * @param ctx PIL instance receiving the payload.
 */
static void sim_deserialize_inputs(gmp_pil_sim_t* ctx)
{
    const gmp_dl_octet_t* pld = ctx->dl_ctx->payload_buf;
    uint16_t pld_len = ctx->dl_ctx->expected_payload_len;
    uint16_t idx = 0;
    size_gt i;
    gmp_safe_pun_t pun;

    /* Decode the fixed input fields. */
    ctx->rx_buf.isr_ticks = sim_unpack_u32(pld, &idx, pld_len);
    ctx->rx_buf.digital_input = sim_unpack_u32(pld, &idx, pld_len);

    /* Decode up to 24 enabled ADC channels. */
    for (i = 0; i < 24; i++)
    {
        if (ctx->mask_rx.bit.adc_result & (1UL << i))
        {
            ctx->rx_buf.adc_result[i] = sim_unpack_u16(pld, &idx, pld_len);
        }
    }

    /* Decode up to eight enabled virtual-panel values. */
    for (i = 0; i < 8; i++)
    {
        if (ctx->mask_rx.bit.panel & (1UL << i))
        {
            pun.u_val = sim_unpack_u32(pld, &idx, pld_len);
            ctx->rx_buf.panel[i] = pun.f_val;
        }
    }
}

/**
 * @brief Serialize enabled output fields according to the transmit mask.
 * @param ctx PIL instance providing the output values.
 * @pre gmp_dev_dl_tx_request_cmd() has started a response transaction.
 */
static void sim_serialize_outputs(gmp_pil_sim_t* ctx)
{
    size_gt i;
    gmp_safe_pun_t pun;
    gmp_datalink_t* dl = ctx->dl_ctx;

    /* Encode the fixed output fields. */
    gmp_dev_dl_tx_append_u32(dl, ctx->tx_buf.digital_out);

    /* Encode up to eight enabled PWM compare values. */
    for (i = 0; i < 8; i++)
    {
        if (ctx->mask_tx.bit.pwm_cmp & (1UL << i))
        {
            gmp_dev_dl_tx_append_u16(dl, ctx->tx_buf.pwm_cmp[i]);
        }
    }

    /* Encode up to eight enabled DAC values. */
    for (i = 0; i < 8; i++)
    {
        if (ctx->mask_tx.bit.dac & (1UL << i))
        {
            gmp_dev_dl_tx_append_u16(dl, ctx->tx_buf.dac[i]);
        }
    }

    /* Encode up to 16 enabled monitor values. */
    for (i = 0; i < 16; i++)
    {
        if (ctx->mask_tx.bit.monitor & (1UL << i))
        {
            pun.f_val = ctx->tx_buf.monitor[i];
            gmp_dev_dl_tx_append_u32(dl, pun.u_val);
        }
    }
}

/* Public API. */

void gmp_pil_sim_init(gmp_pil_sim_t* ctx, gmp_datalink_t* dl_ctx, uint16_t base_cmd)
{
    memset(ctx, 0, sizeof(gmp_pil_sim_t));
    ctx->dl_ctx = dl_ctx;
    ctx->base_cmd = base_cmd;

    /* Preserve the historical behavior: all optional channels are enabled. */
    ctx->mask_tx.all = 0xFFFFFFFF;
    ctx->mask_rx.all = 0xFFFFFFFF;
}

fast_gt gmp_pil_sim_rx_cb(gmp_pil_sim_t* ctx)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    uint16_t rcv_cmd = dl->rx_head.cmd;

    /* Leave commands outside this PIL instance to the next handler. */
    if (rcv_cmd < ctx->base_cmd || rcv_cmd > ctx->base_cmd + 4)
    {
        return 0;
    }

    /* Convert the command into a PIL-local operation. */
    uint16_t offset = rcv_cmd - ctx->base_cmd;
    uint16_t idx = 0;

    switch (offset)
    {
    case GMP_PIL_OFFSET_SIM_SET_MASK_REQ:
        /* Decode the transmit mask followed by the receive mask. */
        if (dl->expected_payload_len >= 8)
        {
            ctx->mask_tx.all = sim_unpack_u32(dl->payload_buf, &idx, dl->expected_payload_len);
            ctx->mask_rx.all = sim_unpack_u32(dl->payload_buf, &idx, dl->expected_payload_len);
        }

        /* Return the masks that were actually applied. */
        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, rcv_cmd);
        gmp_dev_dl_tx_append_u32(dl, ctx->mask_tx.all);
        gmp_dev_dl_tx_append_u32(dl, ctx->mask_rx.all);
        gmp_dev_dl_tx_ready(dl);
        break;

    case GMP_PIL_OFFSET_SIM_STEP_REQ:
        /* Decode inputs, execute one user step, and return its outputs. */
        sim_deserialize_inputs(ctx);

        gmp_pil_sim_step(&ctx->rx_buf, &ctx->tx_buf);

        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, rcv_cmd);
        sim_serialize_outputs(ctx);
        gmp_dev_dl_tx_ready(dl);
        break;

    case GMP_PIL_OFFSET_SIM_SET_INPUT_REQ:
        /* Update inputs without executing a control step. */
        sim_deserialize_inputs(ctx);
        gmp_dev_dl_reply_ack_null(dl);
        break;

    case GMP_PIL_OFFSET_SIM_GET_OUTPUT_REQ:
        /* Return the current outputs without changing inputs. */
        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, rcv_cmd);
        sim_serialize_outputs(ctx);
        gmp_dev_dl_tx_ready(dl);
        break;

    default:
        /* Reject reserved offsets in the PIL command window. */
        gmp_dev_dl_reply_nack(dl, 0x0001);
        break;
    }

    /* Prevent the default Data Link handler from replying a second time. */
    gmp_dev_dl_msg_handled(dl);

    return 1;
}
