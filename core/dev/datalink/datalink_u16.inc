/**
 * @file gmp_datalink_u16.inc
 * @brief Legacy 16-bit-addressed GMP Data Link implementation.
 */

/* Internal helpers. */

/**
 * @brief Reset the receive state machine after a frame or error.
 * @param ctx Data Link instance to reset.
 */
static inline void datalink_reset_rx(gmp_datalink_t* ctx)
{
    ctx->rx_state = GMP_DL_STATE_WAIT_SYNC;
    ctx->rx_hdr_idx = 0;
    ctx->payload_idx = 0;
}

/* Initialization and ISR ingress API. */

void gmp_dev_dl_init(gmp_datalink_t* ctx)
{
    if (ctx == NULL)
        return;
    memset(ctx, 0, sizeof(gmp_datalink_t));
    ctx->rx_state = GMP_DL_STATE_WAIT_SYNC;
    ctx->tx_state = GMP_DL_TX_STATE_IDLE;
    gmp_list_init(&ctx->facility_list);
}

void gmp_dev_dl_push_byte(gmp_datalink_t* ctx, byte_gt raw_data)
{
    uint16_t next_head = ctx->rx_fifo_head + 1;
    if (next_head >= GMP_DL_RX_FIFO_SIZE)
    {
        next_head = 0;
    }

    if (next_head != ctx->rx_fifo_tail)
    {
        ctx->rx_fifo[ctx->rx_fifo_head] = raw_data & 0xFF;
        ctx->rx_fifo_head = next_head;
    }
    else
    {
        ctx->err_fifo_ovf_cnt++;
        ctx->rx_reset_pending = 1;
    }
}

void gmp_dev_dl_push_str(gmp_datalink_t* ctx, const byte_gt* str, size_gt size)
{
    if (!ctx || !str || size == 0)
        return;

    size_gt i;

    for (i = 0; i < size; i++)
    {
        uint16_t next_head = ctx->rx_fifo_head + 1;
        if (next_head >= GMP_DL_RX_FIFO_SIZE)
        {
            next_head = 0;
        }

        if (next_head != ctx->rx_fifo_tail)
        {
            ctx->rx_fifo[ctx->rx_fifo_head] = str[i] & 0xFF;
            ctx->rx_fifo_head = next_head;
        }
        else
        {
            /* Count all remaining units as dropped when the FIFO is full. */
            ctx->err_fifo_ovf_cnt += (size - i);
            ctx->rx_reset_pending = 1;
            break;
        }
    }
}

void gmp_dev_dl_request_rx_reset(gmp_datalink_t* ctx)
{
    if (ctx != NULL)
    {
        ctx->rx_reset_pending = 1;
    }
}

/* Receive and transmit state-machine driver. */

gmp_dl_event_t gmp_dev_dl_loop_cb(gmp_datalink_t* ctx)
{
    if (ctx == NULL)
        return GMP_DL_EVENT_IDLE;

    ctx->service_run_count++;
    if (ctx->rx_reset_pending)
    {
        ctx->rx_reset_pending = 0;
        ctx->rx_fifo_tail = ctx->rx_fifo_head;
        datalink_reset_rx(ctx);
    }

    /* Finish deferred framing before accepting another request. */
    if (ctx->tx_state == GMP_DL_TX_STATE_READY_TO_WARP)
    {
        gmp_dev_dl_tx_warp(ctx);
        ctx->tx_state = GMP_DL_TX_STATE_PENDING_HW;
        return GMP_DL_EVENT_TX_RDY;
    }

    /* Do not parse another request while hardware owns the TX buffers. */
    if (ctx->tx_state == GMP_DL_TX_STATE_PENDING_HW || ctx->tx_state == GMP_DL_TX_STATE_PENDING_HW_HDR ||
        ctx->tx_state == GMP_DL_TX_STATE_PENDING_HW_PLD)
    {
        return GMP_DL_EVENT_IDLE;
    }

    /* Parse queued receive units. */
    while (ctx->rx_fifo_tail != ctx->rx_fifo_head)
    {
        byte_gt byte = ctx->rx_fifo[ctx->rx_fifo_tail];

        /* Pop one addressable unit from the receive FIFO. */
        ctx->rx_fifo_tail++;
        if (ctx->rx_fifo_tail >= GMP_DL_RX_FIFO_SIZE)
        {
            ctx->rx_fifo_tail = 0;
        }

        ctx->last_rx_tick = gmp_base_get_system_tick();

        /* A new start delimiter resynchronizes a partial header. */
        if ((ctx->rx_state == GMP_DL_STATE_HEADER_RECV || ctx->rx_state == GMP_DL_STATE_HEADER_ESCAPE) &&
            byte == GMP_DL_SOF)
        {
            datalink_reset_rx(ctx);
            ctx->rx_state = GMP_DL_STATE_HEADER_RECV;
            continue;
        }

        /* Advance the receive state machine. */
        switch (ctx->rx_state)
        {
        case GMP_DL_STATE_WAIT_SYNC:
            if (byte == GMP_DL_SOF)
            {
                ctx->rx_state = GMP_DL_STATE_HEADER_RECV;
                ctx->rx_hdr_idx = 0;
                ctx->payload_idx = 0;
            }
            break;

        case GMP_DL_STATE_HEADER_RECV:
            if (byte == GMP_DL_ESC)
            {
                ctx->rx_state = GMP_DL_STATE_HEADER_ESCAPE;
            }
            else if (byte == GMP_DL_EOF)
            {
                if (ctx->rx_hdr_idx != GMP_DL_HDR_SIZE)
                {
                    datalink_reset_rx(ctx);
                    break;
                }

                /* Validate the header CRC. */
                uint16_t h_crc_rcv = (ctx->rx_hdr_buf[4] & 0xFF) | ((ctx->rx_hdr_buf[5] & 0xFF) << 8);
                uint16_t h_crc_calc = gmp_base_calculate_crc16(ctx->rx_hdr_buf, 4);

                if (h_crc_calc != h_crc_rcv)
                {
                    ctx->err_hdr_crc_cnt++;
                    datalink_reset_rx(ctx);
                    break;
                }

                /* Decode the validated header. */
                ctx->rx_head.seq_id = ctx->rx_hdr_buf[0] & 0xFF;
                ctx->rx_head.cmd = ctx->rx_hdr_buf[1] & 0xFF;
                ctx->expected_payload_len = (ctx->rx_hdr_buf[2] & 0xFF) | ((ctx->rx_hdr_buf[3] & 0xFF) << 8);

                if (ctx->expected_payload_len > GMP_DL_MTU)
                {
                    datalink_reset_rx(ctx);
                    break;
                }

                /* Deliver an empty-payload frame immediately. */
                if (ctx->expected_payload_len == 0)
                {
                    ctx->flag_reply_handled = 0;
                    datalink_reset_rx(ctx);
                    return GMP_DL_EVENT_RX_OK;
                }
                else
                {
                    ctx->rx_state = GMP_DL_STATE_PAYLOAD_RECV;
                }
            }
            else if (ctx->rx_hdr_idx < GMP_DL_HDR_SIZE)
            {
                ctx->rx_hdr_buf[ctx->rx_hdr_idx++] = byte;
            }
            else
            {
                datalink_reset_rx(ctx);
            }
            break;

        case GMP_DL_STATE_HEADER_ESCAPE:
            if (ctx->rx_hdr_idx < GMP_DL_HDR_SIZE)
            {
                ctx->rx_hdr_buf[ctx->rx_hdr_idx++] = byte ^ GMP_DL_XOR;
                ctx->rx_state = GMP_DL_STATE_HEADER_RECV;
            }
            else
            {
                datalink_reset_rx(ctx);
            }
            break;

        case GMP_DL_STATE_PAYLOAD_RECV:
            ctx->payload_buf[ctx->payload_idx++] = byte;

            if (ctx->payload_idx == (ctx->expected_payload_len + 2))
            {
                uint16_t p_crc_rcv = (ctx->payload_buf[ctx->expected_payload_len] & 0xFF) |
                                     ((ctx->payload_buf[ctx->expected_payload_len + 1] & 0xFF) << 8);
                uint16_t p_crc_calc = gmp_base_calculate_crc16(ctx->payload_buf, ctx->expected_payload_len);

                if (p_crc_calc == p_crc_rcv)
                {
                    ctx->flag_reply_handled = 0;
                    datalink_reset_rx(ctx);
                    return GMP_DL_EVENT_RX_OK;
                }
                else
                {
                    ctx->err_pld_crc_cnt++;
                    datalink_reset_rx(ctx);
                }
            }
            break;

        default:
            datalink_reset_rx(ctx);
            break;
        }
    }

    if (ctx->rx_state != GMP_DL_STATE_WAIT_SYNC)
    {
        if (gmp_base_is_delay_elapsed(ctx->last_rx_tick, GMP_DL_OVERTIME))
        {
            datalink_reset_rx(ctx);
            ctx->err_timeout_cnt++;
        }
    }

    return GMP_DL_EVENT_IDLE;
}

/* Default application-level response policy. */

/**
 * @brief Apply the default response policy to one received frame.
 * @details Handles ECHO, silently consumes NACK and stray input, and returns
 * a standard unsupported-command NACK for unclaimed application commands.
 * @param ctx Data Link instance containing the received frame.
 */
void gmp_dev_dl_default_rx_handler(gmp_datalink_t* ctx)
{
    /* An application handler has already completed this transaction. */
    if (ctx->flag_reply_handled)
        return;

    switch (ctx->rx_head.cmd)
    {
    case GMP_DL_CMD_ECHO:
        /* Echo payloads are returned unchanged. */
        if (ctx->expected_payload_len == 0)
        {
            gmp_dev_dl_reply_ack_null(ctx);
        }
        else
        {
            gmp_dev_dl_tx_request(ctx, ctx->rx_head.seq_id, GMP_DL_CMD_ECHO, ctx->expected_payload_len,
                                  ctx->payload_buf);
            ctx->flag_reply_handled = 1;
        }
        break;

    case GMP_DL_CMD_NACK:
        /* Silently consume NACK to prevent a NACK feedback loop. */
        ctx->flag_reply_handled = 1;
        break;

    case GMP_DL_CMD_STRAY:
        /* Silently consume stray input unless an application intercepted it. */
        ctx->flag_reply_handled = 1;
        break;

    default:
        /* Reject commands not claimed by an application handler. */
        gmp_dev_dl_reply_nack(ctx, 0x0001);
        break;
    }
}

/* Transmit builder API. */

void gmp_dev_dl_tx_request_cmd(gmp_datalink_t* ctx, uint16_t seq, uint16_t cmd)
{
    if (ctx->tx_state != GMP_DL_TX_STATE_IDLE)
        return;

    ctx->tx_head.seq_id = seq;
    ctx->tx_head.cmd = cmd;
    ctx->tx_len = 0;
    ctx->tx_state = GMP_DL_TX_STATE_BUILDING;
}

void gmp_dev_dl_tx_append_payload(gmp_datalink_t* ctx, const byte_gt* data, size_gt actual_payload_len)
{
    if (ctx->tx_state != GMP_DL_TX_STATE_BUILDING)
        return;
    if (actual_payload_len == 0 || data == NULL)
        return;

    if ((ctx->tx_len + actual_payload_len) > GMP_DL_MTU)
        return;

    memcpy(&ctx->tx_buf[ctx->tx_len], data, actual_payload_len * sizeof(byte_gt));
    ctx->tx_len += actual_payload_len;
}

void gmp_dev_dl_tx_append_u8(gmp_datalink_t* ctx, byte_gt val)
{
    if (ctx->tx_state != GMP_DL_TX_STATE_BUILDING)
        return;
    if (ctx->tx_len + 1 > GMP_DL_MTU)
        return;

    ctx->tx_buf[ctx->tx_len++] = val & 0xFF;
}

void gmp_dev_dl_tx_append_u16(gmp_datalink_t* ctx, uint16_t val)
{
    if (ctx->tx_state != GMP_DL_TX_STATE_BUILDING)
        return;
    if (ctx->tx_len + 2 > GMP_DL_MTU)
        return;

    ctx->tx_buf[ctx->tx_len++] = val & 0xFF;        // LSB
    ctx->tx_buf[ctx->tx_len++] = (val >> 8) & 0xFF; // MSB
}

void gmp_dev_dl_tx_append_u32(gmp_datalink_t* ctx, uint32_t val)
{
    if (ctx->tx_state != GMP_DL_TX_STATE_BUILDING)
        return;
    if (ctx->tx_len + 4 > GMP_DL_MTU)
        return;

    ctx->tx_buf[ctx->tx_len++] = val & 0xFF; // LSB
    ctx->tx_buf[ctx->tx_len++] = (val >> 8) & 0xFF;
    ctx->tx_buf[ctx->tx_len++] = (val >> 16) & 0xFF;
    ctx->tx_buf[ctx->tx_len++] = (val >> 24) & 0xFF; // MSB
}

void gmp_dev_dl_tx_ready(gmp_datalink_t* ctx)
{
    if (ctx->tx_state == GMP_DL_TX_STATE_BUILDING)
    {
        ctx->tx_state = GMP_DL_TX_STATE_READY_TO_WARP;
    }
}

void gmp_dev_dl_tx_request(gmp_datalink_t* ctx, uint16_t seq, uint16_t cmd, size_gt actual_payload_len,
                           const byte_gt* data)
{
    gmp_dev_dl_tx_request_cmd(ctx, seq, cmd);
    gmp_dev_dl_tx_append_payload(ctx, data, actual_payload_len);
    gmp_dev_dl_tx_ready(ctx);
}

/* Transmit buffer accessors. */

size_gt gmp_dev_dl_get_tx_capacity(gmp_datalink_t* ctx)
{
    if (ctx->tx_state != GMP_DL_TX_STATE_BUILDING)
        return 0;
    return GMP_DL_MTU - ctx->tx_len;
}

byte_gt* gmp_dev_dl_get_tx_payload_ptr(gmp_datalink_t* ctx)
{
    if (ctx->tx_state != GMP_DL_TX_STATE_BUILDING)
        return NULL;
    return &ctx->tx_buf[ctx->tx_len];
}

const byte_gt* gmp_dev_dl_get_tx_hw_hdr(gmp_datalink_t* ctx, size_gt* out_len)
{
    if (out_len)
        *out_len = ctx->tx_hdr_len;
    return ctx->tx_hdr_buf;
}

const byte_gt* gmp_dev_dl_get_tx_hw_pld(gmp_datalink_t* ctx, size_gt* out_len)
{
    if (out_len)
        *out_len = ctx->tx_len;
    return ctx->tx_buf;
}

/* Frame construction and header escaping. */

void gmp_dev_dl_tx_warp(gmp_datalink_t* ctx)
{
    size_gt i;

    /* Preserve the application payload length before appending its CRC. */
    uint16_t pld_len = ctx->tx_len;

    /* Append the payload CRC only for a non-empty payload. */
    if (pld_len > 0)
    {
        uint16_t p_crc = gmp_base_calculate_crc16(ctx->tx_buf, pld_len);
        ctx->tx_buf[ctx->tx_len++] = p_crc & 0xFF;
        ctx->tx_buf[ctx->tx_len++] = (p_crc >> 8) & 0xFF;
    }

    /* Build the unescaped six-unit header. */
    byte_gt raw_hdr[6];
    raw_hdr[0] = ctx->tx_head.seq_id & 0xFF;
    raw_hdr[1] = ctx->tx_head.cmd & 0xFF;
    raw_hdr[2] = pld_len & 0xFF;
    raw_hdr[3] = (pld_len >> 8) & 0xFF;

    uint16_t h_crc = gmp_base_calculate_crc16(raw_hdr, 4);
    raw_hdr[4] = h_crc & 0xFF;
    raw_hdr[5] = (h_crc >> 8) & 0xFF;

    /* Escape the header into the hardware transmit buffer. */
    size_gt h_idx = 0;
    ctx->tx_hdr_buf[h_idx++] = GMP_DL_SOF;

    for (i = 0; i < 6; i++)
    {
        byte_gt b = raw_hdr[i];
        if (b == GMP_DL_SOF || b == GMP_DL_EOF || b == GMP_DL_ESC)
        {
            ctx->tx_hdr_buf[h_idx++] = GMP_DL_ESC;
            ctx->tx_hdr_buf[h_idx++] = b ^ GMP_DL_XOR;
        }
        else
        {
            ctx->tx_hdr_buf[h_idx++] = b;
        }
    }

    ctx->tx_hdr_buf[h_idx++] = GMP_DL_EOF;
    ctx->tx_hdr_len = h_idx;
}

/* Slave response API. */

void gmp_dev_dl_reply_ack(gmp_datalink_t* ctx)
{
    gmp_dev_dl_tx_request_cmd(ctx, ctx->rx_head.seq_id, ctx->rx_head.cmd);
    ctx->flag_reply_handled = 1;
}

void gmp_dev_dl_reply_ack_null(gmp_datalink_t* ctx)
{
    gmp_dev_dl_tx_request_cmd(ctx, ctx->rx_head.seq_id, ctx->rx_head.cmd);
    gmp_dev_dl_tx_ready(ctx);
    ctx->flag_reply_handled = 1;
}

void gmp_dev_dl_reply_nack(gmp_datalink_t* ctx, uint16_t error_code)
{
    byte_gt nack_payload[2];
    nack_payload[0] = ctx->rx_head.cmd & 0xFF;
    nack_payload[1] = error_code & 0xFF;

    gmp_dev_dl_tx_request(ctx, ctx->rx_head.seq_id, GMP_DL_CMD_NACK, 2, nack_payload);
    ctx->flag_reply_handled = 1;
}

/* Hardware transmit-state transitions. */

void gmp_dev_dl_tx_state_done(gmp_datalink_t* ctx)
{
    ctx->tx_state = GMP_DL_TX_STATE_IDLE;
}

gmp_dl_tx_state_t gmp_dev_dl_tx_state_next(gmp_datalink_t* ctx)
{
    switch (ctx->tx_state)
    {
    case GMP_DL_TX_STATE_PENDING_HW:
        ctx->tx_state = GMP_DL_TX_STATE_PENDING_HW_HDR;
        break;

    case GMP_DL_TX_STATE_PENDING_HW_HDR:
        if (ctx->tx_len > 0)
        {
            ctx->tx_state = GMP_DL_TX_STATE_PENDING_HW_PLD;
        }
        else
        {
            ctx->tx_state = GMP_DL_TX_STATE_IDLE;
        }
        break;

    case GMP_DL_TX_STATE_PENDING_HW_PLD:
    default:
        ctx->tx_state = GMP_DL_TX_STATE_IDLE;
        break;
    }

    return ctx->tx_state;
}

void gmp_dev_dl_msg_handled(gmp_datalink_t* ctx)
{
    ctx->flag_reply_handled = 1;
}
