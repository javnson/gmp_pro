/**
 * @file gmp_datalink_u8.inc
 * @brief Byte-addressed GMP Data Link implementation.
 */

static const uint16_t gmp_dl_u8_crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2aB3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

static uint16_t dl_u8_crc16(const uint8_t* data, size_gt len)
{
    uint16_t crc = 0xFFFF;
    size_gt i;
    for (i = 0; i < len; ++i)
        crc = (uint16_t)((crc << 8) ^ gmp_dl_u8_crc16_table[((crc >> 8) ^ data[i]) & 0xFF]);
    return crc;
}

static void dl_u8_reset_rx(gmp_datalink_t* ctx)
{
    ctx->rx_state = GMP_DL_STATE_WAIT_SYNC;
    ctx->rx_hdr_idx = 0;
    ctx->payload_idx = 0;
}

void gmp_dev_dl_init(gmp_datalink_t* ctx)
{
    if (ctx == NULL)
        return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->rx_state = GMP_DL_STATE_WAIT_SYNC;
    ctx->tx_state = GMP_DL_TX_STATE_IDLE;
}

void gmp_dev_dl_push_byte(gmp_datalink_t* ctx, uint8_t raw_data)
{
    uint16_t next_head;
    if (ctx == NULL)
        return;
    next_head = (uint16_t)(ctx->rx_fifo_head + 1U);
    if (next_head >= GMP_DL_RX_FIFO_SIZE)
        next_head = 0;
    if (next_head != ctx->rx_fifo_tail)
    {
        ctx->rx_fifo[ctx->rx_fifo_head] = raw_data;
        ctx->rx_fifo_head = next_head;
    }
    else
    {
        ctx->err_fifo_ovf_cnt++;
    }
}

void gmp_dev_dl_push_str(gmp_datalink_t* ctx, const uint8_t* str, size_gt size)
{
    size_gt i;
    if (ctx == NULL || str == NULL)
        return;
    for (i = 0; i < size; ++i)
    {
        uint16_t next_head = (uint16_t)(ctx->rx_fifo_head + 1U);
        if (next_head >= GMP_DL_RX_FIFO_SIZE)
            next_head = 0;
        if (next_head == ctx->rx_fifo_tail)
        {
            ctx->err_fifo_ovf_cnt += (uint32_t)(size - i);
            break;
        }
        ctx->rx_fifo[ctx->rx_fifo_head] = str[i];
        ctx->rx_fifo_head = next_head;
    }
}

gmp_dl_event_t gmp_dev_dl_loop_cb(gmp_datalink_t* ctx)
{
    if (ctx == NULL)
        return GMP_DL_EVENT_IDLE;

    if (ctx->tx_state == GMP_DL_TX_STATE_READY_TO_WARP)
    {
        gmp_dev_dl_tx_warp(ctx);
        ctx->tx_state = GMP_DL_TX_STATE_PENDING_HW;
        return GMP_DL_EVENT_TX_RDY;
    }
    if (ctx->tx_state == GMP_DL_TX_STATE_PENDING_HW ||
        ctx->tx_state == GMP_DL_TX_STATE_PENDING_HW_HDR ||
        ctx->tx_state == GMP_DL_TX_STATE_PENDING_HW_PLD)
        return GMP_DL_EVENT_IDLE;

    while (ctx->rx_fifo_tail != ctx->rx_fifo_head)
    {
        uint8_t byte = ctx->rx_fifo[ctx->rx_fifo_tail];
        ctx->rx_fifo_tail = (uint16_t)(ctx->rx_fifo_tail + 1U);
        if (ctx->rx_fifo_tail >= GMP_DL_RX_FIFO_SIZE)
            ctx->rx_fifo_tail = 0;
        ctx->last_rx_tick = gmp_base_get_system_tick();

        if ((ctx->rx_state == GMP_DL_STATE_HEADER_RECV ||
             ctx->rx_state == GMP_DL_STATE_HEADER_ESCAPE) && byte == GMP_DL_SOF)
        {
            dl_u8_reset_rx(ctx);
            ctx->rx_state = GMP_DL_STATE_HEADER_RECV;
            continue;
        }

        switch (ctx->rx_state)
        {
        case GMP_DL_STATE_WAIT_SYNC:
            if (byte == GMP_DL_SOF)
            {
                ctx->rx_state = GMP_DL_STATE_HEADER_RECV;
            }
            else
            {
                ctx->rx_head.seq_id = 0;
                ctx->rx_head.cmd = GMP_DL_CMD_STRAY;
                ctx->payload_buf[0] = byte;
                ctx->expected_payload_len = 1;
                ctx->flag_reply_handled = 0;
                return GMP_DL_EVENT_RX_OK;
            }
            break;

        case GMP_DL_STATE_HEADER_RECV:
            if (byte == GMP_DL_ESC)
            {
                ctx->rx_state = GMP_DL_STATE_HEADER_ESCAPE;
            }
            else if (byte == GMP_DL_EOF)
            {
                uint16_t received_crc;
                if (ctx->rx_hdr_idx != GMP_DL_HDR_SIZE)
                {
                    dl_u8_reset_rx(ctx);
                    break;
                }
                received_crc = (uint16_t)ctx->rx_hdr_buf[4] |
                               (uint16_t)((uint16_t)ctx->rx_hdr_buf[5] << 8);
                if (dl_u8_crc16(ctx->rx_hdr_buf, 4) != received_crc)
                {
                    ctx->err_hdr_crc_cnt++;
                    dl_u8_reset_rx(ctx);
                    break;
                }
                ctx->rx_head.seq_id = ctx->rx_hdr_buf[0];
                ctx->rx_head.cmd = ctx->rx_hdr_buf[1];
                ctx->expected_payload_len = (uint16_t)ctx->rx_hdr_buf[2] |
                                            (uint16_t)((uint16_t)ctx->rx_hdr_buf[3] << 8);
                if (ctx->expected_payload_len > GMP_DL_MTU)
                {
                    dl_u8_reset_rx(ctx);
                    break;
                }
                if (ctx->expected_payload_len == 0)
                {
                    ctx->flag_reply_handled = 0;
                    dl_u8_reset_rx(ctx);
                    return GMP_DL_EVENT_RX_OK;
                }
                ctx->rx_state = GMP_DL_STATE_PAYLOAD_RECV;
            }
            else if (ctx->rx_hdr_idx < sizeof(ctx->rx_hdr_buf))
            {
                ctx->rx_hdr_buf[ctx->rx_hdr_idx++] = byte;
            }
            break;

        case GMP_DL_STATE_HEADER_ESCAPE:
            if (ctx->rx_hdr_idx < sizeof(ctx->rx_hdr_buf))
                ctx->rx_hdr_buf[ctx->rx_hdr_idx++] = (uint8_t)(byte ^ GMP_DL_XOR);
            ctx->rx_state = GMP_DL_STATE_HEADER_RECV;
            break;

        case GMP_DL_STATE_PAYLOAD_RECV:
            ctx->payload_buf[ctx->payload_idx++] = byte;
            if (ctx->payload_idx == (uint16_t)(ctx->expected_payload_len + 2U))
            {
                uint16_t received_crc = (uint16_t)ctx->payload_buf[ctx->expected_payload_len] |
                    (uint16_t)((uint16_t)ctx->payload_buf[ctx->expected_payload_len + 1U] << 8);
                if (dl_u8_crc16(ctx->payload_buf, ctx->expected_payload_len) == received_crc)
                {
                    ctx->flag_reply_handled = 0;
                    dl_u8_reset_rx(ctx);
                    return GMP_DL_EVENT_RX_OK;
                }
                ctx->err_pld_crc_cnt++;
                dl_u8_reset_rx(ctx);
            }
            break;

        default:
            dl_u8_reset_rx(ctx);
            break;
        }
    }

    if (ctx->rx_state != GMP_DL_STATE_WAIT_SYNC &&
        gmp_base_is_delay_elapsed(ctx->last_rx_tick, GMP_DL_OVERTIME))
    {
        dl_u8_reset_rx(ctx);
        ctx->err_timeout_cnt++;
    }
    return GMP_DL_EVENT_IDLE;
}

void gmp_dev_dl_default_rx_handler(gmp_datalink_t* ctx)
{
    if (ctx == NULL || ctx->flag_reply_handled)
        return;
    switch (ctx->rx_head.cmd)
    {
    case GMP_DL_CMD_ECHO:
        if (ctx->expected_payload_len == 0)
            gmp_dev_dl_reply_ack_null(ctx);
        else
        {
            gmp_dev_dl_tx_request(ctx, ctx->rx_head.seq_id, GMP_DL_CMD_ECHO,
                                  ctx->expected_payload_len, ctx->payload_buf);
            ctx->flag_reply_handled = 1;
        }
        break;
    case GMP_DL_CMD_NACK:
    case GMP_DL_CMD_STRAY:
        ctx->flag_reply_handled = 1;
        break;
    default:
        gmp_dev_dl_reply_nack(ctx, 0x0001);
        break;
    }
}

void gmp_dev_dl_tx_request_cmd(gmp_datalink_t* ctx, uint16_t seq, uint16_t cmd)
{
    if (ctx == NULL || ctx->tx_state != GMP_DL_TX_STATE_IDLE)
        return;
    ctx->tx_head.seq_id = (uint16_t)(seq & 0xFFU);
    ctx->tx_head.cmd = (uint16_t)(cmd & 0xFFU);
    ctx->tx_len = 0;
    ctx->tx_state = GMP_DL_TX_STATE_BUILDING;
}

void gmp_dev_dl_tx_append_payload(gmp_datalink_t* ctx, const uint8_t* data,
                                  size_gt actual_payload_len)
{
    if (ctx == NULL || ctx->tx_state != GMP_DL_TX_STATE_BUILDING || data == NULL ||
        actual_payload_len == 0 || ctx->tx_len + actual_payload_len > GMP_DL_MTU)
        return;
    memcpy(&ctx->tx_buf[ctx->tx_len], data, actual_payload_len);
    ctx->tx_len += actual_payload_len;
}

void gmp_dev_dl_tx_append_u8(gmp_datalink_t* ctx, uint8_t val)
{
    if (ctx != NULL && ctx->tx_state == GMP_DL_TX_STATE_BUILDING && ctx->tx_len < GMP_DL_MTU)
        ctx->tx_buf[ctx->tx_len++] = val;
}

void gmp_dev_dl_tx_append_u16(gmp_datalink_t* ctx, uint16_t val)
{
    if (ctx == NULL || ctx->tx_state != GMP_DL_TX_STATE_BUILDING || ctx->tx_len + 2U > GMP_DL_MTU)
        return;
    ctx->tx_buf[ctx->tx_len++] = (uint8_t)val;
    ctx->tx_buf[ctx->tx_len++] = (uint8_t)(val >> 8);
}

void gmp_dev_dl_tx_append_u32(gmp_datalink_t* ctx, uint32_t val)
{
    if (ctx == NULL || ctx->tx_state != GMP_DL_TX_STATE_BUILDING || ctx->tx_len + 4U > GMP_DL_MTU)
        return;
    ctx->tx_buf[ctx->tx_len++] = (uint8_t)val;
    ctx->tx_buf[ctx->tx_len++] = (uint8_t)(val >> 8);
    ctx->tx_buf[ctx->tx_len++] = (uint8_t)(val >> 16);
    ctx->tx_buf[ctx->tx_len++] = (uint8_t)(val >> 24);
}

void gmp_dev_dl_tx_ready(gmp_datalink_t* ctx)
{
    if (ctx != NULL && ctx->tx_state == GMP_DL_TX_STATE_BUILDING)
        ctx->tx_state = GMP_DL_TX_STATE_READY_TO_WARP;
}

void gmp_dev_dl_tx_request(gmp_datalink_t* ctx, uint16_t seq, uint16_t cmd,
                           size_gt actual_payload_len, const uint8_t* data)
{
    gmp_dev_dl_tx_request_cmd(ctx, seq, cmd);
    gmp_dev_dl_tx_append_payload(ctx, data, actual_payload_len);
    gmp_dev_dl_tx_ready(ctx);
}

size_gt gmp_dev_dl_get_tx_capacity(gmp_datalink_t* ctx)
{
    return (ctx != NULL && ctx->tx_state == GMP_DL_TX_STATE_BUILDING) ?
        (GMP_DL_MTU - ctx->tx_len) : 0;
}

uint8_t* gmp_dev_dl_get_tx_payload_ptr(gmp_datalink_t* ctx)
{
    return (ctx != NULL && ctx->tx_state == GMP_DL_TX_STATE_BUILDING) ?
        &ctx->tx_buf[ctx->tx_len] : NULL;
}

const uint8_t* gmp_dev_dl_get_tx_hw_hdr(gmp_datalink_t* ctx, size_gt* out_len)
{
    if (out_len != NULL)
        *out_len = (ctx != NULL) ? ctx->tx_hdr_len : 0;
    return (ctx != NULL) ? ctx->tx_hdr_buf : NULL;
}

const uint8_t* gmp_dev_dl_get_tx_hw_pld(gmp_datalink_t* ctx, size_gt* out_len)
{
    if (out_len != NULL)
        *out_len = (ctx != NULL) ? ctx->tx_len : 0;
    return (ctx != NULL) ? ctx->tx_buf : NULL;
}

void gmp_dev_dl_tx_warp(gmp_datalink_t* ctx)
{
    uint8_t raw_hdr[GMP_DL_HDR_SIZE];
    uint16_t payload_len;
    uint16_t crc;
    size_gt i;
    size_gt header_index = 0;
    if (ctx == NULL)
        return;

    payload_len = (uint16_t)ctx->tx_len;
    if (payload_len > 0)
    {
        crc = dl_u8_crc16(ctx->tx_buf, payload_len);
        ctx->tx_buf[ctx->tx_len++] = (uint8_t)crc;
        ctx->tx_buf[ctx->tx_len++] = (uint8_t)(crc >> 8);
    }

    raw_hdr[0] = (uint8_t)ctx->tx_head.seq_id;
    raw_hdr[1] = (uint8_t)ctx->tx_head.cmd;
    raw_hdr[2] = (uint8_t)payload_len;
    raw_hdr[3] = (uint8_t)(payload_len >> 8);
    crc = dl_u8_crc16(raw_hdr, 4);
    raw_hdr[4] = (uint8_t)crc;
    raw_hdr[5] = (uint8_t)(crc >> 8);

    ctx->tx_hdr_buf[header_index++] = GMP_DL_SOF;
    for (i = 0; i < GMP_DL_HDR_SIZE; ++i)
    {
        if (raw_hdr[i] == GMP_DL_SOF || raw_hdr[i] == GMP_DL_EOF || raw_hdr[i] == GMP_DL_ESC)
        {
            ctx->tx_hdr_buf[header_index++] = GMP_DL_ESC;
            ctx->tx_hdr_buf[header_index++] = (uint8_t)(raw_hdr[i] ^ GMP_DL_XOR);
        }
        else
        {
            ctx->tx_hdr_buf[header_index++] = raw_hdr[i];
        }
    }
    ctx->tx_hdr_buf[header_index++] = GMP_DL_EOF;
    ctx->tx_hdr_len = header_index;
}

void gmp_dev_dl_reply_ack(gmp_datalink_t* ctx)
{
    if (ctx == NULL)
        return;
    gmp_dev_dl_tx_request_cmd(ctx, ctx->rx_head.seq_id, ctx->rx_head.cmd);
    ctx->flag_reply_handled = 1;
}

void gmp_dev_dl_reply_ack_null(gmp_datalink_t* ctx)
{
    if (ctx == NULL)
        return;
    gmp_dev_dl_tx_request_cmd(ctx, ctx->rx_head.seq_id, ctx->rx_head.cmd);
    gmp_dev_dl_tx_ready(ctx);
    ctx->flag_reply_handled = 1;
}

void gmp_dev_dl_reply_nack(gmp_datalink_t* ctx, uint16_t error_code)
{
    uint8_t payload[2];
    if (ctx == NULL)
        return;
    payload[0] = (uint8_t)ctx->rx_head.cmd;
    payload[1] = (uint8_t)error_code;
    gmp_dev_dl_tx_request(ctx, ctx->rx_head.seq_id, GMP_DL_CMD_NACK, 2, payload);
    ctx->flag_reply_handled = 1;
}

void gmp_dev_dl_tx_state_done(gmp_datalink_t* ctx)
{
    if (ctx != NULL)
        ctx->tx_state = GMP_DL_TX_STATE_IDLE;
}

gmp_dl_tx_state_t gmp_dev_dl_tx_state_next(gmp_datalink_t* ctx)
{
    if (ctx == NULL)
        return GMP_DL_TX_STATE_IDLE;
    switch (ctx->tx_state)
    {
    case GMP_DL_TX_STATE_PENDING_HW:
        ctx->tx_state = GMP_DL_TX_STATE_PENDING_HW_HDR;
        break;
    case GMP_DL_TX_STATE_PENDING_HW_HDR:
        ctx->tx_state = (ctx->tx_len > 0) ? GMP_DL_TX_STATE_PENDING_HW_PLD : GMP_DL_TX_STATE_IDLE;
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
    if (ctx != NULL)
        ctx->flag_reply_handled = 1;
}
