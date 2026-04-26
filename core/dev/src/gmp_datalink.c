#include <gmp_core.h>

#include <core/dev/datalink.h>

#include <string.h> // for memset, memcpy

// =========================================================
// 外部依赖声明 (由基础库实现)
// =========================================================
extern uint16_t gmp_base_calculate_crc16(const data_gt* buf, size_gt len);
extern time_gt gmp_base_get_system_tick(void);
extern fast_gt gmp_base_is_delay_elapsed(time_gt start_tick, time_gt delay_ms);

// =========================================================
// 内部辅助函数
// =========================================================

/**
 * @brief 内部复位 RX 状态机
 */
static inline void datalink_reset_rx(gmp_datalink_t* ctx)
{
    ctx->rx_state = GMP_DL_STATE_WAIT_SYNC;
    ctx->rx_hdr_idx = 0;
    ctx->payload_idx = 0;
}

// =========================================================
// 初始化与 ISR 数据推入 API
// =========================================================

void gmp_dev_dl_init(gmp_datalink_t* ctx)
{
    memset(ctx, 0, sizeof(gmp_datalink_t));
    ctx->rx_state = GMP_DL_STATE_WAIT_SYNC;
    ctx->tx_state = GMP_DL_TX_STATE_IDLE;
}

void gmp_dev_dl_push_byte(gmp_datalink_t* ctx, data_gt raw_data)
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
    }
}

void gmp_dev_dl_push_str(gmp_datalink_t* ctx, const data_gt* str, size_gt size)
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
            // FIFO 满，累加剩余丢弃数量并提前退出
            ctx->err_fifo_ovf_cnt += (size - i);
            break;
        }
    }
}

// =========================================================
// 核心状态机驱动 API
// =========================================================

gmp_dl_event_t gmp_dev_dl_loop_cb(gmp_datalink_t* ctx)
{
    // -----------------------------------------------------
    // 优先级 1：处理待封包的 TX 数据
    // -----------------------------------------------------
    if (ctx->tx_state == GMP_DL_TX_STATE_READY_TO_WARP)
    {
        gmp_dev_dl_tx_warp(ctx);
        ctx->tx_state = GMP_DL_TX_STATE_PENDING_HW;
        return GMP_DL_EVENT_TX_RDY;
    }

    // -----------------------------------------------------
    // 优先级 2：如果底层硬件正在发送，阻塞 RX 解析
    // 扩充了对 DMA 子状态的拦截，彻底保护正在发送的物理缓冲区
    // -----------------------------------------------------
    if (ctx->tx_state == GMP_DL_TX_STATE_PENDING_HW || ctx->tx_state == GMP_DL_TX_STATE_PENDING_HW_HDR ||
        ctx->tx_state == GMP_DL_TX_STATE_PENDING_HW_PLD)
    {
        return GMP_DL_EVENT_IDLE;
    }

    // -----------------------------------------------------
    // 优先级 3：执行 RX 解析状态机
    // -----------------------------------------------------
    while (ctx->rx_fifo_tail != ctx->rx_fifo_head)
    {
        data_gt byte = ctx->rx_fifo[ctx->rx_fifo_tail];

        // 弹出队列
        ctx->rx_fifo_tail++;
        if (ctx->rx_fifo_tail >= GMP_DL_RX_FIFO_SIZE)
        {
            ctx->rx_fifo_tail = 0;
        }

        ctx->last_rx_tick = gmp_base_get_system_tick();

        // --- 强制复位保护 ---
        if ((ctx->rx_state == GMP_DL_STATE_HEADER_RECV || ctx->rx_state == GMP_DL_STATE_HEADER_ESCAPE) &&
            byte == GMP_DL_SOF)
        {
            datalink_reset_rx(ctx);
            ctx->rx_state = GMP_DL_STATE_HEADER_RECV;
            continue;
        }

        // --- 状态流转 ---
        switch (ctx->rx_state)
        {
        case GMP_DL_STATE_WAIT_SYNC:
            if (byte == GMP_DL_SOF)
            {
                ctx->rx_state = GMP_DL_STATE_HEADER_RECV;
                ctx->rx_hdr_idx = 0;
                ctx->payload_idx = 0;
            }
            else
            {
                // 游离字符路由：包装成单字符包抛给主循环
                ctx->rx_head.seq_id = 0;
                ctx->rx_head.cmd = GMP_DL_CMD_STRAY;
                ctx->payload_buf[0] = byte;
                ctx->expected_payload_len = 1;
                ctx->flag_reply_handled = 0;
                return GMP_DL_EVENT_RX_OK; // 立刻移交控制权
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

                // 校验 Header CRC
                uint16_t h_crc_rcv = (ctx->rx_hdr_buf[4] & 0xFF) | ((ctx->rx_hdr_buf[5] & 0xFF) << 8);
                uint16_t h_crc_calc = gmp_base_calculate_crc16(ctx->rx_hdr_buf, 4);

                if (h_crc_calc != h_crc_rcv)
                {
                    ctx->err_hdr_crc_cnt++;
                    datalink_reset_rx(ctx);
                    break;
                }

                // 解析合法 Header
                ctx->rx_head.seq_id = ctx->rx_hdr_buf[0] & 0xFF;
                ctx->rx_head.cmd = ctx->rx_hdr_buf[1] & 0xFF;
                ctx->expected_payload_len = (ctx->rx_hdr_buf[2] & 0xFF) | ((ctx->rx_hdr_buf[3] & 0xFF) << 8);

                if (ctx->expected_payload_len > GMP_DL_MTU)
                {
                    datalink_reset_rx(ctx);
                    break;
                }

                // 零长度载荷优化：短路，不接收 PCRC
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
            else
            {
                if (ctx->rx_hdr_idx < sizeof(ctx->rx_hdr_buf) / sizeof(ctx->rx_hdr_buf[0]))
                {
                    ctx->rx_hdr_buf[ctx->rx_hdr_idx++] = byte;
                }
            }
            break;

        case GMP_DL_STATE_HEADER_ESCAPE:
            if (ctx->rx_hdr_idx < sizeof(ctx->rx_hdr_buf) / sizeof(ctx->rx_hdr_buf[0]))
            {
                ctx->rx_hdr_buf[ctx->rx_hdr_idx++] = byte ^ GMP_DL_XOR;
            }
            ctx->rx_state = GMP_DL_STATE_HEADER_RECV;
            break;

        case GMP_DL_STATE_PAYLOAD_RECV:
            ctx->payload_buf[ctx->payload_idx++] = byte;

            // 盲收完成：载荷长度 + 2字节 PCRC
            if (ctx->payload_idx == (ctx->expected_payload_len + 2))
            {
                uint16_t p_crc_rcv = (ctx->payload_buf[ctx->expected_payload_len] & 0xFF) |
                                     ((ctx->payload_buf[ctx->expected_payload_len + 1] & 0xFF) << 8);
                uint16_t p_crc_calc = gmp_base_calculate_crc16(ctx->payload_buf, ctx->expected_payload_len);

                if (p_crc_calc == p_crc_rcv)
                {
                    ctx->flag_reply_handled = 0;
                    datalink_reset_rx(ctx);
                    return GMP_DL_EVENT_RX_OK; // 返回完整帧
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

    // -----------------------------------------------------
    // 优先级 4：看门狗超时检测 (仅在非空闲态)
    // -----------------------------------------------------
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

// =========================================================
// TX 构建器 (Builder Pattern) API
// =========================================================

void gmp_dev_dl_tx_request_cmd(gmp_datalink_t* ctx, uint16_t seq, uint16_t cmd)
{
    if (ctx->tx_state != GMP_DL_TX_STATE_IDLE)
        return;

    ctx->tx_head.seq_id = seq;
    ctx->tx_head.cmd = cmd;
    ctx->tx_len = 0;
    ctx->tx_state = GMP_DL_TX_STATE_BUILDING;
}

void gmp_dev_dl_tx_append_payload(gmp_datalink_t* ctx, const data_gt* data, size_gt actual_payload_len)
{
    if (ctx->tx_state != GMP_DL_TX_STATE_BUILDING)
        return;
    if (actual_payload_len == 0 || data == NULL)
        return;

    if ((ctx->tx_len + actual_payload_len) > GMP_DL_MTU)
        return;

    memcpy(&ctx->tx_buf[ctx->tx_len], data, actual_payload_len * sizeof(data_gt));
    ctx->tx_len += actual_payload_len;
}

void gmp_dev_dl_tx_append_u8(gmp_datalink_t* ctx, data_gt val)
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
                           const data_gt* data)
{
    gmp_dev_dl_tx_request_cmd(ctx, seq, cmd);
    gmp_dev_dl_tx_append_payload(ctx, actual_payload_len, data);
    gmp_dev_dl_tx_ready(ctx);
}

// =========================================================
// TX 缓冲区探针与物理获取 API
// =========================================================

size_gt gmp_dev_dl_get_tx_capacity(gmp_datalink_t* ctx)
{
    if (ctx->tx_state != GMP_DL_TX_STATE_BUILDING)
        return 0;
    return GMP_DL_MTU - ctx->tx_len;
}

data_gt* gmp_dev_dl_get_tx_payload_ptr(gmp_datalink_t* ctx)
{
    if (ctx->tx_state != GMP_DL_TX_STATE_BUILDING)
        return NULL;
    return &ctx->tx_buf[ctx->tx_len];
}

const data_gt* gmp_dev_dl_get_tx_hw_hdr(gmp_datalink_t* ctx, size_gt* out_len)
{
    if (out_len)
        *out_len = ctx->tx_hdr_len;
    return ctx->tx_hdr_buf;
}

const data_gt* gmp_dev_dl_get_tx_hw_pld(gmp_datalink_t* ctx, size_gt* out_len)
{
    if (out_len)
        *out_len = ctx->tx_len;
    return ctx->tx_buf;
}

// =========================================================
// 封包与转义核心引擎 (Warp)
// =========================================================

void gmp_dev_dl_tx_warp(gmp_datalink_t* ctx)
{
    size_gt i;

    // 【优化】提前保存纯业务载荷的真实长度，避免后续的减法运算混淆
    uint16_t pld_len = ctx->tx_len;

    // 1. 如果有 Payload，追加 P_CRC (零长度则不追加)
    if (pld_len > 0)
    {
        uint16_t p_crc = gmp_base_calculate_crc16(ctx->tx_buf, pld_len);
        ctx->tx_buf[ctx->tx_len++] = p_crc & 0xFF;
        ctx->tx_buf[ctx->tx_len++] = (p_crc >> 8) & 0xFF;
        // 此时 ctx->tx_len 变成了物理载荷总长度 (Payload + 2)
    }

    // 2. 构建未转义的原始 Header
    data_gt raw_hdr[6];
    raw_hdr[0] = ctx->tx_head.seq_id & 0xFF;
    raw_hdr[1] = ctx->tx_head.cmd & 0xFF;
    raw_hdr[2] = pld_len & 0xFF;
    raw_hdr[3] = (pld_len >> 8) & 0xFF;

    uint16_t h_crc = gmp_base_calculate_crc16(raw_hdr, 4);
    raw_hdr[4] = h_crc & 0xFF;
    raw_hdr[5] = (h_crc >> 8) & 0xFF;

    // 3. 将原始 Header 转义并写入 tx_hdr_buf
    size_gt h_idx = 0;
    ctx->tx_hdr_buf[h_idx++] = GMP_DL_SOF; // 帧首标志

    for (i = 0; i < 6; i++)
    {
        data_gt b = raw_hdr[i];
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

    ctx->tx_hdr_buf[h_idx++] = GMP_DL_EOF; // 帧头结束标志
    ctx->tx_hdr_len = h_idx;
}

// =========================================================
// 回复 API (Slave Mode)
// =========================================================

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
    data_gt nack_payload[2];
    nack_payload[0] = ctx->rx_head.cmd & 0xFF; // 引发错误的指令
    nack_payload[1] = error_code & 0xFF;       // 错误原因

    gmp_dev_dl_tx_request(ctx, ctx->rx_head.seq_id, GMP_DL_CMD_NACK, 2, nack_payload);
    ctx->flag_reply_handled = 1;
}

// =========================================================
// 硬件发送状态流转 API
// =========================================================

void gmp_dev_dl_tx_state_done(gmp_datalink_t* ctx)
{
    ctx->tx_state = GMP_DL_TX_STATE_IDLE;
}

gmp_dl_tx_state_t gmp_dev_dl_tx_state_next(gmp_datalink_t* ctx)
{
    switch (ctx->tx_state)
    {
    case GMP_DL_TX_STATE_PENDING_HW:
        // 初始触发，先推入 Header 发送阶段
        ctx->tx_state = GMP_DL_TX_STATE_PENDING_HW_HDR;
        break;

    case GMP_DL_TX_STATE_PENDING_HW_HDR:
        // Header 发送完毕，检查是否还有 Payload 需要发送
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
        // Payload 发送完毕，或处于异常状态，强制回归空闲
        ctx->tx_state = GMP_DL_TX_STATE_IDLE;
        break;
    }

    return ctx->tx_state;
}
