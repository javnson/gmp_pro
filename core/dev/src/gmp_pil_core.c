#include <gmp_core.h>

#include <core/dev/pil_core.h> // 根据你的实际路径调整

#include <string.h> // for memset

// =========================================================
// 内部辅助函数：安全的 Little-Endian 反序列化提取器
// =========================================================

/**
 * @brief 从 Datalink 的 payload 缓冲区中安全提取一个 16-bit 整数
 */
static inline uint16_t sim_unpack_u16(const data_gt* buf, uint16_t* idx, uint16_t max_len)
{
    if (*idx + 2 > max_len)
        return 0; // 越界保护

    uint16_t val = ((uint16_t)(buf[*idx] & 0xFF)) | (((uint16_t)(buf[*idx + 1] & 0xFF)) << 8);
    *idx += 2;
    return val;
}

/**
 * @brief 从 Datalink 的 payload 缓冲区中安全提取一个 32-bit 整数
 */
static inline uint32_t sim_unpack_u32(const data_gt* buf, uint16_t* idx, uint16_t max_len)
{
    if (*idx + 4 > max_len)
        return 0; // 越界保护

    uint32_t val = ((uint32_t)(buf[*idx] & 0xFF)) | (((uint32_t)(buf[*idx + 1] & 0xFF)) << 8) |
                   (((uint32_t)(buf[*idx + 2] & 0xFF)) << 16) | (((uint32_t)(buf[*idx + 3] & 0xFF)) << 24);
    *idx += 4;
    return val;
}

// =========================================================
// 核心组包与解包引擎
// =========================================================

/**
 * @brief 根据 mask_rx 将 PC 传来的 payload 解包到 rx_buf 结构体中
 */
static void sim_deserialize_inputs(gmp_pil_sim_t* ctx)
{
    const data_gt* pld = ctx->dl_ctx->payload_buf;
    uint16_t pld_len = ctx->dl_ctx->expected_payload_len;
    uint16_t idx = 0;
    size_gt i;
    gmp_safe_pun_t pun;

    // 1. 提取基础状态 (必须存在的定长头部)
    ctx->rx_buf.isr_ticks = sim_unpack_u32(pld, &idx, pld_len);
    ctx->rx_buf.digital_input = sim_unpack_u32(pld, &idx, pld_len);

    // 2. 根据 Mask 提取 ADC 结果 (24个通道)
    for (i = 0; i < 24; i++)
    {
        if (ctx->mask_rx.bit.adc_result & (1UL << i))
        {
            ctx->rx_buf.adc_result[i] = sim_unpack_u16(pld, &idx, pld_len);
        }
    }

    // 3. 根据 Mask 提取 Panel 虚拟面板数据 (8个通道，32-bit ctrl_gt)
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
 * @brief 根据 mask_tx 将 tx_buf 结构体中的数据打包到发送缓冲区
 * @note 必须在调用 gmp_dev_dl_tx_request_cmd 之后调用此函数！
 */
static void sim_serialize_outputs(gmp_pil_sim_t* ctx)
{
    size_gt i;
    gmp_safe_pun_t pun;
    gmp_datalink_t* dl = ctx->dl_ctx;

    // 1. 压入基础状态
    gmp_dev_dl_tx_append_u32(dl, ctx->tx_buf.digital_out);

    // 2. 根据 Mask 压入 PWM Compare 数据 (8个通道)
    for (i = 0; i < 8; i++)
    {
        if (ctx->mask_tx.bit.pwm_cmp & (1UL << i))
        {
            gmp_dev_dl_tx_append_u16(dl, ctx->tx_buf.pwm_cmp[i]);
        }
    }

    // 3. 根据 Mask 压入 DAC 数据 (8个通道)
    for (i = 0; i < 8; i++)
    {
        if (ctx->mask_tx.bit.dac & (1UL << i))
        {
            gmp_dev_dl_tx_append_u16(dl, ctx->tx_buf.dac[i]);
        }
    }

    // 4. 根据 Mask 压入 Monitor 监控变量 (16个通道，32-bit ctrl_gt)
    for (i = 0; i < 16; i++)
    {
        if (ctx->mask_tx.bit.monitor & (1UL << i))
        {
            pun.f_val = ctx->tx_buf.monitor[i];
            gmp_dev_dl_tx_append_u32(dl, pun.u_val);
        }
    }
}

// =========================================================
// API 实现
// =========================================================

void gmp_pil_sim_init(gmp_pil_sim_t* ctx, gmp_datalink_t* dl_ctx, uint16_t base_cmd)
{
    memset(ctx, 0, sizeof(gmp_pil_sim_t));
    ctx->dl_ctx = dl_ctx;
    ctx->base_cmd = base_cmd;

    // 默认全开，或者全关，取决于你的仿真设计
    ctx->mask_tx.all = 0xFFFFFFFF;
    ctx->mask_rx.all = 0xFFFFFFFF;
}

fast_gt gmp_pil_sim_rx_cb(gmp_pil_sim_t* ctx)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    uint16_t rcv_cmd = dl->rx_head.cmd;

    // 检查此指令是否属于当前 PIL 子系统的控制域
    if (rcv_cmd < ctx->base_cmd || rcv_cmd > ctx->base_cmd + 4)
    {
        return 0; // 不属于我，Pass给其他模块或兜底函数
    }

    // 提取具体的 Offset
    uint16_t offset = rcv_cmd - ctx->base_cmd;
    uint16_t idx = 0;

    switch (offset)
    {
    case GMP_PIL_OFFSET_SIM_SET_MASK_REQ:
        // 解析 8 字节：前 4 字节为 TX Mask，后 4 字节为 RX Mask
        if (dl->expected_payload_len >= 8)
        {
            ctx->mask_tx.all = sim_unpack_u32(dl->payload_buf, &idx, dl->expected_payload_len);
            ctx->mask_rx.all = sim_unpack_u32(dl->payload_buf, &idx, dl->expected_payload_len);
        }

        // 【优化】：直接构建 8 字节的镜像回传包 (去除 Status 占位符)
        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, rcv_cmd);
        gmp_dev_dl_tx_append_u32(dl, ctx->mask_tx.all); // 回传实际应用的 TX Mask
        gmp_dev_dl_tx_append_u32(dl, ctx->mask_rx.all); // 回传实际应用的 RX Mask
        gmp_dev_dl_tx_ready(dl);
        break;

    case GMP_PIL_OFFSET_SIM_STEP_REQ:
        // 1. 将接收到的字节流解包到结构体
        sim_deserialize_inputs(ctx);

        // 2. 执行用户算法逻辑 (Weak callback / 外部实现)
        gmp_pil_sim_step(&ctx->rx_buf, &ctx->tx_buf);

        // 3. 将计算结果打包回传
        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, rcv_cmd);
        sim_serialize_outputs(ctx);
        gmp_dev_dl_tx_ready(dl);
        break;

    case GMP_PIL_OFFSET_SIM_SET_INPUT_REQ:
        // 仅解包覆盖输入数据，不触发控制算法步进
        sim_deserialize_inputs(ctx);
        gmp_dev_dl_reply_ack_null(dl);
        break;

    case GMP_PIL_OFFSET_SIM_GET_OUTPUT_REQ:
        // 仅索要当前的计算输出状态，不修改输入
        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, rcv_cmd);
        sim_serialize_outputs(ctx);
        gmp_dev_dl_tx_ready(dl);
        break;

    default:
        // 异常的 offset
        gmp_dev_dl_reply_nack(dl, 0x0001);
        break;
    }

    // 【规范化接口】通知底层 Datalink：这笔交易我已经闭环了，不需要执行默认兜底了！
    gmp_dev_dl_msg_handled(dl);

    return 1; // 成功处理
}

// =========================================================
// 弱定义算法
// =========================================================

//#pragma weak gmp_pil_sim_step
//void gmp_pil_sim_step(const gmp_sim_rx_buf_t* rx, gmp_sim_tx_buf_t* tx) {
//    // 默认实现为空
//}
