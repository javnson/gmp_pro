
#include <gmp_core.h>

#include <string.h>

#include <core/dev/tunable_sim.h>

/**
 * @brief Default implementation of the simulation step. 
 * Marked as weak to allow user override without function pointers.
 */
#if defined(__GNUC__) || defined(__TI_COMPILER_VERSION__)
__attribute__((weak)) void gmp_sim_step_callback(const gmp_sim_rx_buf_t* rx, gmp_sim_tx_buf_t* tx)
{
    // Default: Do nothing
}
#endif

// =========================================================
// STRICT 8-BIT SERIALIZATION (For Universal Compatibility)
// =========================================================

static inline void sim_pack_8(data_gt* buf, size_gt* idx, uint8_t val)
{
    buf[(*idx)++] = (data_gt)(val & 0xFF);
}

static inline void sim_pack_16(data_gt* buf, size_gt* idx, uint16_t val)
{
    sim_pack_8(buf, idx, (uint8_t)(val & 0xFF));
    sim_pack_8(buf, idx, (uint8_t)((val >> 8) & 0xFF));
}

static inline void sim_pack_32(data_gt* buf, size_gt* idx, uint32_t val)
{
    sim_pack_16(buf, idx, (uint16_t)(val & 0xFFFF));
    sim_pack_16(buf, idx, (uint16_t)((val >> 16) & 0xFFFF));
}

static inline uint8_t sim_unpack_8(const data_gt* buf, size_gt* idx)
{
    return (uint8_t)(buf[(*idx)++] & 0xFF);
}

static inline uint16_t sim_unpack_16(const data_gt* buf, size_gt* idx)
{
    uint16_t low = sim_unpack_8(buf, idx);
    uint16_t high = sim_unpack_8(buf, idx);
    return (uint16_t)(low | (high << 8));
}

static inline uint32_t sim_unpack_32(const data_gt* buf, size_gt* idx)
{
    uint32_t low = sim_unpack_16(buf, idx);
    uint32_t high = sim_unpack_16(buf, idx);
    return (uint32_t)(low | (high << 16));
}

// =========================================================
// INTERNAL HELPERS FOR DYNAMIC PACKING/UNPACKING
// =========================================================

static inline void unpack_rx_buffer(gmp_tunable_sim_t* ctx, const data_gt* payload)
{
    size_gt p_idx = 0;
    ctx->rx_buf.isr_ticks = sim_unpack_32(payload, &p_idx);
    ctx->rx_buf.digital_input = sim_unpack_32(payload, &p_idx);

    for (int i = 0; i < 24; i++)
    {
        if ((ctx->mask_rx.all >> i) & 1)
            ctx->rx_buf.adc_result[i] = sim_unpack_16(payload, &p_idx);
    }
    for (int i = 0; i < 8; i++)
    {
        if ((ctx->mask_rx.all >> (24 + i)) & 1)
        {
            uint32_t raw = sim_unpack_32(payload, &p_idx);
            ctx->rx_buf.panel[i] = *((ctrl_gt*)&raw);
        }
    }
}

static inline size_gt pack_tx_buffer(gmp_tunable_sim_t* ctx, data_gt* tx_payload)
{
    size_gt tx_idx = 0;
    sim_pack_32(tx_payload, &tx_idx, ctx->mask_tx.all); // Always send current mask
    sim_pack_32(tx_payload, &tx_idx, ctx->tx_buf.digital_out);

    for (int i = 0; i < 8; i++)
    {
        if ((ctx->mask_tx.all >> i) & 1)
            sim_pack_16(tx_payload, &tx_idx, ctx->tx_buf.pwm_cmp[i]);
    }
    for (int i = 0; i < 8; i++)
    {
        if ((ctx->mask_tx.all >> (8 + i)) & 1)
            sim_pack_16(tx_payload, &tx_idx, ctx->tx_buf.dac[i]);
    }
    for (int i = 0; i < 16; i++)
    {
        if ((ctx->mask_tx.all >> (16 + i)) & 1)
        {
            uint32_t raw = *((uint32_t*)&ctx->tx_buf.monitor[i]);
            sim_pack_32(tx_payload, &tx_idx, raw);
        }
    }
    return tx_idx;
}

// =========================================================
// API IMPLEMENTATIONS
// =========================================================

void gmp_tunable_sim_init(gmp_tunable_sim_t* ctx, gmp_datalink_t* dl_ctx, uint16_t base_cmd)
{
    if (!ctx)
        return;
    memset(ctx, 0, sizeof(gmp_tunable_sim_t));
    ctx->dl_ctx = dl_ctx;
    ctx->base_cmd = base_cmd;

    // Initial state: Enable basic I/O by default
    ctx->mask_tx.all = 0xFFFFFFFF;
    ctx->mask_rx.all = 0xFFFFFFFF;
}

fast_gt gmp_tunable_sim_rx_cb(gmp_tunable_sim_t* ctx, uint16_t target_id, uint16_t cmd, const data_gt* payload,
                              size_gt len)
{
    if (!ctx || cmd < ctx->base_cmd)
        return GMP_TUNABLE_PASS;
    uint16_t offset = cmd - ctx->base_cmd;

    switch (offset)
    {
    case GMP_TUNABLE_OFFSET_SIM_SET_MASK_REQ: {
        if (len < 8)
            return GMP_TUNABLE_HANDLED;
        size_gt p_idx = 0;
        ctx->mask_tx.all = sim_unpack_32(payload, &p_idx);
        ctx->mask_rx.all = sim_unpack_32(payload, &p_idx);

        data_gt tx_payload[4];
        size_gt tx_idx = 0;
        sim_pack_32(tx_payload, &tx_idx, GMP_EC_OK);
        gmp_datalink_send(ctx->dl_ctx, target_id, ctx->base_cmd + GMP_TUNABLE_OFFSET_SIM_SET_MASK_ACK, tx_payload,
                          tx_idx);
        return GMP_TUNABLE_HANDLED;
    }

    // --- 核心步进：吃输入 -> 算一步 -> 吐输出 ---
    case GMP_TUNABLE_OFFSET_SIM_STEP_REQ: {
        unpack_rx_buffer(ctx, payload);

        gmp_sim_step_callback(&ctx->rx_buf, &ctx->tx_buf);

        data_gt tx_payload[GMP_DL_MTU];
        size_gt tx_idx = pack_tx_buffer(ctx, tx_payload);
        gmp_datalink_send(ctx->dl_ctx, target_id, ctx->base_cmd + GMP_TUNABLE_OFFSET_SIM_STEP_ACK, tx_payload, tx_idx);
        return GMP_TUNABLE_HANDLED;
    }

    // --- 【新增】静默输入：吃输入 -> 不运行 ---
    case GMP_TUNABLE_OFFSET_SIM_SET_INPUT_REQ: {
        unpack_rx_buffer(ctx, payload);

        // 简单回复一个 4 字节的 OK 状态
        data_gt tx_payload[4];
        size_gt tx_idx = 0;
        sim_pack_32(tx_payload, &tx_idx, GMP_EC_OK);
        gmp_datalink_send(ctx->dl_ctx, target_id, ctx->base_cmd + GMP_TUNABLE_OFFSET_SIM_SET_INPUT_ACK, tx_payload,
                          tx_idx);
        return GMP_TUNABLE_HANDLED;
    }

    // --- 【新增】静默输出：不运行 -> 吐输出 ---
    case GMP_TUNABLE_OFFSET_SIM_GET_OUTPUT_REQ: {
        data_gt tx_payload[GMP_DL_MTU];
        size_gt tx_idx = pack_tx_buffer(ctx, tx_payload);
        // 与 STEP 吐出的数据格式完全一致，上位机可以用同一段逻辑解包
        gmp_datalink_send(ctx->dl_ctx, target_id, ctx->base_cmd + GMP_TUNABLE_OFFSET_SIM_GET_OUTPUT_ACK, tx_payload,
                          tx_idx);
        return GMP_TUNABLE_HANDLED;
    }

    default:
        return GMP_TUNABLE_PASS;
    }
}
