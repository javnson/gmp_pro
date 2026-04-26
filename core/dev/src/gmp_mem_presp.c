#include <gmp_core.h>

#include <core/dev/mem_presp.h>

// =========================================================
// 内部辅助工具
// =========================================================

static inline fast16_gt mem_unpack_u8(const data_gt* buf, uint16_t* idx, uint16_t max_len)
{
    if (*idx >= max_len)
        return 0;
    fast16_gt val = buf[*idx] & 0xFF;
    *idx += 1;
    return val;
}

static inline uint32_t mem_unpack_u32(const data_gt* buf, uint16_t* idx, uint16_t max_len)
{
    if (*idx + 4 > max_len)
        return 0;
    uint32_t val = ((uint32_t)(buf[*idx] & 0xFF)) | (((uint32_t)(buf[*idx + 1] & 0xFF)) << 8) |
                   (((uint32_t)(buf[*idx + 2] & 0xFF)) << 16) | (((uint32_t)(buf[*idx + 3] & 0xFF)) << 24);
    *idx += 4;
    return val;
}

// =========================================================
// 核心逻辑实现
// =========================================================

void gmp_mem_persp_init(gmp_mem_persp_t* ctx, gmp_datalink_t* dl, uint16_t base_cmd, const gmp_mem_region_t* regions,
                        fast16_gt region_count)
{
    ctx->dl_ctx = dl;
    ctx->base_cmd = base_cmd;
    ctx->regions = regions;
    ctx->region_count = region_count;
}

/**
 * @brief 安全沙箱越界校验引擎
 * @param addr 目标起始字节地址
 * @param total_bytes 欲访问的总字节数
 * @param is_write 是否为写操作
 * @return 1: 合法, 0: 越界或无权限
 */
static fast_gt check_sandbox_validity(gmp_mem_persp_t* ctx, uint32_t addr, uint32_t total_bytes, fast16_gt is_write)
{
    size_gt i;

    for (i = 0; i < ctx->region_count; i++)
    {
        const gmp_mem_region_t* reg = &ctx->regions[i];

        // 【核心修改】：在运行期，将原生指针转换为统一的“字节地址当量”
        uint32_t safe_start_addr = ((uint32_t)reg->base_addr) * GMP_PORT_DATA_SIZE_PER_BYTES;

        // 检查是否完全被包裹在沙箱内部
        if (addr >= safe_start_addr && (addr + total_bytes) <= (safe_start_addr + reg->byte_length))
        {
            if (is_write && reg->perm != GMP_MEM_PERM_RW)
                return 0; // 写操作遭遇 RO 区域
            return 1;
        }
    }
    return 0; // 不在任何白名单沙箱中
}

fast_gt gmp_mem_persp_rx_cb(gmp_mem_persp_t* ctx)
{
    gmp_datalink_t* dl = ctx->dl_ctx;
    uint16_t cmd = dl->rx_head.cmd;

    if (cmd != ctx->base_cmd && cmd != (ctx->base_cmd + 1))
        return 0;

    uint16_t len = dl->expected_payload_len;
    uint16_t idx = 0;
    const data_gt* pld = dl->payload_buf;

    // 解析请求头部: [Addr(4B)] + [Item Size(1B)] + [Item Count(2B)]
    uint32_t target_addr = mem_unpack_u32(pld, &idx, len);
    fast16_gt item_size = mem_unpack_u8(pld, &idx, len);
    uint16_t item_count = mem_unpack_u8(pld, &idx, len) | (mem_unpack_u8(pld, &idx, len) << 8);

    uint32_t total_bytes = (uint32_t)item_size * item_count;
    uint8_t status = 0;

    size_gt c;

    // ==========================================
    // 内存读取 (READ)
    // ==========================================
    if (cmd == ctx->base_cmd)
    {
        if (!check_sandbox_validity(ctx, target_addr, total_bytes, 0))
        {
            status = 1; // 非法访问
        }

        gmp_dev_dl_tx_request_cmd(dl, dl->rx_head.seq_id, cmd);
        gmp_dev_dl_tx_append_u8(dl, status);

        if (status == 0)
        {
            for (c = 0; c < item_count; c++)
            {
                uint32_t byte_addr = target_addr + c * item_size;

// --- 跨平台原子读取引擎 ---
#if GMP_PORT_DATA_SIZE_PER_BYTES == 2                  // DSP C2000 架构
                uint32_t native_addr = byte_addr >> 1; // 转换为物理字指针
                if (item_size == 1)
                {
                    uint16_t word = *((uint16_t*)native_addr);
                    gmp_dev_dl_tx_append_u8(dl, (byte_addr & 1) ? (word >> 8) : (word & 0xFF));
                }
                else if (item_size == 2)
                {
                    uint16_t word = *((uint16_t*)native_addr); // 16-bit 原子读
                    gmp_dev_dl_tx_append_u8(dl, word & 0xFF);
                    gmp_dev_dl_tx_append_u8(dl, (word >> 8) & 0xFF);
                }
                else if (item_size == 4)
                {
                    uint32_t dword = *((uint32_t*)native_addr); // 32-bit 原子读
                    gmp_dev_dl_tx_append_u8(dl, dword & 0xFF);
                    gmp_dev_dl_tx_append_u8(dl, (dword >> 8) & 0xFF);
                    gmp_dev_dl_tx_append_u8(dl, (dword >> 16) & 0xFF);
                    gmp_dev_dl_tx_append_u8(dl, (dword >> 24) & 0xFF);
                }
#else // ARM/Zynq 架构
                uint32_t native_addr = byte_addr;
                if (item_size == 1)
                {
                    gmp_dev_dl_tx_append_u8(dl, *((uint8_t*)native_addr));
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

    // ==========================================
    // 内存写入 (WRITE)
    // ==========================================
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

// --- 跨平台原子写入引擎 ---
#if GMP_PORT_DATA_SIZE_PER_BYTES == 2 // DSP C2000 架构
                uint32_t native_addr = byte_addr >> 1;
                if (item_size == 1)
                {
                    uint8_t val = mem_unpack_u8(pld, &idx, len);
                    uint16_t word = *((uint16_t*)native_addr);
                    // 读-改-写 (C2000 的 8-bit 切片拼接)
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
#else // ARM/Zynq 架构
                uint32_t native_addr = byte_addr;
                if (item_size == 1)
                {
                    *((uint8_t*)native_addr) = mem_unpack_u8(pld, &idx, len);
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
