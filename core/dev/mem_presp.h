
#include <core/dev/datalink.h>


#ifndef GMP_MEM_PERSP_H
#define GMP_MEM_PERSP_H


// 内存读写权限
#define GMP_MEM_PERM_RO 0x00
#define GMP_MEM_PERM_RW 0x01

// 1. 内存沙箱块 (Memory Sandbox Region) 定义
typedef struct
{
    void* base_addr;      // 【修改】：存原生物理指针，躲过编译器的链接期乘法拦截
    uint32_t byte_length; // 允许访问的字节长度
    fast16_gt perm;       // 权限属性
} gmp_mem_region_t;

// 2. 透视服务对象 (Class Context)
typedef struct
{
    gmp_datalink_t* dl_ctx;          // 绑定的通信链路对象
    uint16_t base_cmd;               // 占用的基地址指令 (Read = base, Write = base + 1)
    const gmp_mem_region_t* regions; // 注册的沙箱白名单数组
    fast16_gt region_count;          // 白名单数量
} gmp_mem_persp_t;

// =========================================================
// API 声明
// =========================================================

/**
 * @brief 初始化内存透视对象
 */
void gmp_mem_persp_init(gmp_mem_persp_t* ctx, gmp_datalink_t* dl, uint16_t base_cmd, const gmp_mem_region_t* regions,
                        fast16_gt region_count);

/**
 * @brief 内存透视接收回调 (需放置在 RX_OK 事件处理链中)
 * @return 1 表示指令属于该对象并已处理, 0 表示不属于该对象
 */
fast_gt gmp_mem_persp_rx_cb(gmp_mem_persp_t* ctx);

#endif // GMP_MEM_PERSP_H
