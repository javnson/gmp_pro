#ifndef GMP_PARAM_H
#define GMP_PARAM_H

#include <core/dev/datalink.h>

// 1. 变量类型定义 (决定总线上的字节数)
typedef enum
{
    GMP_PARAM_TYPE_U16 = 0,
    GMP_PARAM_TYPE_I16,
    GMP_PARAM_TYPE_U32,
    GMP_PARAM_TYPE_I32,
    GMP_PARAM_TYPE_F32
} gmp_param_type_t;

// 2. 读写权限属性
#define GMP_PARAM_PERM_RO 0x00
#define GMP_PARAM_PERM_RW 0x01

// 3. 数据字典条目结构体 (静态存储区)
typedef struct
{
    void* addr;            // 变量物理地址
    gmp_param_type_t type; // 变量类型
    fast16_gt perm;        // 权限属性 (兼容 DSP 16-bit 对齐)
} gmp_param_item_t;

// 4. 可调参数组对象 (Class Context)
typedef struct
{
    gmp_datalink_t* dl_ctx;       // 绑定的通信链路对象
    uint16_t base_cmd;            // 占用的基地址指令 (Read = base, Write = base + 1)
    const gmp_param_item_t* dict; // 指向绑定的数据字典数组
    fast16_gt dict_size;          // 字典的最大容量 (最大支持 255)
} gmp_param_tunable_t;

// =========================================================
// API 声明
// =========================================================

/**
 * @brief 初始化可调参数组对象
 */
void gmp_param_tunable_init(gmp_param_tunable_t* ctx, gmp_datalink_t* dl, uint16_t base_cmd,
                            const gmp_param_item_t* dict, fast16_gt dict_size);

/**
 * @brief 可调参数组接收回调 (需放置在 RX_OK 事件处理链中)
 * @return 1 表示指令属于该对象并已处理, 0 表示不属于该对象
 */
fast_gt gmp_param_tunable_rx_cb(gmp_param_tunable_t* ctx);

#endif // GMP_PARAM_H
