

#include <core/dev/ring_buf.h>

#ifndef _FILE_AT_DEVICE_H_
#define _FILE_AT_DEVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// AT command type enum
typedef enum
{
    AT_CMD_TYPE_EXEC,  // AT+CMD execute
    AT_CMD_TYPE_QUERY, // AT+CMD? require
    AT_CMD_TYPE_TEST,  // AT+CMD=? test and help
    AT_CMD_TYPE_SETUP  // AT+CMD=... setting
} at_cmd_type_t;

// Command State
typedef enum
{
    AT_STATUS_OK = 0, // Command execute successfully
    AT_STATUS_ERROR,  // Command execute complete with errors.
    AT_STATUS_PENDING // Command pending and waiting for next call
} at_status_t;

// Error code of AT device
typedef enum
{
    AT_ERR_RX_OVERFLOW,   // 接收缓冲区溢出
    AT_ERR_LINE_OVERFLOW, // 单行命令超长
    AT_ERR_CMD_NOT_FOUND, // 未知命令
    AT_ERR_EXEC_FAIL      // 命令执行返回错误
} at_error_code_t;

/* 前置声明 */
struct _tag_at_device_entity;

/* 错误回调函数定义 */
typedef void (*at_error_handler_t)(struct _tag_at_device_entity* dev, at_error_code_t err);

// AT device command object
typedef struct _tag_at_device_cmd
{
    // name of device command，example "PWM"
    const char* name;

    // length of device command
    uint16_t name_length;

    // support attribute of this command
    uint16_t attr;

    // command handler
    at_status_t (*handler)(struct _tag_at_device_entity* dev, at_cmd_type_t type, char* args, uint16_t length);

    // help information of this command
    const char* help_info;
} at_device_cmd_t;

#ifndef AT_DEVICE_RX_BUFFER
#define AT_DEVICE_RX_BUFFER 128
#endif // AT_DEVICE_RX_BUFFER

#ifndef AT_CMD_MAX_LEN
#define AT_CMD_MAX_LEN 16 // 命令名最大长度
#endif                    // AT_CMD_MAX_LEN

#ifndef AT_LINE_MAX_LEN
#define AT_LINE_MAX_LEN 64 // 单行命令最大解析长度（用于从RingBuffer提取一行）
#endif                     // AT_LINE_MAX_LEN

/* AT 设备对象 */
typedef struct _tag_at_device_entity
{
    /* 接收缓冲区 */
    ringbuf_t buffer;
    data_gt mem_pool[AT_DEVICE_RX_BUFFER];

    /* 解析用行缓冲区 */
    char cmd_buffer[AT_LINE_MAX_LEN];
    uint16_t line_idx;

    /* 状态标志 */
    volatile fast_gt flag_overwrite; // 0:正常, 1:RingBuf溢出, 2:LineBuf溢出

    /* 命令表 (注意：必须是可写的 RAM 区域，以便排序) */
    at_device_cmd_t* cmd_table;
    uint16_t cmd_table_size;

    /* 异步/挂起上下文 */
    const at_device_cmd_t* pending_cmd; // 当前正在挂起的命令
    char* pending_args;                 // 挂起时的参数指针
    at_cmd_type_t pending_type;         // 挂起时的命令类型
    uint16_t pending_len;               // 挂起时的参数长度

    /* 用户回调 */
    at_error_handler_t error_callback;

    /* 用户自定义数据 (方便在回调中获取外部对象) */
    void* user_data;

} at_device_entity_t;

/* API */
void at_device_init(at_device_entity_t* dev, at_device_cmd_t* table, uint16_t table_size, at_error_handler_t err_cb);
void at_device_rx_isr(at_device_entity_t* dev, char* content, int len);
void at_device_dispatch(at_device_entity_t* dev);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_AT_DEVICE_H_
