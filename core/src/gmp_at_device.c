/**
 * @file gmp_at_device.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2026-01-09
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

#include <core/dev/at_device.h>

/* qsort 比较函数 */
static int cmd_compare(const void* a, const void* b)
{
    const at_device_cmd_t* cmd_a = (const at_device_cmd_t*)a;
    const at_device_cmd_t* cmd_b = (const at_device_cmd_t*)b;
    return strcmp(cmd_a->name, cmd_b->name);
}

/* 二分查找命令 */
static const at_device_cmd_t* find_cmd_binary(at_device_entity_t* dev, const char* name, uint16_t len)
{
    int left = 0;
    int right = dev->cmd_table_size - 1;

    while (left <= right)
    {
        int mid = left + (right - left) / 2;
        const at_device_cmd_t* cmd = &dev->cmd_table[mid];

        // 比较：注意需要限制长度，防止前缀匹配错误
        // 但 cmd->name 是以 \0 结尾的，而 name 只是字符串中间的一段
        // 所以我们用 strncmp 比较 len，同时确认 cmd->name 的长度也正好是 len

        int cmp = strncmp(name, cmd->name, len);
        if (cmp == 0)
        {
            if (strlen(cmd->name) == len)
            {
                return cmd; // 完全匹配
            }
            else
            {
                // 长度不同，视为不匹配。
                // 例如 name="CMD", cmd->name="CMDA". strncmp返回0，但其实cmd更大
                // 需要决定方向。这里简单处理：如果是前缀关系，长的更大。
                cmp = -1; // 这里的逻辑视具体字符串而定，严谨做法如下：
            }
        }

        // 修正比较逻辑，处理长度不一致问题
        if (cmp == 0)
        {
            // 前缀相同，比较长度
            if (len < strlen(cmd->name))
                cmp = -1;
            else
                cmp = 1;
        }

        if (cmp < 0)
        {
            right = mid - 1; // name < cmd->name
        }
        else
        {
            left = mid + 1; // name > cmd->name
        }
    }
    return NULL;
}

/* 错误上报包装 */
static void report_error(at_device_entity_t* dev, at_error_code_t code)
{
    if (dev->error_callback)
    {
        dev->error_callback(dev, code);
    }
}

/* ================= 核心 API ================= */

void at_device_init(at_device_entity_t* dev, at_device_cmd_t* table, uint16_t table_size, at_error_handler_t err_cb)
{
    gmp_base_assert(dev);

    // 1. 基础初始化
    ringbuf_init(&dev->buffer, dev->mem_pool, AT_DEVICE_RX_BUFFER);
    dev->line_idx = 0;
    dev->flag_overwrite = 0;
    dev->error_callback = err_cb;

    dev->pending_cmd = NULL;
    dev->pending_args = NULL;

    // 2. 注册命令表
    dev->cmd_table = table;
    dev->cmd_table_size = table_size;

    memset(dev->mem_pool, AT_DEVICE_RX_BUFFER, 0);
    memset(dev->cmd_buffer, AT_LINE_MAX_LEN, 0);

    // 3. 排序命令表 (优化：O(N) -> O(logN) 查找)
    // 警告：确保 table 指向的是 RAM 区域，否则这里会 Crash
    if (dev->cmd_table && dev->cmd_table_size > 1)
    {
        qsort(dev->cmd_table, dev->cmd_table_size, sizeof(at_device_cmd_t), cmd_compare);
    }
}

/* 中断接收函数 (ISR) */
void at_device_rx_isr(at_device_entity_t* dev, char* content, size_gt len)
{
    gmp_base_assert(dev);
    if (len <= 0)
        return;

    size_gt free = ringbuf_get_free(&dev->buffer);

    if (len > free)
    {
        // 空间不足：尽可能填满，然后标记溢出
        if (free > 0)
        {
            ringbuf_put_array(&dev->buffer, (data_gt*)content, free);
        }
        dev->flag_overwrite = 1; // 标记 RingBuffer 溢出
    }
    else
    {
        ringbuf_put_array(&dev->buffer, (data_gt*)content, len);
    }
}

/* 执行命令逻辑 */
static void process_line(at_device_entity_t* dev)
{
    char* line = dev->cmd_buffer;
    uint16_t len = dev->line_idx;

    //    // 1. 简单校验
    //    if (len < 2 || line[0] != 'A' || line[1] != 'T')
    //        return;

    // --- 1. 增强型帧头校验与噪声过滤 ---
    uint16_t start_idx = 0;
    uint16_t found_header = 0;

    // 遍历 buffer，寻找连续的 'A' 和 'T'
    // 注意：循环条件必须是 len - 1，防止检查 line[i+1] 时越界
    for (start_idx = 0; start_idx < (len - 1); start_idx++)
    {
        // 直接比较字符，兼容 C2000 的 16-bit char 特性
        if (line[start_idx] == 'A' && line[start_idx + 1] == 'T')
        {
            // 找到了，start_idx 现在指向 'A'
            found_header = 1;
            break;
        }
    }

    // 如果没找到 AT，或者剩余长度太短不足以构成指令，直接返回
    if (!found_header)
    {
        return;
    }

    // --- 2. 指针修正 ---
    // 将 line 指针向后移动，跳过噪声 (例如跳过 0xFF)
    line += start_idx;
    len -= start_idx;

    // 2. AT 握手命令硬解析
    if (len == 2 || (len == 3 && line[2] == '\r'))
    {
        gmp_base_print("OK\r\n");
        return;
    }

    // 3. 解析命令名
    char* cmd_start = line + 2;
    if (*cmd_start == '+')
        cmd_start++;

    char* args = NULL;
    at_cmd_type_t type = AT_CMD_TYPE_EXEC;

    // 寻找分隔符
    char* eq = strchr(cmd_start, '=');
    char* qm = strchr(cmd_start, '?');

    if (qm && (!eq || qm < eq))
    {
        // =?
        if (eq && eq == qm - 1)
        {
            type = AT_CMD_TYPE_TEST;
            *eq = '\0';
        }
        // ?
        else
        {
            type = AT_CMD_TYPE_QUERY;
            *qm = '\0';
        }
    }
    // =
    else if (eq)
    {
        type = AT_CMD_TYPE_SETUP;
        args = eq + 1;
        *eq = '\0';
    }

    uint16_t name_len = (uint16_t)strlen(cmd_start);

    // 4. 二分查找
    const at_device_cmd_t* cmd = find_cmd_binary(dev, cmd_start, name_len);

    if (cmd)
    {
        // 首次执行命令
        at_status_t status = AT_STATUS_OK;
        uint16_t args_len = args ? (uint16_t)strlen(args) : 0;

        if (cmd->handler)
        {
            status = cmd->handler(dev, type, args, args_len);
        }

        if (status == AT_STATUS_PENDING)
        {
            // 进入异步挂起模式
            dev->pending_cmd = cmd;
            // 指向 line_buffer 内部，只要 line_idx 不重置，就是安全的
            dev->pending_args = args;
            dev->pending_type = type;
            dev->pending_len = args_len;
            // 关键：直接返回，不重置 line_idx，保留buffer
            return;
        }
        else if (status == AT_STATUS_ERROR)
        {
            report_error(dev, AT_ERR_EXEC_FAIL);
        }
        // OK 或 ERROR 都视为结束
    }
    else
    {
        report_error(dev, AT_ERR_CMD_NOT_FOUND);
    }

    // 命令结束，准备解析下一行
    dev->line_idx = 0;
}

void at_device_dispatch(at_device_entity_t* dev)
{
    gmp_base_assert(dev);
    data_gt byte;

    // ------------------------------------------------
    // 0. 获取当前DMA/FIFO中的所有数据
    // ------------------------------------------------

    // ------------------------------------------------
    // 1. 异步命令处理 (Async Phase)
    // ------------------------------------------------
    if (dev->pending_cmd != NULL)
    {
        // 再次调用同一个 handler
        at_status_t status = dev->pending_cmd->handler(dev, dev->pending_type, dev->pending_args, dev->pending_len);

        if (status == AT_STATUS_PENDING)
        {
            return; // 仍然未完成，保持阻塞
        }

        // 命令完成 (OK 或 ERROR)
        dev->pending_cmd = NULL; // 清除挂起状态
        dev->pending_args = NULL;

        // 异步命令结束后，彻底清空 buffer，防止残留
        memset(dev->cmd_buffer, 0, AT_LINE_MAX_LEN);
        dev->line_idx = 0;

        if (status == AT_STATUS_ERROR)
        {
            report_error(dev, AT_ERR_EXEC_FAIL);
        }
    }

    // ------------------------------------------------
    // 2. 错误恢复 (Recovery Phase) - RingBuffer 溢出
    // ------------------------------------------------
    if (dev->flag_overwrite == 1)
    {
        report_error(dev, AT_ERR_RX_OVERFLOW);
        while (ringbuf_get_one(&dev->buffer, &byte))
            ; // Drain buffer

        // 溢出恢复时，彻底清空 buffer
        memset(dev->cmd_buffer, 0, AT_LINE_MAX_LEN);
        dev->line_idx = 0;

        dev->flag_overwrite = 0;
        return;
    }

    // ------------------------------------------------
    // 3. 数据解析 (Parsing Phase)
    // ------------------------------------------------
    while (ringbuf_get_one(&dev->buffer, &byte))
    {
        // LineBuffer 溢出恢复中... (寻找一行结束标志来复位)
        if (dev->flag_overwrite == 2)
        {
            // 既然我们要忽略 \n，那么只有收到 \r 才代表这一行（错误的行）真正结束了
            if (byte == '\r')
            {
                dev->flag_overwrite = 0; // 恢复正常状态

                // 行溢出恢复后，准备接纳新行前清空
                memset(dev->cmd_buffer, 0, AT_LINE_MAX_LEN);
                dev->line_idx = 0;
            }
            continue; // 丢弃当前字符（包括 \n 和其他字符），直到遇到 \r
        }

        // ---------------------------------------------------------
        // 核心解析逻辑调整：只认 \r，忽略 \n
        // ---------------------------------------------------------

        if (byte == '\r') // 收到回车符，执行命令
        {
            if (dev->line_idx > 0)
            {
                // 在此处强制封口，确保字符串安全
                dev->cmd_buffer[dev->line_idx] = '\0'; // 封口

                // 此时行缓冲中是一个标准的null-terminated string
                process_line(dev);

                // 检查是否进入了 Pending 状态
                if (dev->pending_cmd != NULL)
                {
                    return; // 立即退出，保留 cmd_buffer 内容供异步任务使用
                }

                // 正常命令处理完毕后，清空整个 buffer
                memset(dev->cmd_buffer, 0, AT_LINE_MAX_LEN);
                dev->line_idx = 0;
            }
            // else: line_idx == 0, 说明收到空指令 (例如连续的 \r\r)，直接忽略，buffer 已清空无需操作
        }
        else if (byte == '\n' || byte == '\t')
        {
            // 显式忽略换行符和制表符
            // 不处理，不存入 buffer，直接跳过。
            // 这样即使收到 \r\n，\r 触发执行，\n 在下一次循环被这里吃掉。
            continue;
        }
        else
        {
            // 正常字符，存入 buffer
            if (dev->line_idx < (AT_LINE_MAX_LEN - 1))
            {
                dev->cmd_buffer[dev->line_idx++] = byte;
            }
            else
            {
                // 单行命令过长
                dev->flag_overwrite = 2; // 标记 Line Overflow
                report_error(dev, AT_ERR_LINE_OVERFLOW);

                // 可选：立即清空 buffer 以便调试观察
                // memset(dev->cmd_buffer, 0, AT_LINE_MAX_LEN);
                // dev->line_idx = 0;

                // 可选：溢出时也可以强制封口打印调试信息，但通常直接丢弃
                // dev->cmd_buffer[AT_LINE_MAX_LEN - 1] = '\0';
            }
        }
    }
}
