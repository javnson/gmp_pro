/**
 * @file dev_util.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

/////////////////////////////////////////////////////////////////
// Ring buffer
//
#include <core/dev/ring_buf.h>

/**
 * @brief 1. 初始化环形缓冲区
 * @param rb 缓冲区对象指针
 * @param pool 用户静态分配的内存首地址
 * @param size 内存总长度（注意：实际可用容量为 size - 1）
 */
void ringbuf_init(ringbuf_t* rb, data_gt* pool, size_gt size)
{
    if (!rb || !pool || size < 2)
        return;

    rb->mem_pool = pool;
    rb->capacity = size;
    rb->iget = 0;
    rb->iset = 0;
}

/**
 * @brief 6. 获取当前buffer的可用容量
 * @return 剩余可写入的数量
 */
size_gt ringbuf_get_free(const ringbuf_t* rb)
{
    // 总容量 - 已用 - 1 (保留一个位置用于区分空满)
    return (rb->capacity - 1) - ringbuf_used(rb);
}

/**
 * @brief 2. 写入一个最小单位数据
 * @return 1: 写入成功, 0: 缓冲区已满
 */
fast_gt ringbuf_put_one(ringbuf_t* rb, data_gt data)
{
    size_gt next_iset = (rb->iset + 1);

    // 优化模运算：如果 capacity 是2的幂次，可用 & (cap-1)
    // 这里使用通用取模
    if (next_iset >= rb->capacity)
    {
        next_iset = 0;
    }

    // 检查是否满 (next write == read)
    if (next_iset == rb->iget)
    {
        return 0;
    }

    rb->mem_pool[rb->iset] = data;

    // 内存屏障（可选）：确保数据写入先于索引更新，防止乱序
    // __DMB(); // ARM Cortex-M 指令

    rb->iset = next_iset;
    return 1;
}

/**
 * @brief 3. 读取一个最小单位数据
 * @param data 读出的数据存放指针
 * @return 1: 读取成功, 0: 缓冲区为空
 */
fast_gt ringbuf_get_one(ringbuf_t* rb, data_gt* data)
{
    if (rb->iget == rb->iset)
    {
        return 0; // 空
    }

    *data = rb->mem_pool[rb->iget];

    size_gt next_iget = rb->iget + 1;
    if (next_iget >= rb->capacity)
    {
        next_iget = 0;
    }

    rb->iget = next_iget;
    return 1;
}

/**
 * @brief 4. 写入一串数据
 * @param data 源数据指针
 * @param len 写入长度
 * @return 实际写入的长度 (如果空间不足，可能小于 len，或者是0，取决于策略)
 * 这里策略为：如果空间不够，则尽可能写入填满为止
 */
size_gt ringbuf_put_array(ringbuf_t* rb, const data_gt* data, size_gt len)
{
    size_gt free_space = ringbuf_get_free(rb);
    if (free_space == 0)
        return 0;

    // 限制写入长度为实际可用空间
    if (len > free_space)
    {
        len = free_space;
    }

    size_gt current_iset = rb->iset;
    size_gt items_to_end = rb->capacity - current_iset;

    if (len <= items_to_end)
    {
        // 情况A：不需要回绕，直接拷贝
        memcpy(&rb->mem_pool[current_iset], data, len * sizeof(data_gt));
        rb->iset = (current_iset + len) % rb->capacity; // 只有正好填满到末尾时需要模
        if (rb->iset == rb->capacity)
            rb->iset = 0;
    }
    else
    {
        // 情况B：需要回绕，分两段拷贝
        // 1. 拷贝到缓冲区末尾
        memcpy(&rb->mem_pool[current_iset], data, items_to_end * sizeof(data_gt));
        // 2. 剩余部分拷贝到缓冲区开头
        memcpy(&rb->mem_pool[0], data + items_to_end, (len - items_to_end) * sizeof(data_gt));

        rb->iset = len - items_to_end;
    }

    return len;
}

/**
 * @brief 5. 读出一串数据
 * @param dest 目标buffer指针
 * @param len 期望读取长度
 * @return 实际读取到的长度
 */
size_gt ringbuf_get_array(ringbuf_t* rb, data_gt* dest, size_gt len)
{
    size_gt used_count = ringbuf_used(rb);
    if (used_count == 0)
        return 0;

    // 限制读取长度为实际存在的数据量
    if (len > used_count)
    {
        len = used_count;
    }

    size_gt current_iget = rb->iget;
    size_gt items_to_end = rb->capacity - current_iget;

    if (len <= items_to_end)
    {
        // 情况A：数据连续，未回绕
        memcpy(dest, &rb->mem_pool[current_iget], len * sizeof(data_gt));
        rb->iget = (current_iget + len) % rb->capacity;
        if (rb->iget == rb->capacity)
            rb->iget = 0;
    }
    else
    {
        // 情况B：数据回绕
        // 1. 读出直到末尾的部分
        memcpy(dest, &rb->mem_pool[current_iget], items_to_end * sizeof(data_gt));
        // 2. 读出开头剩余的部分
        memcpy(dest + items_to_end, &rb->mem_pool[0], (len - items_to_end) * sizeof(data_gt));

        rb->iget = len - items_to_end;
    }

    return len;
}

/////////////////////////////////////////////////////////////////
// channel
// 

/**
 * @brief initialize a half duplex channel
 * @param channel half_duplex_ift handle
 * @param buf
 * @param length
 * @param capacity
 */
void gmp_dev_init_half_duplex_channel(half_duplex_ift *channel, data_gt *buf, size_gt length, size_gt capacity)
{
    channel->buf = buf;
    channel->length = length;
    channel->capacity = capacity;
}

/**
 * @brief initialize a duplex channel
 * @param channel duplex_ift handle
 * @param tx_buf transimit buffer
 * @param rx_buf receive buffer
 * @param length length of transmit and receive buffer
 * @param capacity capacity of tx and rx buffer
 */
void gmp_dev_init_duplex_channel(duplex_ift *channel, data_gt *tx_buf, data_gt *rx_buf, size_gt length,
                                 size_gt capacity)
{
    channel->tx_buf = tx_buf;
    channel->rx_buf = rx_buf;
    channel->length = length;
    channel->capacity = capacity;
}

/**
 * @brief initialize a half duplex with address interface
 * @param channel half duplex with address interface handle
 * @param address initialize a address
 * @param msg msg source address
 * @param length length of address
 */
void gmp_dev_init_half_duplex_with_addr_channel(half_duplex_with_addr_ift *channel, addr32_gt address, data_gt *msg,
                                                size_gt length)
{
    channel->address = address;
    channel->msg = msg;
    channel->length = length;
}

/**
 * @brief initialize iic memeory interface object.
 * @param channel handle of IIC memory
 * @param dev_addr IIC device Address
 * @param mem_addr memory address of IIC device
 * @param mem_length mem address length
 * @param msg message object
 * @param length length of message
 */
void gmp_dev_init_iic_memory_channel(iic_memory_ift *channel, addr32_gt dev_addr, addr32_gt mem_addr,
                                     fast_gt mem_length, data_gt *msg, size_gt length)
{
    channel->dev_addr = dev_addr;
    channel->mem_addr = mem_addr;
    channel->mem_length = mem_length;
    channel->msg = msg;
    channel->length = length;
}

/**
 * @brief initialize a can interface object
 * @param channel can interface handle
 * @param id can address
 * @param properties can frame type
 * @param length can data length
 */
void gmp_dev_init_can_channel(can_ift *channel, addr32_gt id, uint32_t properties)
{
    channel->id = id;
    channel->properties = properties;
    channel->length = 0;
    memset(channel->data, 0, 8);
}
