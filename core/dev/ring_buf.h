

#ifndef _FILE_RING_BUF_H_
#define _FILE_RING_BUF_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#ifndef RINGBUF_NULL_RET
#define RINGBUF_NULL_RET ((-1))
#endif // RINGBUF_NULL_RET

//
// main function summary
// + init create a ringbuffer object
// + peek get the current buffer item, -1 if no object in this buffer
// + put  push a item in this buffer, -1 if the array is full, 0 if operation done completely
// + get_spare_size get ring buffer spare size
// + get_valid size get ring buffer valid size
//

// basic memory topology
// 0 | 1 | 2 | 3 | 4 | 5
//     ^ i get: read number here
//                 ^ i set: set number here

typedef struct _tag_ringbuf_t
{
    // buffer pool pointer (statically allocated by user)
    data_gt* mem_pool;

    // length of buffer. Note: Usable size is capacity - 1
    size_gt capacity;

    // the position to read (Head/Consumer)
    volatile size_gt iget;

    // the write position (Tail/Producer)
    volatile size_gt iset;
} ringbuf_t;

/**
 * @brief 1. 初始化环形缓冲区
 * @param rb 缓冲区对象指针
 * @param pool 用户静态分配的内存首地址
 * @param size 内存总长度（注意：实际可用容量为 size - 1）
 */
void ringbuf_init(ringbuf_t* rb, data_gt* pool, size_gt size);

/**
 * @brief 计算当前已使用的数据量 (Helper)
 */
GMP_STATIC_INLINE 
size_gt ringbuf_used(const ringbuf_t* rb)
{
    size_gt w = rb->iset;
    size_gt r = rb->iget;

    if (w >= r)
    {
        return w - r;
    }
    else
    {
        return rb->capacity - (r - w);
    }
}

/**
 * @brief Get current buffer valid capacity
 * @return 剩余可写入的数量
 */
size_gt ringbuf_get_free(const ringbuf_t* rb);

/**
 * @brief 2. 写入一个最小单位数据
 * @return 1: 写入成功, 0: 缓冲区已满
 */
fast_gt ringbuf_put_one(ringbuf_t* rb, data_gt data);

/**
 * @brief 3. 读取一个最小单位数据
 * @param data 读出的数据存放指针
 * @return true: 读取成功, false: 缓冲区为空
 */
fast_gt ringbuf_get_one(ringbuf_t* rb, data_gt* data);

/**
 * @brief 4. 写入一串数据
 * @param data 源数据指针
 * @param len 写入长度
 * @return 实际写入的长度 (如果空间不足，可能小于 len，或者是0，取决于策略)
 * 这里策略为：如果空间不够，则尽可能写入填满为止
 */
size_gt ringbuf_put_array(ringbuf_t* rb, const data_gt* data, size_gt len);

/**
 * @brief 5. 读出一串数据
 * @param dest 目标buffer指针
 * @param len 期望读取长度
 * @return 实际读取到的长度
 */
size_gt ringbuf_get_array(ringbuf_t* rb, data_gt* dest, size_gt len);




#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_RING_BUF_H_
