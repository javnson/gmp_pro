

#ifndef _FILE_RING_BUF_H_
#define _FILE_RING_BUF_H_

#include <gmp_type.h>

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
 * @brief Initializes a ring buffer over caller-owned storage.
 * @param[out] rb Ring-buffer object to initialize.
 * @param[in,out] pool Statically allocated storage for buffer elements.
 * @param[in] size Total element count; the usable capacity is `size - 1`.
 */
void ringbuf_init(ringbuf_t* rb, data_gt* pool, size_gt size);

/**
 * @brief Returns the number of elements currently stored in the buffer.
 * @param[in] rb Ring-buffer object.
 * @return Number of readable elements.
 */
GMP_STATIC_INLINE size_gt ringbuf_used(const ringbuf_t* rb)
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
 * @brief Returns the number of elements that can be written without overflow.
 * @param[in] rb Ring-buffer object.
 * @return Number of writable elements.
 */
size_gt ringbuf_get_free(const ringbuf_t* rb);

/**
 * @brief Writes one element to the buffer.
 * @param[in,out] rb Ring-buffer object.
 * @param[in] data Element to write.
 * @return `1` on success or `0` when the buffer is full.
 */
fast_gt ringbuf_put_one(ringbuf_t* rb, data_gt data);

/**
 * @brief Reads one element from the buffer.
 * @param[in,out] rb Ring-buffer object.
 * @param[out] data Destination for the element.
 * @return `1` on success or `0` when the buffer is empty.
 */
fast_gt ringbuf_get_one(ringbuf_t* rb, data_gt* data);

/**
 * @brief Writes as many elements as the available space permits.
 * @param[in,out] rb Ring-buffer object.
 * @param[in] data Source array.
 * @param[in] len Requested element count.
 * @return Number of elements written.
 */
size_gt ringbuf_put_array(ringbuf_t* rb, const data_gt* data, size_gt len);

/**
 * @brief Reads up to the requested number of elements.
 * @param[in,out] rb Ring-buffer object.
 * @param[out] dest Destination array.
 * @param[in] len Requested element count.
 * @return Number of elements read.
 */
size_gt ringbuf_get_array(ringbuf_t* rb, data_gt* dest, size_gt len);




#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_RING_BUF_H_
