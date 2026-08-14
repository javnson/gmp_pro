/**
 * @file ring_buffer.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements a caller-owned, single-ring data buffer.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_type.h>

/////////////////////////////////////////////////////////////////
// Ring buffer
//
#include <core/base/ds/ring_buf.h>

/**
 * @brief Initializes a ring buffer over caller-owned storage.
 * @param[out] rb Ring-buffer object to initialize.
 * @param[in,out] pool Statically allocated storage for buffer elements.
 * @param[in] size Total element count; the usable capacity is `size - 1`.
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
 * @brief Returns the number of writable elements.
 * @param[in] rb Ring-buffer object.
 * @return Number of elements that can be written without overflow.
 */
size_gt ringbuf_get_free(const ringbuf_t* rb)
{
    // Reserve one element to distinguish the full and empty states.
    return (rb->capacity - 1) - ringbuf_used(rb);
}

/**
 * @brief Writes one element to the buffer.
 * @param[in,out] rb Ring-buffer object.
 * @param[in] data Element to write.
 * @return `1` on success or `0` when the buffer is full.
 */
fast_gt ringbuf_put_one(ringbuf_t* rb, data_gt data)
{
    size_gt next_iset = (rb->iset + 1);

    // Use an explicit wrap so capacity does not need to be a power of two.
    if (next_iset >= rb->capacity)
    {
        next_iset = 0;
    }

    // The buffer is full when the next write position reaches the reader.
    if (next_iset == rb->iget)
    {
        return 0;
    }

    rb->mem_pool[rb->iset] = data;

    // A platform may add a memory barrier here for cross-context sharing.

    rb->iset = next_iset;
    return 1;
}

/**
 * @brief Reads one element from the buffer.
 * @param[in,out] rb Ring-buffer object.
 * @param[out] data Destination for the element.
 * @return `1` on success or `0` when the buffer is empty.
 */
fast_gt ringbuf_get_one(ringbuf_t* rb, data_gt* data)
{
    if (rb->iget == rb->iset)
    {
        return 0;
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
 * @brief Writes as many elements as the available space permits.
 * @param[in,out] rb Ring-buffer object.
 * @param[in] data Source array.
 * @param[in] len Requested element count.
 * @return Number of elements written.
 */
size_gt ringbuf_put_array(ringbuf_t* rb, const data_gt* data, size_gt len)
{
    size_gt free_space = ringbuf_get_free(rb);
    if (free_space == 0)
        return 0;

    // Limit the transfer to the available space.
    if (len > free_space)
    {
        len = free_space;
    }

    size_gt current_iset = rb->iset;
    size_gt items_to_end = rb->capacity - current_iset;

    if (len <= items_to_end)
    {
        // The requested range is contiguous.
        memcpy(&rb->mem_pool[current_iset], data, len * sizeof(data_gt));
        rb->iset = (current_iset + len) % rb->capacity;
        if (rb->iset == rb->capacity)
            rb->iset = 0;
    }
    else
    {
        // Split a wrapped transfer at the end of the backing array.
        memcpy(&rb->mem_pool[current_iset], data, items_to_end * sizeof(data_gt));
        memcpy(&rb->mem_pool[0], data + items_to_end, (len - items_to_end) * sizeof(data_gt));

        rb->iset = len - items_to_end;
    }

    return len;
}

/**
 * @brief Reads up to the requested number of elements.
 * @param[in,out] rb Ring-buffer object.
 * @param[out] dest Destination array.
 * @param[in] len Requested element count.
 * @return Number of elements read.
 */
size_gt ringbuf_get_array(ringbuf_t* rb, data_gt* dest, size_gt len)
{
    size_gt used_count = ringbuf_used(rb);
    if (used_count == 0)
        return 0;

    // Limit the transfer to the number of readable elements.
    if (len > used_count)
    {
        len = used_count;
    }

    size_gt current_iget = rb->iget;
    size_gt items_to_end = rb->capacity - current_iget;

    if (len <= items_to_end)
    {
        // The requested range is contiguous.
        memcpy(dest, &rb->mem_pool[current_iget], len * sizeof(data_gt));
        rb->iget = (current_iget + len) % rb->capacity;
        if (rb->iget == rb->capacity)
            rb->iget = 0;
    }
    else
    {
        // Split a wrapped transfer at the end of the backing array.
        memcpy(dest, &rb->mem_pool[current_iget], items_to_end * sizeof(data_gt));
        memcpy(dest + items_to_end, &rb->mem_pool[0], (len - items_to_end) * sizeof(data_gt));

        rb->iget = len - items_to_end;
    }

    return len;
}
