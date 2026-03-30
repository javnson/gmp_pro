/**
 * @file ctl_mem_view.h
 * @defgroup mem_view Memory View Manager
 * @brief High-performance memory mapping module for 1D, 2D (SoA), and 3D arrays.
 * @details Abstracts a flat contiguous memory pool into safe multi-dimensional views.
 * Extremely useful for data logging, lookup tables, and matrix operations.
 * @{
 */

#ifndef _FILE_CTL_MEM_VIEW_H_
#define _FILE_CTL_MEM_VIEW_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Data structure for the memory view manager.
 */
typedef struct _tag_ctl_mem_view
{
    ctrl_gt* buffer;   /*!< Pointer to the 1D continuous memory block. */
    uint32_t capacity; /*!< Total maximum capacity of the buffer (number of elements). */
} ctl_mem_view_t;

/**
 * @brief Initializes the memory view manager.
 * @param[out] obj      Pointer to the memory view instance.
 * @param[in]  mem_pool Pointer to the pre-allocated memory array.
 * @param[in]  capacity Total number of elements the pool can hold.
 */
GMP_STATIC_INLINE void ctl_init_mem_view(ctl_mem_view_t* obj, ctrl_gt* mem_pool, uint32_t capacity)
{
    obj->buffer = mem_pool;
    obj->capacity = capacity;
}

// ============================================================================
// 1D Access
// ============================================================================

/**
 * @brief 1D Access: Sets a value at the specified absolute index.
 */
GMP_STATIC_INLINE void ctl_mem_set_1d(ctl_mem_view_t* obj, uint32_t idx, ctrl_gt value)
{
    if (idx < obj->capacity)
    {
        obj->buffer[idx] = value;
    }
}

/**
 * @brief 1D Access: Gets a value at the specified absolute index.
 */
GMP_STATIC_INLINE ctrl_gt ctl_mem_get_1d(ctl_mem_view_t* obj, uint32_t idx)
{
    if (idx < obj->capacity)
    {
        return obj->buffer[idx];
    }
    return float2ctrl(0.0f);
}

// ============================================================================
// 2D Access (Structure of Arrays)
// ============================================================================

/**
 * @brief 2D Access: Sets a value using Structure of Arrays (SoA) layout.
 * @details Address = (dim * depth) + idx.
 */
GMP_STATIC_INLINE void ctl_mem_set_2d_soa(ctl_mem_view_t* obj, uint16_t dim, uint32_t idx, uint32_t depth,
                                          ctrl_gt value)
{
    uint32_t pos = (dim * depth) + idx;
    if (pos < obj->capacity)
    {
        obj->buffer[pos] = value;
    }
}

/**
 * @brief 2D Access: Gets a value using Structure of Arrays (SoA) layout.
 */
GMP_STATIC_INLINE ctrl_gt ctl_mem_get_2d_soa(ctl_mem_view_t* obj, uint16_t dim, uint32_t idx, uint32_t depth)
{
    uint32_t pos = (dim * depth) + idx;
    if (pos < obj->capacity)
    {
        return obj->buffer[pos];
    }
    return float2ctrl(0.0f);
}

// ============================================================================
// 3D Access (Structure of Arrays of Arrays)
// ============================================================================

/**
 * @brief 3D Access: Sets a value in a 3D memory map.
 * @details Address = (dim1 * max_dim2 * depth) + (dim2 * depth) + idx.
 */
GMP_STATIC_INLINE void ctl_mem_set_3d_soa(ctl_mem_view_t* obj, uint16_t dim1, uint16_t max_dim2, uint16_t dim2,
                                          uint32_t idx, uint32_t depth, ctrl_gt value)
{
    uint32_t pos = (dim1 * max_dim2 * depth) + (dim2 * depth) + idx;
    if (pos < obj->capacity)
    {
        obj->buffer[pos] = value;
    }
}

/**
 * @brief 3D Access: Gets a value in a 3D memory map.
 */
GMP_STATIC_INLINE ctrl_gt ctl_mem_get_3d_soa(ctl_mem_view_t* obj, uint16_t dim1, uint16_t max_dim2, uint16_t dim2,
                                             uint32_t idx, uint32_t depth)
{
    uint32_t pos = (dim1 * max_dim2 * depth) + (dim2 * depth) + idx;
    if (pos < obj->capacity)
    {
        return obj->buffer[pos];
    }
    return float2ctrl(0.0f);
}

#ifdef __cplusplus
}
#endif

#endif // _FILE_CTL_MEM_VIEW_H_

/** @} */
