/**
 * @file paired_lut1d.h
 * @brief Provides a 1D look-up table (LUT) functionality for paired {x, y} data structures.
 *
 * @version 0.1
 * @date 2024-10-25
 *
 */

#ifndef _FILE_PAIRED_LUT1D_H_
#define _FILE_PAIRED_LUT1D_H_


#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup PAIRED_LUT1D Paired 1D Look-Up Table
 * @brief A library for 1D data searching and interpolation using array of structs {x, y}.
 * @details This module implements algorithms for searching and interpolating values
 * from pre-defined tables where data is organized as pairs of {input, output}.
 * The x-axis content (input) must be a strictly monotonically increasing sequence.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Paired 1D Look-Up Table (LUT)                                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief Standard paired data point structure {x, y}.
 * @details Memory layout is identical to custom structs like {im, alpha}, 
 * allowing safe type-casting from user-defined calibration tables.
 */
typedef struct _tag_lut1d_pair
{
    ctrl_gt x; //!< Input value (e.g., current magnitude Im).
    ctrl_gt y; //!< Output value (e.g., advance angle alpha).
} ctl_lut1d_pair_t;

/**
 * @brief Data structure for a Paired 1D Look-Up Table.
 */
typedef struct _tag_paired_lut1d
{
    uint32_t size;                 //!< The number of {x, y} pairs in the table.
    const ctl_lut1d_pair_t* table; //!< Pointer to the array of paired data.
} ctl_paired_lut1d_t;

/**
 * @brief Initializes a Paired 1D Look-Up Table.
 * @param[out] lut   Pointer to the Paired 1D LUT instance.
 * @param[in]  table Pointer to the array of {x, y} structured data.
 * @param[in]  size  The number of elements in the table array.
 */
void ctl_init_paired_lut1d(ctl_paired_lut1d_t* lut, const ctl_lut1d_pair_t* table, uint32_t size);

/**
 * @brief Performs a binary search to find the index of the lower bound for a target value.
 * @param[in] lut    Pointer to the Paired 1D LUT instance.
 * @param[in] target The input value (x) to search for.
 * @return int32_t The index of the grid point immediately to the left of (or equal to) the target.
 * Returns -1 if the target is below the first element.
 * Returns (size-2) if the target is at or above the last element.
 */
GMP_STATIC_INLINE int32_t ctl_search_paired_lut1d_index(const ctl_paired_lut1d_t* lut, ctrl_gt target)
{
    // Handle out-of-bounds cases
    if (target < lut->table[0].x)
    {
        return -1;
    }
    if (target >= lut->table[lut->size - 1].x)
    {
        return lut->size - 2;
    }

    // Binary search
    uint32_t left = 0;
    uint32_t right = lut->size - 1;
    while (left <= right)
    {
        uint32_t mid = left + (right - left) / 2;
        if (lut->table[mid].x > target)
        {
            right = mid - 1;
        }
        else
        {
            left = mid + 1;
        }
    }
    return right;
}

/**
 * @brief Performs 1D linear interpolation on paired data.
 * @param[in] lut    Pointer to the Paired 1D LUT instance.
 * @param[in] target The input x-value at which to interpolate.
 * @return ctrl_gt The interpolated y-value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_interpolate_paired_lut1d(const ctl_paired_lut1d_t* lut, ctrl_gt target)
{
    int32_t index = ctl_search_paired_lut1d_index(lut, target);

    // Handle boundary conditions (Clamp to extremes)
    if (index < 0)
    {
        return lut->table[0].y; // Target is below the range
    }
    if ((uint32_t)index >= lut->size - 1)
    {
        return lut->table[lut->size - 1].y; // Target is above the range
    }

    // Linear interpolation
    ctrl_gt x0 = lut->table[index].x;
    ctrl_gt y0 = lut->table[index].y;
    ctrl_gt x1 = lut->table[index + 1].x;
    ctrl_gt y1 = lut->table[index + 1].y;

    // Weight calculation: (target - x0) / (x1 - x0)
    ctrl_gt weight = ctl_div(target - x0, x1 - x0);

    // Result: y0 + weight * (y1 - y0)
    return y0 + ctl_mul(weight, (y1 - y0));
}

/**
 * @}
 */ // end of PAIRED_LUT1D group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PAIRED_LUT1D_H_
