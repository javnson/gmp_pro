/**
 * @file lookup_table.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides 1D and 2D look-up table (LUT) functionalities.
 * @version 0.2
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file implements algorithms for searching and interpolating values
 * from pre-defined tables. It includes a 1D LUT with binary search and linear
 * interpolation, a 2D LUT for non-uniform grids, and a 2D LUT for uniform grids,
 * both using bilinear interpolation. These are essential for efficiently
 * implementing complex, nonlinear functions in embedded systems.
 */

#ifndef _LOOKUP_TABLE_H_
#define _LOOKUP_TABLE_H_

#include <ctl/math_block/gmp_math.h>
#include <stdint.h> // For int32_t

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup lookup_tables Look-Up Tables (LUT)
 * @brief A library for 1D and 2D data searching and interpolation.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* 1D Look-Up Table (LUT)                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a 1D Look-Up Table.
 * @details The axis content must be a strictly monotonically increasing sequence.
 */
typedef struct _tag_lut1d_t
{
    uint32_t size;       //!< The number of elements in the LUT axis.
    const ctrl_gt* axis; //!< Pointer to the array of axis values.
} ctl_lut1d_t;

/**
 * @brief Initializes a 1D Look-Up Table.
 * @param[out] lut Pointer to the 1D LUT instance.
 * @param[in] axis Pointer to the array of axis data.
 * @param[in] size The number of elements in the axis array.
 */
void ctl_init_lut1d(ctl_lut1d_t* lut, const ctrl_gt* axis, uint32_t size);

/**
 * @brief Performs a binary search to find the index of the lower bound for a target value.
 * @param[in] lut Pointer to the 1D LUT instance.
 * @param[in] target The value to search for in the axis.
 * @return int32_t The index of the grid point immediately to the left of (or equal to) the target.
 * Returns -1 if the target is below the first element.
 * Returns (size-2) if the target is at or above the last element.
 */
GMP_STATIC_INLINE int32_t ctl_search_lut1d_index(const ctl_lut1d_t* lut, ctrl_gt target)
{
    // Handle out-of-bounds cases
    if (target < lut->axis[0])
    {
        return -1;
    }
    if (target >= lut->axis[lut->size - 1])
    {
        return lut->size - 2;
    }

    // Binary search
    uint32_t left = 0;
    uint32_t right = lut->size - 1;
    while (left <= right)
    {
        uint32_t mid = left + (right - left) / 2;
        if (lut->axis[mid] > target)
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
 * @brief Performs 1D linear interpolation.
 * @param[in] lut Pointer to the 1D LUT instance containing the x-axis data.
 * @param[in] values Pointer to the array of y-axis values corresponding to the x-axis.
 * @param[in] target The x-value at which to interpolate.
 * @return ctrl_gt The interpolated y-value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_interpolate_lut1d(const ctl_lut1d_t* lut, const ctrl_gt* values, ctrl_gt target)
{
    int32_t index = ctl_search_lut1d_index(lut, target);

    // Handle boundary conditions
    if (index < 0)
    {
        return values[0]; // Target is below the range
    }
    if ((uint32_t)index >= lut->size - 1)
    {
        return values[lut->size - 1]; // Target is above the range
    }

    // Linear interpolation
    ctrl_gt x0 = lut->axis[index];
    ctrl_gt y0 = values[index];
    ctrl_gt x1 = lut->axis[index + 1];
    ctrl_gt y1 = values[index + 1];

    ctrl_gt weight = ctl_div(target - x0, x1 - x0);
    return y0 + ctl_mul(weight, (y1 - y0));
}

/*---------------------------------------------------------------------------*/
/* 2D Look-Up Table (Non-Uniform Grid)                                       */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a 2D LUT with non-uniformly spaced axes.
 */
typedef struct _tag_lut2d_t
{
    ctl_lut1d_t dim1_axis;   //!< 1D LUT for the first dimension (x-axis).
    ctl_lut1d_t dim2_axis;   //!< 1D LUT for the second dimension (y-axis).
    const ctrl_gt** surface; //!< Pointer to a 2D array of surface values.
} ctl_lut2d_t;

/**
 * @brief Initializes a 2D Look-Up Table.
 * @param[out] lut Pointer to the 2D LUT instance.
 * @param[in] axis1 Pointer to the array for the 1st axis.
 * @param[in] size1 Number of elements in the 1st axis.
 * @param[in] axis2 Pointer to the array for the 2nd axis.
 * @param[in] size2 Number of elements in the 2nd axis.
 * @param[in] surface Pointer to the 2D array (size1 x size2) of surface data.
 */
void ctl_init_lut2d(ctl_lut2d_t* lut, const ctrl_gt* axis1, uint32_t size1, const ctrl_gt* axis2, uint32_t size2,
                    const ctrl_gt** surface);

/**
 * @brief Performs 2D bilinear interpolation on a non-uniform grid.
 * @param[in] lut Pointer to the 2D LUT instance.
 * @param[in] target1 The target value on the 1st axis.
 * @param[in] target2 The target value on the 2nd axis.
 * @return ctrl_gt The interpolated surface value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_interpolate_lut2d(const ctl_lut2d_t* lut, ctrl_gt target1, ctrl_gt target2)
{
    // Find lower-bound indices for both axes
    int32_t idx1 = ctl_search_lut1d_index(&lut->dim1_axis, target1);
    int32_t idx2 = ctl_search_lut1d_index(&lut->dim2_axis, target2);

    // Clamp indices to valid range
    idx1 = (idx1 < 0) ? 0 : idx1;
    idx2 = (idx2 < 0) ? 0 : idx2;
    idx1 = (idx1 > (int32_t)lut->dim1_axis.size - 2) ? (lut->dim1_axis.size - 2) : idx1;
    idx2 = (idx2 > (int32_t)lut->dim2_axis.size - 2) ? (lut->dim2_axis.size - 2) : idx2;

    // Get corner points of the interpolation cell
    ctrl_gt p00 = lut->surface[idx1][idx2];
    ctrl_gt p10 = lut->surface[idx1 + 1][idx2];
    ctrl_gt p01 = lut->surface[idx1][idx2 + 1];
    ctrl_gt p11 = lut->surface[idx1 + 1][idx2 + 1];

    // Calculate interpolation weights
    ctrl_gt w1 =
        ctl_div(target1 - lut->dim1_axis.axis[idx1], lut->dim1_axis.axis[idx1 + 1] - lut->dim1_axis.axis[idx1]);
    ctrl_gt w2 =
        ctl_div(target2 - lut->dim2_axis.axis[idx2], lut->dim2_axis.axis[idx2 + 1] - lut->dim2_axis.axis[idx2]);

    // Bilinear interpolation
    ctrl_gt r1 = p00 + ctl_mul(w1, p10 - p00);
    ctrl_gt r2 = p01 + ctl_mul(w1, p11 - p01);
    return r1 + ctl_mul(w2, r2 - r1);
}

/*---------------------------------------------------------------------------*/
/* 2D Look-Up Table (Uniform Grid)                                           */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a 2D LUT with uniformly spaced axes.
 */
typedef struct _tag_uniform_lut2d_t
{
    ctrl_gt x_min;      //!< Minimum value of the x-axis.
    ctrl_gt x_step_inv; //!< Inverse of the x-axis step size (1 / delta_x).
    uint32_t x_size;    //!< Number of points on the x-axis.

    ctrl_gt y_min;      //!< Minimum value of the y-axis.
    ctrl_gt y_step_inv; //!< Inverse of the y-axis step size (1 / delta_y).
    uint32_t y_size;    //!< Number of points on the y-axis.

    const ctrl_gt** surface; //!< Pointer to a 2D array of surface values.
} ctl_uniform_lut2d_t;

/**
 * @brief Initializes a 2D LUT with a uniform grid.
 * @param[out] lut Pointer to the uniform 2D LUT instance.
 * @param[in] x_min Minimum value of the x-axis.
 * @param[in] x_max Maximum value of the x-axis.
 * @param[in] x_size Number of points on the x-axis.
 * @param[in] y_min Minimum value of the y-axis.
 * @param[in] y_max Maximum value of the y-axis.
 * @param[in] y_size Number of points on the y-axis.
 * @param[in] surface Pointer to the 2D array (x_size x y_size) of surface data.
 */
void ctl_init_uniform_lut2d(ctl_uniform_lut2d_t* lut, ctrl_gt x_min, ctrl_gt x_max, uint32_t x_size, ctrl_gt y_min,
                            ctrl_gt y_max, uint32_t y_size, const ctrl_gt** surface);

/**
 * @brief Performs 2D bilinear interpolation on a uniform grid.
 * @param[in] lut Pointer to the uniform 2D LUT instance.
 * @param[in] x The target value on the x-axis.
 * @param[in] y The target value on the y-axis.
 * @return ctrl_gt The interpolated surface value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_interpolate_uniform_lut2d(const ctl_uniform_lut2d_t* lut, ctrl_gt x, ctrl_gt y)
{
    // Calculate floating point indices
    ctrl_gt x_fidx = ctl_mul(x - lut->x_min, lut->x_step_inv);
    ctrl_gt y_fidx = ctl_mul(y - lut->y_min, lut->y_step_inv);

    // Clamp indices
    x_fidx = ctl_sat(x_fidx, (ctrl_gt)(lut->x_size - 1), 0);
    y_fidx = ctl_sat(y_fidx, (ctrl_gt)(lut->y_size - 1), 0);

    // Get integer indices (lower bound)
    uint32_t x_idx = (uint32_t)x_fidx;
    uint32_t y_idx = (uint32_t)y_fidx;

    // Get interpolation weights (fractional part)
    ctrl_gt wx = x_fidx - (ctrl_gt)x_idx;
    ctrl_gt wy = y_fidx - (ctrl_gt)y_idx;

    // Get corner points
    ctrl_gt p00 = lut->surface[x_idx][y_idx];
    ctrl_gt p10 = lut->surface[x_idx + 1][y_idx];
    ctrl_gt p01 = lut->surface[x_idx][y_idx + 1];
    ctrl_gt p11 = lut->surface[x_idx + 1][y_idx + 1];

    // Bilinear interpolation
    ctrl_gt r1 = p00 + ctl_mul(wx, p10 - p00);
    ctrl_gt r2 = p01 + ctl_mul(wx, p11 - p01);
    return r1 + ctl_mul(wy, r2 - r1);
}

/**
 * @}
 */ // end of lookup_tables group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _LOOKUP_TABLE_H_
