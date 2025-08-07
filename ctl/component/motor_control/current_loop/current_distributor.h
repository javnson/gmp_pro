/**
 * @file current_distributor.h
 * @brief Implements a current distributor using a look-up table (LUT).
 * @details This module takes a total current magnitude command (Im) and distributes
 * it into d-axis (Id) and q-axis (Iq) components. The distribution is
 * determined by a current angle (alpha), which can either be a fixed constant
 * or interpolated from a 1D look-up table using the functions provided by
 * the surf_search module. This is typically used to implement strategies like
 * MTPA (Maximum Torque Per Ampere) where the optimal current angle varies
 * with the total current.
 *
 * @version 0.2
 * @date 2025-08-06
 */

#ifndef _FILE_CURRENT_DISTRIBUTOR_H_
#define _FILE_CURRENT_DISTRIBUTOR_H_

#include <ctl/component/intrinsic/advance/surf_search.h> // Dependency for LUT search and interpolation

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Look-up Table based Current Distributor                                   */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CURRENT_DISTRIBUTOR LUT Current Distributor
 * @brief Distributes a total current command into Id and Iq components via a LUT.
 * @{
 */

//================================================================================
// Type Defines & Enums
//================================================================================

#ifndef GMP_STATIC_INLINE
#define GMP_STATIC_INLINE static inline
#endif

// Define the standard control data type if not already defined
#ifndef CTRL_GT_DEFINED
#define CTRL_GT_DEFINED
typedef float ctrl_gt;
typedef float parameter_gt;
#endif

/**
 * @brief Defines the operating modes for the current distributor.
 */
typedef enum
{
    DIST_MODE_CONST_ALPHA, ///< Use a fixed, constant current angle (alpha).
    DIST_MODE_LUT_LINEAR   ///< Use a 1D Look-Up Table to find alpha based on current magnitude.
} ctl_dist_mode_t;

/**
 * @brief Main structure for the LUT-based current distributor.
 */
typedef struct
{
    // --- Outputs ---
    ctrl_gt id_ref; ///< The calculated d-axis current reference.
    ctrl_gt iq_ref; ///< The calculated q-axis current reference.

    // --- Configuration & State ---
    ctl_dist_mode_t mode; ///< The selected distribution mode.
    ctrl_gt const_alpha;  ///< The fixed angle used in CONST_ALPHA mode (in radians).

    // --- LUT Data (for LUT_LINEAR mode) ---
    ctl_lut1d_t im_axis_lut;     ///< The LUT structure for the current magnitude (Im) axis.
    const ctrl_gt* alpha_values; ///< Pointer to the array of corresponding alpha values (in radians).

} ctl_current_distributor_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the current distributor.
 * @details Sets up the distributor's mode and configures the look-up table data
 * if LUT mode is selected.
 * @param[out] dist Pointer to the current distributor structure.
 * @param[in]  mode The desired operating mode (CONST_ALPHA or LUT_LINEAR).
 * @param[in]  im_axis Pointer to the array of current magnitude (Im) values for the LUT's x-axis.
 * @param[in]  alpha_values Pointer to the array of corresponding current angle (alpha) values for the LUT's y-axis.
 * @param[in]  lut_size The number of elements in the LUT arrays.
 * @param[in]  const_alpha_rad The fixed angle (in radians) to be used in CONST_ALPHA mode.
 */
GMP_STATIC_INLINE void ctl_init_current_distributor(ctl_current_distributor_t* dist, ctl_dist_mode_t mode,
                                                    const ctrl_gt* im_axis, const ctrl_gt* alpha_values,
                                                    uint32_t lut_size, ctrl_gt const_alpha_rad)
{
    dist->mode = mode;
    dist->const_alpha = const_alpha_rad;
    dist->id_ref = 0.0f;
    dist->iq_ref = 0.0f;

    if (mode == DIST_MODE_LUT_LINEAR && im_axis != NULL && alpha_values != NULL && lut_size > 1)
    {
        // Initialize the 1D LUT structure from surf_search.h
        ctl_init_lut1d(&dist->im_axis_lut, im_axis, lut_size);
        dist->alpha_values = alpha_values;
    }
    else
    {
        // If not using LUT mode or data is invalid, default to constant alpha
        dist->mode = DIST_MODE_CONST_ALPHA;
        dist->alpha_values = NULL;
    }
}

/**
 * @brief Executes one step of the current distribution calculation.
 * @param[out] dist Pointer to the current distributor structure.
 * @param[in]  im_cmd The signed, total torque-producing current command from the speed loop.
 */
GMP_STATIC_INLINE void ctl_step_current_distributor(ctl_current_distributor_t* dist, ctrl_gt im_cmd)
{
    ctrl_gt alpha;
    ctrl_gt im_mag = fabsf(im_cmd);

    // 1. Determine the current angle 'alpha' based on the selected mode.
    if (dist->mode == DIST_MODE_LUT_LINEAR)
    {
        // Use the generic 1D interpolation function from surf_search.h
        alpha = ctl_interpolate_lut1d(&dist->im_axis_lut, dist->alpha_values, im_mag);
    }
    else // DIST_MODE_CONST_ALPHA
    {
        alpha = dist->const_alpha;
    }

    // 2. Calculate Id and Iq based on the total current magnitude and the angle alpha.
    ctrl_gt id_mag = im_mag * cosf(alpha);
    ctrl_gt iq_mag = im_mag * sinf(alpha);

    // 3. Apply the original sign of the command to the torque-producing component (Iq).
    // The sign of Id is determined by the LUT (typically negative for MTPA).
    dist->id_ref = id_mag;
    dist->iq_ref = (im_cmd >= 0.0f) ? iq_mag : -iq_mag;
}

/**
 * @brief Gets the calculated d-axis current reference.
 * @param[in] dist Pointer to the current distributor structure.
 * @return The calculated d-axis current reference.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_distributor_id_ref(const ctl_current_distributor_t* dist)
{
    return dist->id_ref;
}

/**
 * @brief Gets the calculated q-axis current reference.
 * @param[in] dist Pointer to the current distributor structure.
 * @return The calculated q-axis current reference.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_distributor_iq_ref(const ctl_current_distributor_t* dist)
{
    return dist->iq_ref;
}

/** @} */ // end of CURRENT_DISTRIBUTOR group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CURRENT_DISTRIBUTOR_H_
