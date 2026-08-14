/**
 * @file flc.h
 * @brief A complete Fuzzy Logic Controller (FLC) using a look-up table.
 * @version 1.0
 * @date 2024-08-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef _FLC_CONTROLLER_H_
#define _FLC_CONTROLLER_H_

// Include the look-up table and interpolation library.
#include <ctl/math_block/gmp_math.h>
#include <ctl/component/intrinsic/advance/surf_search.h>

// Include the saturation component
#include <ctl/component/intrinsic/basic/saturation.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup fuzzy_logic_controller Fuzzy Logic Controller
 * @brief A complete FLC implementation based on a 2D look-up table.
 * @details This module integrates a pre-generated fuzzy control surface
 * (from fuzzy_2d.c) with a 2D uniform grid interpolation algorithm
 * (from surf_search.h) to create a complete FLC. It includes scaling
 * factors (gains) for inputs and output, which is essential for tuning
 * the controller's performance in a real-world application.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* External Data Declarations from fuzzy_2d.c                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief Extern declaration for the 21x21 fuzzy control surface matrix.
 * @details This data should be defined in fuzzy_2d.c.
 */
extern ctrl_gt fuzzy_matrix[21][21];

/**
 * @brief Extern declaration for the first dimension's axis points.
 */
extern ctrl_gt fuzzy_matrix_segment1[21];

/**
 * @brief Extern declaration for the second dimension's axis points.
 */
extern ctrl_gt fuzzy_matrix_segment2[21];

/*---------------------------------------------------------------------------*/
/* FLC Controller Structure and Functions                                    */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Fuzzy Logic Controller instance.
 */
typedef struct _tag_flc_controller_t
{
    ctl_uniform_lut2d_t lut; //!< The uniform 2D look-up table object.

    ctl_saturation_t sat_e;  //!< Saturation of error
    ctrl_gt ge;              //!< Gain for the Error input.
    ctl_saturation_t sat_ce; //!< Saturation of Error Rate
    ctrl_gt gce;             //!< Gain for the Error Rate input.
    ctl_saturation_t sat_u;  //!< saturation of output
    ctrl_gt gu;              //!< Gain for the controller Output.
    ctrl_gt out;             //!< output of FLC controller.
} flc_controller_t;

/**
 * @brief Initializes the Fuzzy Logic Controller.
 *
 * @param[out] flc Pointer to the FLC instance to initialize.
 * @param[in] x_min Minimum value of the error (x-axis).
 * @param[in] x_max Maximum value of the error (x-axis).
 * @param[in] x_size Number of points of the error on the x-axis.
 * @param[in] y_min Minimum value of the error rate (y-axis).
 * @param[in] y_max Maximum value of the error rate (y-axis).
 * @param[in] y_size Number of points of the error rate on the y-axis.
 * @param[in] surface Pointer to the 2D array (x_size x y_size), (error, error rate) of surface data.
 * @param[in] gain_err The gain for the 'Error' input.
 * @param[in] gain_err_diff The gain for the 'Error Rate' input.
 * @param[in] gain_output The gain for the final control output.
 *
 * @details This function configures the internal look-up table object with the
 * data from fuzzy_2d.c and sets the scaling factors.
 * It must be called once before using flc_run().
 */
GMP_STATIC_INLINE void ctl_init_flc(flc_controller_t* flc, ctrl_gt x_min, ctrl_gt x_max, uint32_t x_size, ctrl_gt y_min,
                                    ctrl_gt y_max, uint32_t y_size, const ctrl_gt* surface, ctrl_gt gain_err,
                                    ctrl_gt gain_err_diff, ctrl_gt gain_output)
{
    // Store the gains
    flc->ge = gain_err;
    flc->gce = gain_err_diff;
    flc->gu = gain_output;

    // saturation
    ctl_init_saturation(&flc->sat_e, x_min, x_max);
    ctl_init_saturation(&flc->sat_ce, y_min, y_max);
    ctl_init_saturation(&flc->sat_u, (-CTL_CTRL_CONST_1), CTL_CTRL_CONST_1);

    // Initialize the uniform 2D LUT using data from fuzzy_2d.c
    ctl_init_uniform_lut2d(&flc->lut,
                           x_min,    // x_min = -1.0
                           x_max,    // x_max = 1.0
                           x_size,   // x_size
                           y_min,    // y_min = -1.0
                           y_max,    // y_max = 1.0
                           y_size,   // y_size
                           surface); // surface data
}

/**
 * @brief Executes one step of the Fuzzy Logic Controller.
 *
 * @param[in] flc Pointer to the initialized FLC instance.
 * @param[in] error The current system error (e).
 * @param[in] error_rate The current system error rate (ec).
 * @return ctrl_gt The calculated control output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_flc(flc_controller_t* flc, ctrl_gt error, ctrl_gt error_rate)
{
    // 1. Apply input gains to normalize the inputs
    ctrl_gt norm_error = ctl_step_saturation(&flc->sat_e, ctl_mul(error, flc->ge));
    ctrl_gt norm_error_rate = ctl_step_saturation(&flc->sat_ce, ctl_mul(error_rate, flc->gce));

    // 3. Perform interpolation using the look-up table
    ctrl_gt norm_output = ctl_step_interpolate_uniform_lut2d(&flc->lut, norm_error, norm_error_rate);

    // 4. Apply output gain to get the final control value
    flc->out = ctl_step_saturation(&flc->sat_u, ctl_mul(norm_output, flc->gu));

    return flc->out;
}

/**
 * @}
 */ // end of fuzzy_logic_controller group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FLC_CONTROLLER_H_
