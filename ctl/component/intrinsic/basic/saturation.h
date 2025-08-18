/**
 * @file saturation.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides standard and bipolar saturation (limiter) blocks.
 * @version 1,05
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */
#ifndef _SATURATION_H_
#define _SATURATION_H_

#include <ctl/math_block/gmp_math.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup saturation_blocks Saturation (Limiter) Blocks
 * @brief A collection of signal limiting modules.
 * @details This module implements mathematical blocks for limiting signals.
 * It includes a standard saturation block that clamps a signal between an
 * upper and lower limit, and a bipolar saturation block that applies
 * different dead-zones or limits for positive and negative values.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Standard Saturation                                                       */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a standard saturation block.
 */
typedef struct _tag_saturation_t
{
    ctrl_gt out;     //!< The last calculated output.
    ctrl_gt out_min; //!< The lower limit of the output.
    ctrl_gt out_max; //!< The upper limit of the output.
} ctl_saturation_t;

/**
 * @brief Initializes the standard saturation block.
 * @param[out] obj Pointer to the saturation instance.
 * @param[in] out_min The lower saturation limit.
 * @param[in] out_max The upper saturation limit.
 */
void ctl_init_saturation(ctl_saturation_t* obj, ctrl_gt out_min, ctrl_gt out_max);

/**
 * @brief Executes one step of the saturation block.
 * @details Clamps the input value to be within [out_min, out_max].
 * @param[in,out] obj Pointer to the saturation instance.
 * @param[in] input The signal to be limited.
 * @return ctrl_gt The saturated output value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_saturation(ctl_saturation_t* obj, ctrl_gt input)
{
    obj->out = ctl_sat(input, obj->out_max, obj->out_min);
    return obj->out;
}

/**
 * @brief Sets new limits for the saturation block.
 * @param[out] obj Pointer to the saturation instance.
 * @param[in] out_min The new lower limit.
 * @param[in] out_max The new upper limit.
 */
GMP_STATIC_INLINE void ctl_set_saturation_limits(ctl_saturation_t* obj, ctrl_gt out_min, ctrl_gt out_max)
{
    obj->out_min = out_min;
    obj->out_max = out_max;
}

/*---------------------------------------------------------------------------*/
/* Bipolar Saturation                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a bipolar saturation block.
 * @details This block applies different saturation bands for positive and negative inputs.
 * For positive inputs, the output is clamped to [out_min, out_max].
 * For negative inputs, the output is clamped to [-out_max, -out_min].
 * This is useful for creating a dead-zone around zero if out_min > 0.
 */
typedef struct _tag_bipolar_saturation_t
{
    ctrl_gt out;     //!< The last calculated output.
    ctrl_gt out_min; //!< The inner limit (magnitude). Should be positive.
    ctrl_gt out_max; //!< The outer limit (magnitude). Should be positive.
} ctl_bipolar_saturation_t;

/**
 * @brief Initializes the bipolar saturation block.
 * @param[out] obj Pointer to the bipolar saturation instance.
 * @param[in] out_min The inner limit (e.g., dead-zone edge). Must be >= 0.
 * @param[in] out_max The outer limit. Must be >= out_min.
 */
void ctl_init_bipolar_saturation(ctl_bipolar_saturation_t* obj, ctrl_gt out_min, ctrl_gt out_max);

/**
 * @brief Executes one step of the bipolar saturation block.
 * @param[in,out] obj Pointer to the bipolar saturation instance.
 * @param[in] input The signal to be limited.
 * @return ctrl_gt The saturated output value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_bipolar_saturation(ctl_bipolar_saturation_t* obj, ctrl_gt input)
{
    if (input > 0)
    {
        obj->out = ctl_sat(input, obj->out_max, obj->out_min);
    }
    else
    {
        obj->out = ctl_sat(input, -obj->out_min, -obj->out_max);
    }
    return obj->out;
}

/*---------------------------------------------------------------------------*/
/* atan soft Saturation                                                      */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a atan soft saturation block.
 * @details This block defined a soft saturation block.
 * @f[ out = gain\times tan^{-1} (sf \times x) @f]
 * output will clamp to [-gain, gain].
 */
typedef struct _tag_atan_saturation_t
{
    ctrl_gt out;  //!< The last calculated output.
    ctrl_gt gain; //!< The output gain, output limit (magnitude). Should be positive.
    ctrl_gt sf;   //!< The input scale factor
} ctl_atan_saturation_t;

/**
 * @brief initialize a atan soft saturation module.
 * @param[out] sat handle of saturation
 * @param[in] gain gain of saturatioin output.
 * @param[in] scale_factor scale factor of input
 */
void ctl_init_tanh_saturation(ctl_atan_saturation_t* sat, ctrl_gt gain, ctrl_gt scale_factor);

/**
 * @brief step the soft atan saturation module.
 * @param[in] sat handle of saturation.
 * @param[in] input input to saturation.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_tanh_saturation(ctl_atan_saturation_t* sat, ctrl_gt input)
{
    sat->out = ctl_mul(sat->gain, ctl_atan2(ctl_mul(input, sat->sf), CTL_CTRL_CONST_1));
    return sat->out;
}

/**
 * @}
 */ // end of saturation_blocks group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _SATURATION_H_
