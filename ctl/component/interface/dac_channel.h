/**
 * @file dac_channel.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a module for processing and scaling a single DAC channel.
 * @details This module takes a control value, applies gain and bias, and scales
 * it to the appropriate raw integer format for a DAC peripheral.
 * It supports both fixed-point and floating-point data types.
 * @version 0.2
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_DAC_CHANNEL_H_
#define _FILE_DAC_CHANNEL_H_

// #include <gmp_core.h>
#include <ctl/component/interface/interface_base.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup DAC_CHANNEL Digital-to-Analog Converter (DAC) Channel
 * @ingroup GMP_CTL_COMMON_INTERFACES
 * @brief A module for handling a single DAC output channel.
 *
 * This module converts a logical control value into a raw digital value
 * suitable for being written to a DAC hardware register.
 * The conversion follows the formula:
 * //tex: \text{value}_{\text{final}} = (\text{value}_{\text{raw}} \cdot \text{gain}) + \text{bias}
 * @f$
 * \text{value}_{\text{final}} = (\text{value}_{\text{raw}} \cdot \text{gain}) + \text{bias}
 * @f$
 */

/*---------------------------------------------------------------------------*/
/* DAC Channel Module                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup DAC_CHANNEL
 * @{
 */

/**
 * @brief Data structure for a single DAC channel.
 */
typedef struct _tag_dac_channel_t
{
    ctrl_gt raw;        /**< INPUT: The target value to be output by the DAC, in control format (p.u. or IQ). */
    fast_gt resolution; /**< The resolution of the DAC in bits (e.g., 12 for a 12-bit DAC). */
    fast_gt iqn;        /**< The IQ format number for fixed-point representation (e.g., 24 for _IQ24). */
    ctrl_gt gain;       /**< The gain applied to the raw input value. */
    ctrl_gt bias;       /**< The bias or offset added to the raw value after gain scaling. */
    dac_gt value;       /**< OUTPUT: The final calculated raw integer value to be sent to the DAC hardware. */
} dac_channel_t;

void ctl_init_dac_channel(dac_channel_t* dac, ctrl_gt gain, ctrl_gt bias, fast_gt resolution, fast_gt iqn);

/**
 * @brief Sets the input value for the DAC channel.
 * @param dac Pointer to the DAC channel object.
 * @param raw_input The new raw input value in control format.
 */
GMP_STATIC_INLINE void ctl_set_dac_channel_input(dac_channel_t* dac, ctrl_gt raw_input)
{
    dac->raw = raw_input;
}

/**
 * @brief Executes one step of the DAC conversion.
 * @details This function takes the raw input, applies gain and bias, and calculates
 * the final integer value for the DAC peripheral.
 * @param dac Pointer to the DAC channel object.
 * @return The final calculated raw integer value for the DAC.
 */
GMP_STATIC_INLINE dac_gt ctl_step_dac_channel(dac_channel_t* dac)
{
    // Apply gain and bias to the raw control value
    ctrl_gt temp_val = ctl_add(ctl_mul(dac->raw, dac->gain), dac->bias);

#if defined CTRL_GT_IS_FIXED
    // For fixed-point, shift the result to match the DAC resolution
    dac->value = (dac_gt)(temp_val >> (dac->iqn - dac->resolution));
#elif defined CTRL_GT_IS_FLOAT
    // For float, scale the per-unit value by the maximum DAC value
    dac->value = (dac_gt)(temp_val * (float)((1 << dac->resolution) - 1));
#else
#error "The system does not specify ctrl_gt is float or fixed. You should define CTRL_GT_IS_FLOAT or CTRL_GT_IS_FIXED."
#endif // CTRL_GT_IS_XXX

    return dac->value;
}

/**
 * @brief Gets the last calculated raw output value for the DAC.
 * @param dac Pointer to the DAC channel object.
 * @return The raw integer value ready for the DAC hardware.
 */
GMP_STATIC_INLINE dac_gt ctl_get_dac_channel_value(dac_channel_t* dac)
{
    return dac->value;
}

/**
 * @brief Sets the gain for the DAC channel.
 * @param dac Pointer to the DAC channel object.
 * @param gain The new gain value.
 */
GMP_STATIC_INLINE void ctl_set_dac_channel_gain(dac_channel_t* dac, ctrl_gt gain)
{
    dac->gain = gain;
}

/**
 * @brief Gets the current gain of the DAC channel.
 * @param dac Pointer to the DAC channel object.
 * @return The current gain value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_dac_channel_gain(dac_channel_t* dac)
{
    return dac->gain;
}

/**
 * @brief Sets the bias for the DAC channel.
 * @param dac Pointer to the DAC channel object.
 * @param bias The new bias value.
 */
GMP_STATIC_INLINE void ctl_set_dac_channel_bias(dac_channel_t* dac, ctrl_gt bias)
{
    dac->bias = bias;
}

/**
 * @brief Gets the current bias of the DAC channel.
 * @param dac Pointer to the DAC channel object.
 * @return The current bias value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_dac_channel_bias(dac_channel_t* dac)
{
    return dac->bias;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_DAC_CHANNEL_H_
