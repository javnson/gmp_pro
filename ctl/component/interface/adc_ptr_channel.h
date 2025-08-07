/**
 * @file adc_ptr_channel.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides ADC channel processing modules that access raw ADC data via pointers.
 * @details This implementation is an alternative to `adc_channel.h` and is useful
 * when raw ADC values are stored in a contiguous memory block (like a DMA buffer),
 * as it avoids copying data for each processing step.
 * @version 0.1
 * @date 2025-03-21
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_ADC_PTR_CHANNEL_H_
#define _FILE_ADC_PTR_CHANNEL_H_

// #include <gmp_core.h>
#include <ctl/component/interface/interface_base.h>

// It's good practice to include the header for any assertion macros used.
// For example, if gmp_base_assert is based on the standard library assert.
// #include <assert.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup ADC_PTR_SINGLE_CHANNEL Single ADC Channel (Pointer based)
 * @brief A module for processing a single ADC channel using a pointer to the raw data.
 *
 * This module converts a raw ADC value, accessed via a pointer, into a calibrated control value.
 * @f$
 * \text{value} = \text{gain} \cdot (\text{raw}_{\text{scaled}} - \text{bias})
 * @f$
 */

/**
 * @defgroup ADC_PTR_DUAL_CHANNEL Dual ADC Channel (Pointer based)
 * @brief A module for processing two ADC channels using a pointer to the raw data array.
 *
 * Used for sampling two independent analog inputs or a differential pair from a contiguous memory block.
 */

/**
 * @defgroup ADC_PTR_TRI_CHANNEL Triple ADC Channel (Pointer based)
 * @brief A module for processing three ADC channels using a pointer to the raw data array.
 *
 * Designed for three-phase systems where current or voltage samples are stored in an array.
 */

/*---------------------------------------------------------------------------*/
/* Single ADC Channel (Pointer based)                                        */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup ADC_PTR_SINGLE_CHANNEL
 * @{
 */

/**
 * @brief Data structure for a single pointer-based ADC channel.
 */
typedef struct _tag_ptr_adc_channel_t
{
    adc_ift control_port; /**< OUTPUT: The ADC data output interface. */
    adc_gt* raw;          /**< INPUT: Pointer to the raw data from the ADC peripheral. */
    fast_gt resolution;   /**< The resolution of the ADC in bits (e.g., 12 for 12-bit). */

#if defined CTRL_GT_IS_FIXED
    fast_gt shift_index; /**< The bit-shift required to convert the raw value to the target IQ format. */
#elif defined CTRL_GT_IS_FLOAT
    ctrl_gt per_unit_base; /**< The base value for converting the raw integer to a per-unit float. */
#endif

    ctrl_gt bias; /**< The bias or offset of the ADC data. */
    ctrl_gt gain; /**< The gain of the ADC data. Negative gain is permitted. */
} ptr_adc_channel_t;

void ctl_init_ptr_adc_channel(ptr_adc_channel_t* adc, adc_gt* adc_target, ctrl_gt gain, ctrl_gt bias,
                              fast_gt resolution, fast_gt iqn);

/**
 * @brief Processes the raw ADC input (via pointer) and computes the final scaled value.
 * @details This function must be called every control cycle to update the value.
 * @param adc_obj Pointer to the pointer-based ADC channel object.
 * @return The final, calibrated ADC value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_ptr_adc_channel(ptr_adc_channel_t* adc_obj)
{
    // Ensure the raw pointer is valid.
    gmp_base_assert(adc_obj->raw);

    adc_gt raw = *adc_obj->raw;
    ctrl_gt raw_data;

#if defined CTRL_GT_IS_FIXED
    // For fixed-point, shift the raw data to the correct IQ format.
    raw_data = raw << adc_obj->shift_index;
#elif defined CTRL_GT_IS_FLOAT
    // For floating-point, scale the raw data to a per-unit value.
    raw_data = (ctrl_gt)raw * adc_obj->per_unit_base;
#else
#error "The system does not specify ctrl_gt is float or fixed. You should define CTRL_GT_IS_FLOAT or CTRL_GT_IS_FIXED."
#endif // CTRL_GT_IS_XXX

    ctrl_gt raw_without_bias = raw_data - adc_obj->bias;
    adc_obj->control_port.value = ctl_mul(raw_without_bias, adc_obj->gain);

    return adc_obj->control_port.value;
}

/**
 * @brief Gets the most recently calculated ADC value.
 * @param adc Pointer to the ADC channel object.
 * @return The last computed ADC value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_ptr_adc_data(ptr_adc_channel_t* adc)
{
    return adc->control_port.value;
}

/**
 * @brief Sets the bias for the ADC channel.
 * @param adc Pointer to the ADC channel object.
 * @param bias The new bias value to set.
 */
GMP_STATIC_INLINE void ctl_set_ptr_adc_channel_bias(ptr_adc_channel_t* adc, ctrl_gt bias)
{
    adc->bias = bias;
}

/**
 * @brief Gets the current bias of the ADC channel.
 * @param adc Pointer to the ADC channel object.
 * @return The current bias value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_ptr_adc_channel_bias(ptr_adc_channel_t* adc)
{
    return adc->bias;
}

/**
 * @brief Gets a pointer to the control port interface of the ADC channel.
 * @param adc Pointer to the ADC channel object.
 * @return Pointer to the `adc_ift` control port.
 */
GMP_STATIC_INLINE adc_ift* ctl_get_ptr_adc_channel_ctrl_port(ptr_adc_channel_t* adc)
{
    return &adc->control_port;
}

/**
 * @}
 */

/*---------------------------------------------------------------------------*/
/* Dual ADC Channel (Pointer based)                                          */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup ADC_PTR_DUAL_CHANNEL
 * @{
 */

/**
 * @brief Data structure for a dual pointer-based ADC channel.
 * @details The raw data is accessed from a continuous array via a pointer.
 * @f$
 * \text{value}[i] = \text{gain}[i] \cdot (\text{raw}_{\text{scaled}}[i] - \text{bias}[i])
 * @f$
 */
typedef struct _tag_adc_ptr_dual_channel_t
{
    dual_adc_ift control_port; /**< OUTPUT: The dual ADC data output interface. */
    adc_gt* raw;               /**< INPUT: Pointer to a fixed-length (2) array of raw ADC data. */
    fast_gt resolution;        /**< The resolution of the ADC in bits. */

#if defined CTRL_GT_IS_FIXED
    fast_gt shift_index; /**< The bit-shift required to convert raw values to the target IQ format. */
#elif defined CTRL_GT_IS_FLOAT
    ctrl_gt per_unit_base; /**< The base value for converting raw integers to per-unit floats. */
#endif

    ctrl_gt bias[2]; /**< The bias of the ADC data for each channel. */
    ctrl_gt gain[2]; /**< The gain of the ADC data for each channel. */
} dual_ptr_adc_channel_t;

void ctl_init_dual_ptr_adc_channel(dual_ptr_adc_channel_t* adc, adc_gt* adc_target, ctrl_gt gain, ctrl_gt bias,
                                   fast_gt resolution, fast_gt iqn);

/**
 * @brief Processes the raw inputs for both ADC channels from the pointer source.
 * @param adc_obj Pointer to the dual pointer-based ADC channel object.
 */
GMP_STATIC_INLINE void ctl_step_dual_ptr_adc_channel(dual_ptr_adc_channel_t* adc_obj)
{
    gmp_base_assert(adc_obj->raw);

    for (int i = 0; i < 2; ++i)
    {
        ctrl_gt raw_data;
#if defined CTRL_GT_IS_FIXED
        raw_data = adc_obj->raw[i] << adc_obj->shift_index;
#elif defined CTRL_GT_IS_FLOAT
        raw_data = (ctrl_gt)adc_obj->raw[i] * adc_obj->per_unit_base;
#else
#error "The system does not specify ctrl_gt is float or fixed. You should define CTRL_GT_IS_FLOAT or CTRL_GT_IS_FIXED."
#endif // CTRL_GT_IS_XXX

        ctrl_gt raw_without_bias = raw_data - adc_obj->bias[i];
        adc_obj->control_port.value.dat[i] = ctl_mul(raw_without_bias, adc_obj->gain[i]);
    }
}

/**
 * @brief Gets the processed data from both channels.
 * @param adc Pointer to the dual ADC channel object.
 * @param dat1 Pointer to store the value of channel 1.
 * @param dat2 Pointer to store the value of channel 2.
 */
GMP_STATIC_INLINE void ctl_get_dual_ptr_adc_channel(dual_ptr_adc_channel_t* adc, ctrl_gt* dat1, ctrl_gt* dat2)
{
    *dat1 = adc->control_port.value.dat[0];
    *dat2 = adc->control_port.value.dat[1];
}

/**
 * @brief Gets the processed data from channel 1.
 * @param adc Pointer to the dual ADC channel object.
 * @return The processed value of channel 1.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_dual_ptr_adc_channel1(dual_ptr_adc_channel_t* adc)
{
    return adc->control_port.value.dat[0];
}

/**
 * @brief Gets the processed data from channel 2.
 * @param adc Pointer to the dual ADC channel object.
 * @return The processed value of channel 2.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_dual_ptr_adc_channel2(dual_ptr_adc_channel_t* adc)
{
    return adc->control_port.value.dat[1];
}

/**
 * @brief Calculates the differential value between the two channels (ch1 - ch2).
 * @param adc Pointer to the dual ADC channel object.
 * @return The result of `channel_1 - channel_2`.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_dual_ptr_adc_channel_diff(dual_ptr_adc_channel_t* adc)
{
    return adc->control_port.value.dat[0] - adc->control_port.value.dat[1];
}

/**
 * @brief Calculates the common-mode value of the two channels ((ch1 + ch2) / 2).
 * @param adc Pointer to the dual ADC channel object.
 * @return The result of `(channel_1 + channel_2) / 2`.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_dual_ptr_adc_channel_comm(dual_ptr_adc_channel_t* adc)
{
    return ctl_div2(adc->control_port.value.dat[0] + adc->control_port.value.dat[1]);
}

/**
 * @brief Gets the processed data from both channels as a 2D vector.
 * @param adc Pointer to the dual ADC channel object.
 * @param vector Pointer to a `ctl_vector2_t` to store the result.
 */
GMP_STATIC_INLINE void ctl_get_dual_ptr_adc_channel_via_vector2(dual_ptr_adc_channel_t* adc, ctl_vector2_t* vector)
{
    vector->dat[0] = adc->control_port.value.dat[0];
    vector->dat[1] = adc->control_port.value.dat[1];
}

/**
 * @brief Gets a pointer to the control port interface of the dual ADC channel.
 * @param adc Pointer to the dual ADC channel object.
 * @return Pointer to the `dual_adc_ift` control port.
 */
GMP_STATIC_INLINE dual_adc_ift* ctl_get_dual_ptr_adc_channel_ctrl_port(dual_ptr_adc_channel_t* adc)
{
    return &adc->control_port;
}

/**
 * @}
 */

/*---------------------------------------------------------------------------*/
/* Triple ADC Channel (Pointer based)                                        */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup ADC_PTR_TRI_CHANNEL
 * @{
 */

/**
 * @brief Data structure for a three-channel pointer-based ADC module.
 * @details The raw data is accessed from a continuous array via a pointer.
 * @f$
 * \text{value}[i] = \text{gain}[i] \cdot (\text{raw}_{\text{scaled}}[i] - \text{bias}[i])
 * @f$
 */
typedef struct _tag_tri_ptr_adc_channel_t
{
    tri_adc_ift control_port; /**< OUTPUT: The triple ADC data output interface. */
    adc_gt* raw;              /**< INPUT: Pointer to a fixed-length (3) array of raw ADC data. */
    fast_gt resolution;       /**< The resolution of the ADC in bits. */

#if defined CTRL_GT_IS_FIXED
    fast_gt shift_index; /**< The bit-shift required to convert raw values to the target IQ format. */
#elif defined CTRL_GT_IS_FLOAT
    ctrl_gt per_unit_base; /**< The base value for converting raw integers to per-unit floats. */
#endif

    ctrl_gt bias[3]; /**< The bias of the ADC data for each channel. */
    ctrl_gt gain[3]; /**< The gain of the ADC data for each channel. */
} tri_ptr_adc_channel_t;

void ctl_init_tri_ptr_adc_channel(tri_ptr_adc_channel_t* adc, adc_gt* adc_target, ctrl_gt gain, ctrl_gt bias,
                                  fast_gt resolution, fast_gt iqn);

/**
 * @brief Processes the raw inputs for all three ADC channels from the pointer source.
 * @param adc_obj Pointer to the triple pointer-based ADC channel object.
 */
GMP_STATIC_INLINE void ctl_step_tri_ptr_adc_channel(tri_ptr_adc_channel_t* adc_obj)
{
    gmp_base_assert(adc_obj->raw);

    for (int i = 0; i < 3; ++i)
    {
        ctrl_gt raw_data;
#if defined CTRL_GT_IS_FIXED
        raw_data = adc_obj->raw[i] << adc_obj->shift_index;
#elif defined CTRL_GT_IS_FLOAT
        raw_data = (ctrl_gt)adc_obj->raw[i] * adc_obj->per_unit_base;
#else
#error "The system does not specify ctrl_gt is float or fixed. You should define CTRL_GT_IS_FLOAT or CTRL_GT_IS_FIXED."
#endif // CTRL_GT_IS_XXX

        ctrl_gt raw_without_bias = raw_data - adc_obj->bias[i];
        adc_obj->control_port.value.dat[i] = ctl_mul(raw_without_bias, adc_obj->gain[i]);
    }
}

/**
 * @brief Gets the processed data from all three channels as a 3D vector.
 * @param adc Pointer to the triple ADC channel object.
 * @param vec Pointer to a `ctl_vector3_t` to store the result.
 */
GMP_STATIC_INLINE void ctl_get_tri_ptr_adc_channel_via_vector3(tri_ptr_adc_channel_t* adc, ctl_vector3_t* vec)
{
    vec->dat[0] = adc->control_port.value.dat[0];
    vec->dat[1] = adc->control_port.value.dat[1];
    vec->dat[2] = adc->control_port.value.dat[2];
}

/**
 * @brief Sets the bias for a specific channel in the triple ADC object.
 * @param adc Pointer to the triple ADC channel object.
 * @param index The channel index (0, 1, or 2).
 * @param bias The new bias value to set.
 */
GMP_STATIC_INLINE void ctl_set_tri_ptr_adc_channel_bias(tri_ptr_adc_channel_t* adc, fast_gt index, ctrl_gt bias)
{
    gmp_base_assert(index < 3);
    adc->bias[index] = bias;
}

/**
 * @brief Gets the bias of a specific channel.
 * @param adc Pointer to the triple ADC channel object.
 * @param index The channel index (0, 1, or 2).
 * @return The current bias value of the specified channel.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_tri_ptr_adc_bias(tri_ptr_adc_channel_t* adc, fast_gt index)
{
    gmp_base_assert(index < 3);
    return adc->bias[index];
}

/**
 * @brief Gets a pointer to the control port interface of the triple ADC channel.
 * @param adc Pointer to the triple ADC channel object.
 * @return Pointer to the `tri_adc_ift` control port.
 */
GMP_STATIC_INLINE tri_adc_ift* ctl_get_tri_ptr_adc_channel_ctrl_port(tri_ptr_adc_channel_t* adc)
{
    return &adc->control_port;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ADC_PTR_CHANNEL_H_
