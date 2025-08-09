/**
 * @file adc_channel.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides structures and functions for handling single, dual, and triple ADC channels,
 * as well as a utility for ADC bias calibration. This module supports both fixed-point
 * and floating-point arithmetic.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_ADC_CHANNEL_H_
#define _FILE_ADC_CHANNEL_H_

// #include <gmp_core.h>
#include <ctl/component/interface/interface_base.h>

// help user to calculate ADC bias and gain
#include <ctl/component/interface/bias_model.h>
#include <ctl/component/interface/gain_model.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup ADC_SINGLE_CHANNEL Single ADC Channel
 * @ingroup GMP_CTL_COMMON_INTERFACES
 * @brief A module for processing a single ADC channel.
 *
 * This module converts raw ADC data into a calibrated control value using a specified gain and bias.
 * @f$
 * \text{value} = \text{gain} \cdot (\text{raw}_{\text{scaled}} - \text{bias})
 * @f$
 */

/**
 * @defgroup ADC_DUAL_CHANNEL Dual ADC Channel
 * @ingroup GMP_CTL_COMMON_INTERFACES
 * @brief A module for processing two ADC channels.
 *
 * This module is typically used for sampling two independent analog inputs or a differential pair.
 */

/**
 * @defgroup ADC_TRI_CHANNEL Triple ADC Channel
 * @ingroup GMP_CTL_COMMON_INTERFACES
 * @brief A module for processing three ADC channels.
 *
 * This module is designed for three-phase systems, such as motor control or power grid applications.
 */

/**
 * @defgroup ADC_BIAS_CALIBRATOR ADC Bias Calibrator
 * @ingroup GMP_CTL_COMMON_INTERFACES
 * @brief A utility for calibrating the offset (bias) of an ADC channel.
 *
 * This module uses a low-pass filter to determine the DC bias from a series of ADC readings.
 */

/*---------------------------------------------------------------------------*/
/* Single ADC Channel                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup ADC_SINGLE_CHANNEL
 * @{
 */

/**
 * @brief Data structure for a single ADC channel.
 */
typedef struct _tag_adc_channel_t
{
    adc_ift control_port; /**< OUTPUT: The ADC data output interface. */
    adc_gt raw;           /**< INPUT: Raw data from the ADC peripheral. */
    fast_gt resolution;   /**< The resolution of the ADC in bits (e.g., 12 for 12-bit). */
    fast_gt iqn;          /**< The IQ format number for fixed-point representation (e.g., 24 for _IQ24). */
    ctrl_gt bias;         /**< The bias or offset of the ADC data. */
    ctrl_gt gain;         /**< The gain of the ADC data. Negative gain is permitted. */
} adc_channel_t;

void ctl_init_adc_channel(adc_channel_t* adc_obj, ctrl_gt gain, ctrl_gt bias, fast_gt resolution, fast_gt iqn);

/**
 * @brief Processes the raw ADC input and computes the final scaled value.
 * @param adc Pointer to the ADC channel object.
 * @param raw The raw ADC value to process.
 * @return The final, calibrated ADC value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_adc_channel(adc_channel_t* adc, adc_gt raw)
{
    adc->raw = raw;

#if defined CTRL_GT_IS_FIXED
    // For fixed-point number, transfer ADC data to the specified IQn format
    ctrl_gt raw_data = adc->raw << (adc->iqn - adc->resolution);
#elif defined CTRL_GT_IS_FLOAT
    // For floating-point number, transfer ADC data to per-unit (p.u.)
    ctrl_gt raw_data = (ctrl_gt)adc->raw / (1 << adc->resolution);
#else
#error "The system does not specify ctrl_gt is float or fixed. You should define CTRL_GT_IS_FLOAT or CTRL_GT_IS_FIXED."
#endif // CTRL_GT_IS_XXX

    // Remove bias
    ctrl_gt raw_without_bias = raw_data - adc->bias;

    // Apply gain
    adc->control_port.value = ctl_mul(raw_without_bias, adc->gain);

    return adc->control_port.value;
}

/**
 * @brief Gets the most recently calculated ADC value.
 * @param adc Pointer to the ADC channel object.
 * @return The last computed ADC value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_adc_data(adc_channel_t* adc)
{
    return adc->control_port.value;
}

/**
 * @brief Sets the bias for the ADC channel.
 * @param adc Pointer to the ADC channel object.
 * @param bias The new bias value to set.
 */
GMP_STATIC_INLINE void ctl_set_adc_channel_bias(adc_channel_t* adc, ctrl_gt bias)
{
    adc->bias = bias;
}

/**
 * @brief Gets the current bias of the ADC channel.
 * @param adc Pointer to the ADC channel object.
 * @return The current bias value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_adc_channel_bias(adc_channel_t* adc)
{
    return adc->bias;
}

/**
 * @brief Gets a pointer to the control port interface of the ADC channel.
 * @param adc Pointer to the ADC channel object.
 * @return Pointer to the `adc_ift` control port.
 */
GMP_STATIC_INLINE adc_ift* ctl_get_adc_channel_ctrl_port(adc_channel_t* adc)
{
    return &adc->control_port;
}

/**
 * @}
 */

/*---------------------------------------------------------------------------*/
/* Dual ADC Channel                                                          */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup ADC_DUAL_CHANNEL
 * @{
 */

/**
 * @brief Data structure for a dual ADC channel peripheral interface.
 * @details This can be used to sample two independent channels or a differential pair.
 * The conversion follows the formula:
 * @f$
 * \text{value}[i] = \text{gain}[i] \cdot (\text{raw}_{\text{scaled}}[i] - \text{bias}[i])
 * @f$
 */
typedef struct _tag_adc_dual_channel_t
{
    dual_adc_ift control_port; /**< OUTPUT: The dual ADC data output interface. */
    adc_gt raw[2];             /**< INPUT: Raw data from the ADC peripheral for both channels. */
    fast_gt resolution;        /**< The resolution of the ADC in bits. */
    fast_gt iqn;               /**< The IQ format number for fixed-point representation. */
    ctrl_gt bias[2];           /**< The bias of the ADC data for each channel. */
    ctrl_gt gain[2];           /**< The gain of the ADC data for each channel. */
} adc_dual_channel_t, dual_adc_channel_t;

void ctl_init_dual_adc_channel(dual_adc_channel_t* adc, ctrl_gt gain, ctrl_gt bias, fast_gt resolution, fast_gt iqn);

/**
 * @brief Processes the raw inputs for both ADC channels.
 * @param adc_obj Pointer to the dual ADC channel object.
 * @param raw1 The raw ADC value for channel 1.
 * @param raw2 The raw ADC value for channel 2.
 */
GMP_STATIC_INLINE void ctl_step_dual_adc_channel(dual_adc_channel_t* adc_obj, adc_gt raw1, adc_gt raw2)
{
    ctrl_gt raw_data;
    ctrl_gt raw_without_bias;

    adc_obj->raw[0] = raw1;
    adc_obj->raw[1] = raw2;

    for (int i = 0; i < 2; ++i)
    {
#if defined CTRL_GT_IS_FIXED
        raw_data = adc_obj->raw[i] << (adc_obj->iqn - adc_obj->resolution);
#elif defined CTRL_GT_IS_FLOAT
        raw_data = (ctrl_gt)adc_obj->raw[i] / (1 << adc_obj->resolution);
#else
#error "The system does not specify ctrl_gt is float or fixed. You should define CTRL_GT_IS_FLOAT or CTRL_GT_IS_FIXED."
#endif // CTRL_GT_IS_XXX

        raw_without_bias = raw_data - adc_obj->bias[i];
        adc_obj->control_port.value.dat[i] = ctl_mul(raw_without_bias, adc_obj->gain[i]);
    }
}

/**
 * @brief Gets the processed data from both channels.
 * @param adc Pointer to the dual ADC channel object.
 * @param dat1 Pointer to store the value of channel 1.
 * @param dat2 Pointer to store the value of channel 2.
 */
GMP_STATIC_INLINE void ctl_get_dual_adc_channel(dual_adc_channel_t* adc, ctrl_gt* dat1, ctrl_gt* dat2)
{
    *dat1 = adc->control_port.value.dat[0];
    *dat2 = adc->control_port.value.dat[1];
}

/**
 * @brief Gets the processed data from channel 1.
 * @param adc Pointer to the dual ADC channel object.
 * @return The processed value of channel 1.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_dual_adc_channel1(dual_adc_channel_t* adc)
{
    return adc->control_port.value.dat[0];
}

/**
 * @brief Gets the processed data from channel 2.
 * @param adc Pointer to the dual ADC channel object.
 * @return The processed value of channel 2.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_dual_adc_channel2(dual_adc_channel_t* adc)
{
    return adc->control_port.value.dat[1];
}

/**
 * @brief Calculates the differential value between the two channels (ch1 - ch2).
 * @param adc Pointer to the dual ADC channel object.
 * @return The result of `channel_1 - channel_2`.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_dual_adc_channel_diff(dual_adc_channel_t* adc)
{
    return adc->control_port.value.dat[0] - adc->control_port.value.dat[1];
}

/**
 * @brief Calculates the common-mode value of the two channels ((ch1 + ch2) / 2).
 * @param adc Pointer to the dual ADC channel object.
 * @return The result of `(channel_1 + channel_2) / 2`.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_dual_adc_channel_comm(dual_adc_channel_t* adc)
{
    return ctl_div2(adc->control_port.value.dat[0] + adc->control_port.value.dat[1]);
}

/**
 * @brief Gets the processed data from both channels as a 2D vector.
 * @param adc Pointer to the dual ADC channel object.
 * @param vector Pointer to a `ctl_vector2_t` to store the result.
 */
GMP_STATIC_INLINE void ctl_get_dual_adc_channel_via_vector2(dual_adc_channel_t* adc, ctl_vector2_t* vector)
{
    vector->dat[0] = adc->control_port.value.dat[0];
    vector->dat[1] = adc->control_port.value.dat[1];
}

/**
 * @brief Gets a pointer to the control port interface of the dual ADC channel.
 * @param adc Pointer to the dual ADC channel object.
 * @return Pointer to the `dual_adc_ift` control port.
 */
GMP_STATIC_INLINE dual_adc_ift* ctl_get_dual_adc_channel_ctrl_port(dual_adc_channel_t* adc)
{
    return &adc->control_port;
}

/**
 * @}
 */

/*---------------------------------------------------------------------------*/
/* Triple ADC Channel                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup ADC_TRI_CHANNEL
 * @{
 */

/**
 * @brief Data structure for a three-channel ADC module.
 * @details Primarily for three-phase systems. The conversion follows the formula:
 * @f$
 * \text{value}[i] = \text{gain}[i] \cdot (\text{raw}_{\text{scaled}}[i] - \text{bias}[i])
 * @f$
 */
typedef struct _tag_tri_adc_channel_t
{
    tri_adc_ift control_port; /**< OUTPUT: The triple ADC data output interface. */
    adc_gt raw[3];            /**< INPUT: Raw data from the ADC for all three channels. */
    fast_gt resolution;       /**< The resolution of the ADC in bits. */
    fast_gt iqn;              /**< The IQ format number for fixed-point representation. */
    ctrl_gt bias[3];          /**< The bias of the ADC data for each channel. */
    ctrl_gt gain[3];          /**< The gain of the ADC data for each channel. */
} tri_adc_channel_t;

void ctl_init_tri_adc_channel(tri_adc_channel_t* adc, ctrl_gt gain, ctrl_gt bias, fast_gt resolution, fast_gt iqn);

/**
 * @brief Processes the raw inputs for all three ADC channels.
 * @param adc_obj Pointer to the triple ADC channel object.
 * @param raw1 The raw ADC value for channel 1.
 * @param raw2 The raw ADC value for channel 2.
 * @param raw3 The raw ADC value for channel 3.
 */
GMP_STATIC_INLINE void ctl_step_adc_tri_channel(tri_adc_channel_t* adc_obj, adc_gt raw1, adc_gt raw2, adc_gt raw3)
{
    ctrl_gt raw_data;
    ctrl_gt raw_without_bias;

    adc_obj->raw[0] = raw1;
    adc_obj->raw[1] = raw2;
    adc_obj->raw[2] = raw3;

    for (int i = 0; i < 3; ++i)
    {
#if defined CTRL_GT_IS_FIXED
        raw_data = adc_obj->raw[i] << (adc_obj->iqn - adc_obj->resolution);
#elif defined CTRL_GT_IS_FLOAT
        raw_data = (ctrl_gt)adc_obj->raw[i] / (1 << adc_obj->resolution);
#else
#error "The system does not specify ctrl_gt is float or fixed. You should define CTRL_GT_IS_FLOAT or CTRL_GT_IS_FIXED."
#endif // CTRL_GT_IS_XXX

        raw_without_bias = raw_data - adc_obj->bias[i];
        adc_obj->control_port.value.dat[i] = ctl_mul(raw_without_bias, adc_obj->gain[i]);
    }
}

/**
 * @brief Gets the processed data from all three channels as a 3D vector.
 * @param adc Pointer to the triple ADC channel object.
 * @param vec Pointer to a `ctl_vector3_t` to store the result.
 */
GMP_STATIC_INLINE void ctl_get_tri_adc_channel_via_vector3(tri_adc_channel_t* adc, ctl_vector3_t* vec)
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
GMP_STATIC_INLINE void ctl_set_tri_adc_channel_bias(tri_adc_channel_t* adc, fast_gt index, ctrl_gt bias)
{
    adc->bias[index] = bias;
}

/**
 * @brief Gets the bias of a specific channel.
 * @param adc Pointer to the triple ADC channel object.
 * @param index The channel index (0, 1, or 2).
 * @return The current bias value of the specified channel.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_tri_adc_bias(tri_adc_channel_t* adc, fast_gt index)
{
    return adc->bias[index];
}

/**
 * @brief Gets a pointer to the control port interface of the triple ADC channel.
 * @param adc Pointer to the triple ADC channel object.
 * @return Pointer to the `tri_adc_ift` control port.
 */
GMP_STATIC_INLINE tri_adc_ift* ctl_get_tri_adc_channel_ctrl_port(tri_adc_channel_t* adc)
{
    return &adc->control_port;
}

/**
 * @}
 */

/*---------------------------------------------------------------------------*/
/* ADC Bias Calibrator                                                       */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup ADC_BIAS_CALIBRATOR
 * @{
 */

#include <ctl/component/intrinsic/discrete/biquad_filter.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>

/**
 * @brief Data structure for the ADC bias calibrator.
 */
typedef struct _tag_adc_bias_calibrator_t
{
    ctrl_gt raw;              /**< INPUT: Raw ADC data fed into the calibrator. */
    ctl_filter_IIR2_t filter; /**< The 2nd-order IIR filter used for averaging. */
    uint32_t filter_tick;     /**< Counter for the number of samples processed. */
    uint32_t filter_period;   /**< The total number of samples to average over. */
    fast_gt enable_filter;    /**< Flag to enable or disable the calibration process. */
    fast_gt output_valid;     /**< Flag indicating that the calibration result is valid. */
} adc_bias_calibrator_t;

void ctl_init_adc_calibrator(adc_bias_calibrator_t* obj, parameter_gt fc, parameter_gt Q, parameter_gt fs);

/**
 * @brief Enables the ADC bias calibration process.
 * @param obj Pointer to the ADC bias calibrator object.
 */
void ctl_enable_adc_calibrator(adc_bias_calibrator_t* obj);

/**
 * @brief Checks if the ADC calibration period is complete.
 * @param obj Pointer to the ADC bias calibrator object.
 * @return 1 if complete, 0 otherwise.
 */
GMP_STATIC_INLINE fast_gt ctl_is_adc_calibrator_cmpt(adc_bias_calibrator_t* obj)
{
    return (obj->filter_tick > obj->filter_period) ? 1 : 0;
}

/**
 * @brief Checks if the ADC calibrator is currently enabled.
 * @param obj Pointer to the ADC bias calibrator object.
 * @return 1 if enabled, 0 if disabled.
 */
GMP_STATIC_INLINE fast_gt ctl_is_adc_calibrator_enabled(adc_bias_calibrator_t* obj)
{
    return obj->enable_filter;
}

/**
 * @brief Checks if the calibration result is valid.
 * @param obj Pointer to the ADC bias calibrator object.
 * @return 1 if the result is valid, 0 otherwise.
 */
GMP_STATIC_INLINE fast_gt ctl_is_adc_calibrator_result_valid(adc_bias_calibrator_t* obj)
{
    return obj->output_valid;
}

/**
 * @brief Resets the calibrator to prepare for a new calibration sequence.
 * @param obj Pointer to the ADC bias calibrator object.
 */
GMP_STATIC_INLINE void ctl_clear_adc_calibrator(adc_bias_calibrator_t* obj)
{
    obj->filter_tick = 0;
    obj->raw = 0;
    obj->output_valid = 0;
    obj->enable_filter = 0;
}

/**
 * @brief Executes one step of the calibration process. This should be called periodically (e.g., in an ISR).
 * @param obj Pointer to the ADC bias calibrator object.
 * @param adc_value The current ADC value to be included in the calibration.
 */
GMP_STATIC_INLINE void ctl_step_adc_calibrator(adc_bias_calibrator_t* obj, ctrl_gt adc_value)
{
    obj->raw = adc_value;

    if (obj->enable_filter)
    {
        ctl_step_biquad_filter(&obj->filter, adc_value);
        obj->filter_tick += 1;

        if (ctl_is_adc_calibrator_cmpt(obj))
        {
            obj->enable_filter = 0;
            obj->output_valid = 1;
        }
    }
}

/**
 * @brief Gets the calculated ADC bias result from the filter.
 * @param obj Pointer to the ADC bias calibrator object.
 * @return The calculated average bias value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_adc_calibrator_result(adc_bias_calibrator_t* obj)
{
    return ctl_get_biquad_filter_output(&obj->filter);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _FILE_ADC_CHANNEL_H_
