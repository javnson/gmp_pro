#ifndef _FILE_CTL_ADV_ADC_H_
#define _FILE_CTL_ADV_ADC_H_

/*---------------------------------------------------------------------------*/
/* Advanced ADC Channel (Direct LUT Mapping)                                 */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup ADV_ADC_CHANNEL Advanced ADC Channel
 * @brief Extension of the standard ADC channel with non-linear LUT calibration.
 * @{
 */

/**
 * @brief Data structure for an advanced ADC channel supporting LUT calibration.
 */
typedef struct _tag_ctl_adv_adc_channel
{
    adc_ift control_port; /**< OUTPUT: Universal ADC data interface. */
    ctl_paired_lut1d_t
        lut; /**< Embedded calibration table mapping [0.0, 1.0] normalized input to corrected physical/pu [Y]. */
    
    fast_gt resolution; /**< ADC hardware resolution in bits (e.g., 12 for 12-bit). */
    fast_gt iqn;        /**< Fixed-point representation format (e.g., 24 for _IQ24). */
} ctl_adv_adc_channel_t;

/**
 * @brief Initializes the advanced ADC channel.
 * @param[out] adv_adc Pointer to the advanced ADC channel instance.
 * @param[in]  resolution ADC hardware bit-resolution (e.g., 12).
 * @param[in]  iqn IQ format selection for fixed-point math (e.g., 24).
 * @param[in]  table Pointer to the monotonic paired lookup array {x, y}.
 * @param[in]  size Number of elements in the lookup table.
 */
void ctl_init_adv_adc_channel(ctl_adv_adc_channel_t* adv_adc, fast_gt resolution, fast_gt iqn,
                              const ctl_lut1d_pair_t* table, uint32_t size);

/**
 * @brief Runs advanced correction step on the raw ADC hardware input.
 * @details Directly normalizes the raw ADC integer to the [0.0, 1.0] range, 
 * then maps it to the target output using high-efficiency 1D linear interpolation.
 * Offset and gain scaling are natively processed inside the calibration LUT.
 * @param[in,out] adv_adc Pointer to the advanced ADC channel instance.
 * @param[in]     raw The raw digital register value from the ADC peripheral.
 * @return ctrl_gt Calibrated and linearized output value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_adv_adc_channel(ctl_adv_adc_channel_t* adv_adc, adc_gt raw)
{
    // 1. Normalize raw ADC reading to [0.0, 1.0] pu domain
#if defined CTRL_GT_IS_FIXED
    ctrl_gt raw_normalized = (ctrl_gt)raw << (adv_adc->iqn - adv_adc->resolution);
#elif defined CTRL_GT_IS_FLOAT
    ctrl_gt raw_normalized = (ctrl_gt)raw / (1 << adv_adc->resolution);
#else
#error "System configuration error: Define CTRL_GT_IS_FLOAT or CTRL_GT_IS_FIXED."
#endif

    // 2. Direct lookup of calibrated/scaled output (implicitly handles gain and offset)
    adv_adc->control_port.value = ctl_step_interpolate_paired_lut1d(&adv_adc->lut, raw_normalized);

    return adv_adc->control_port.value;
}

/**
 * @brief Retrieves the last calculated calibrated ADC value.
 * @param[in] adv_adc Pointer to the advanced ADC channel instance.
 * @return The last computed ADC value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_adv_adc_data(ctl_adv_adc_channel_t* adv_adc)
{
    return adv_adc->control_port.value;
}

/**
 * @}
 */

#endif // _FILE_CTL_ADV_ADC_H_
