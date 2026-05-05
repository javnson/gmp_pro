/**
 * @file ctl_protection.h
 * @author GMP Library Contributors
 * @brief Specialized Power Electronics Protection Library.
 * 
 * @details
 * Temperature Sensor Node (PT100 / PT1000 Specific) 
 * 
 * @version 2.0 (Specialized Node Architecture)
 * @copyright Copyright GMP(c) 2024-2026
 */

#ifndef _CTL_PROTECTION_H_
#define _CTL_PROTECTION_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Protection node for PT100/PT1000 temperature sensors.
 * @details Converts raw ADC input to Ohms, calculates real-time Celsius temperature,
 * and trips if the temperature exceeds the safe limit. 
 * Optimized to use only 1 subtraction and 2 multiplications per step.
 */
typedef struct _tag_prot_pt_sensor_t
{
    fast_gt is_enabled;        //!< Master enable flag.
    parameter_gt temp_limit_c; //!< Upper temperature limit in Celsius (Supremum).

    // --- Sensor Conversion Parameters ---
    parameter_gt adc_to_ohm_gain; //!< Gain to convert raw ADC/PU input to Resistance (Ohms).
    parameter_gt r0;              //!< Base resistance at 0¡ãC (e.g., 100.0 for PT100).
    parameter_gt inv_alpha_r0;    //!< Pre-calculated constant: 1 / (R0 * alpha) for fast execution.

    // --- State & Diagnostics ---
    uint16_t trip_limit_count;     //!< Debounce limit (usually high for thermal inertia).
    uint16_t current_count;        //!< Current violation counter.
    uint32_t status_bit;           //!< Global error bitmask.
    parameter_gt fault_record_val; //!< Recorded Peak Temperature in Celsius!

} ctl_prot_pt_sensor_t;

/**
 * @brief Initializes a PT-series temperature protection node.
 * 
 * @param[out] node Pointer to the temperature protection node.
 * @param[in]  status_bit The error bitmask for this fault.
 * @param[in]  temp_limit_c Maximum allowed temperature in Celsius.
 * @param[in]  adc_gain The hardware-specific gain mapping ADC PU to Ohms.
 * @param[in]  r0 Sensor base resistance (100.0f for PT100, 1000.0f for PT1000).
 * @param[in]  alpha Temperature coefficient (typically 0.00385f for Platinum).
 * @param[in]  trip_limit Debounce count.
 */
void ctl_init_prot_pt_sensor(ctl_prot_pt_sensor_t* node, uint32_t status_bit, parameter_gt temp_limit_c,
                             parameter_gt adc_gain, parameter_gt r0, parameter_gt alpha, uint16_t trip_limit);

/**
 * @brief Step: Evaluates PT100/PT1000 Temperature Protection (Peak Mode).
 * @note Because thermal dynamics are extremely slow, this function should IDEALLY 
 * be placed in the slow background loop (e.g., 1ms or 10ms task), NOT the 20kHz ISR.
 * 
 * @param[in,out] node Pointer to the temperature node.
 * @param[in] raw_adc_input The raw ADC feedback (Voltage, PU, or raw integer).
 * @return uint32_t The status bit if tripped, otherwise 0.
 */
GMP_STATIC_INLINE uint32_t ctl_step_prot_pt_sensor(ctl_prot_pt_sensor_t* node, parameter_gt raw_adc_input)
{
    if (!node->is_enabled)
        return 0;

    // 1. Convert ADC input to Ohms
    parameter_gt r_sensor = raw_adc_input * node->adc_to_ohm_gain;

    // 2. Ultra-fast Celsius Calculation: T = (R_sensor - R0) * inv_alpha_r0
    parameter_gt current_temp_c = (r_sensor - node->r0) * node->inv_alpha_r0;

    // 3. High Limit Peak-Hold Evaluation
    if (current_temp_c > node->temp_limit_c)
    {
        if (node->current_count < node->trip_limit_count)
        {
            node->current_count++;
        }

        if (node->current_count >= node->trip_limit_count)
        {
            // Record the Peak Temperature for the Blackbox
            if (current_temp_c > node->fault_record_val)
            {
                node->fault_record_val = current_temp_c;
            }
            return node->status_bit;
        }
    }
    else
    {
        node->current_count = 0;
    }
    return 0;
}


#ifdef __cplusplus
}
#endif

#endif // _CTL_PROTECTION_H_
