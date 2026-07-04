#include <gmp_core.h>

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
                             parameter_gt adc_gain, parameter_gt r0, parameter_gt alpha, uint16_t trip_limit)
{
    node->is_enabled = 0;
    node->temp_limit_c = temp_limit_c;

    node->adc_to_ohm_gain = adc_gain;
    node->r0 = r0;

    // Pre-calculate the inversion constant to avoid division during real-time execution.
    // For PT100 (alpha=0.00385): inv_alpha_r0 = 1 / (100 * 0.00385) = 2.5974
    parameter_gt pre_calc = 1.0f / (r0 * alpha);
    node->inv_alpha_r0 = pre_calc;

    node->trip_limit_count = trip_limit;
    node->current_count = 0;
    node->status_bit = status_bit;
    node->fault_record_val = 0.0f;
}
