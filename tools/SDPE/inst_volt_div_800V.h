/**
 * @file
 * @brief SDPE Auto-Generated Component Header
 * @note Generated from: inst_volt_div_800V.json
 * @note Using Paradigm: voltage_divider.json
 * @note WARNING: DO NOT MODIFY THIS FILE MANUALLY. EDIT THE SOURCE JSON INSTEAD.
 * @note Command: python sdpe_cli.py -i instances/inst_volt_div_800V.json -t paradigms/voltage_divider.json
 */

#ifndef _FILE_SDPE_CTRL_DCBUS_VOLTAGE_UDC_H_
#define _FILE_SDPE_CTRL_DCBUS_VOLTAGE_UDC_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/* --- Resistor Voltage Divider Sensor (udc) Parameters --- */


/**
 * @brief High-side resistor value
 * @unit Ohms
 */
#define CTRL_DCBUS_VOLTAGE_R_HIGH_OHM           ((3000000.0f))


/**
 * @brief Low-side resistor value
 * @unit Ohms
 */
#define CTRL_DCBUS_VOLTAGE_R_LOW_OHM            ((10000.0f))


/**
 * @brief Buffer or amplifier gain after divider
 * @unit V/V
 */
#define CTRL_DCBUS_VOLTAGE_AMP_GAIN             ((1.0f))


/**
 * @brief Hardware bias voltage before ADC
 * @unit V
 */
#define CTRL_DCBUS_VOLTAGE_BIAS_V               ((0.0f))


/**
 * @brief Maximum measurable continuous voltage
 * @unit V
 */
#define CTRL_DCBUS_VOLTAGE_MAX_VOLTAGE_V        ((1000.0f))


/**
 * @brief Overall accuracy class
 * @unit %
 */
#define CTRL_DCBUS_VOLTAGE_ACCURACY_CLASS_PCT   ((1.0f))



/* --- Derived Sensitivity and Bias Macros --- */

/**
 * @brief Calculated hardware sensitivity for ADC scaling
 * @unit V/V
 */
#define CTRL_DCBUS_VOLTAGE_SENSITIVITY         (GMP_SDPE_CALC_VOLTAGE_DIVIDER_SENSITIVITY(CTRL_DCBUS_VOLTAGE_R_HIGH_OHM, CTRL_DCBUS_VOLTAGE_R_LOW_OHM, CTRL_DCBUS_VOLTAGE_AMP_GAIN))

/**
 * @brief Hardware bias voltage for ADC offset
 * @unit V
 */
#define CTRL_DCBUS_VOLTAGE_BIAS                (GMP_SDPE_CALC_BIAS(CTRL_DCBUS_VOLTAGE_BIAS_V))

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SDPE_CTRL_DCBUS_VOLTAGE_UDC_H_
