/**
 * @file
 * @brief SDPE Auto-Generated Component Header
 * @note Generated from: inst_shunt_test.json
 * @note Using Paradigm: current_shunt.json
 * @note WARNING: DO NOT MODIFY THIS FILE MANUALLY. EDIT THE SOURCE JSON INSTEAD.
 * @note Command: python sdpe_cli.py -i inst_shunt_test.json -t paradigms/current_shunt.json
 */

#ifndef _FILE_SDPE_CTRL_INVERTER_CURRENT_IUVW_H_
#define _FILE_SDPE_CTRL_INVERTER_CURRENT_IUVW_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/* --- Shunt-based Current Sensor with Op-Amp (iuvw) Parameters --- */


/**
 * @brief Shunt resistor value
 * @unit Ohms
 */
#define CTRL_INVERTER_CURRENT_R_SHUNT_OHM          ((0.002f))


/**
 * @brief Total amplifier gain
 * @unit V/V
 */
#define CTRL_INVERTER_CURRENT_AMP_GAIN             ((20.0f))


/**
 * @brief Hardware bias voltage before ADC
 * @unit V
 */
#define CTRL_INVERTER_CURRENT_BIAS_V               ((1.65f))


/**
 * @brief Maximum continuous current
 * @unit Arms
 */
#define CTRL_INVERTER_CURRENT_MAX_CONT_ARMS        ((15.0f))


/**
 * @brief Maximum peak current
 * @unit Ap
 */
#define CTRL_INVERTER_CURRENT_MAX_PEAK_AP          ((30.0f))


/**
 * @brief Overall accuracy class
 * @unit %
 */
#define CTRL_INVERTER_CURRENT_ACCURACY_CLASS_PCT   ((0.5f))


/**
 * @brief Main amplifier or CSA chip part number
 */
#define CTRL_INVERTER_CURRENT_CHIP_PN              "INA240A1"



/* --- Derived Sensitivity and Bias Macros --- */

/**
 * @brief Calculated hardware sensitivity for ADC scaling
 * @unit V/A
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY         (GMP_SDPE_CALC_SHUNT_SENSITIVITY(CTRL_INVERTER_CURRENT_R_SHUNT_OHM, CTRL_INVERTER_CURRENT_AMP_GAIN))

/**
 * @brief Hardware bias voltage for ADC offset
 * @unit V
 */
#define CTRL_INVERTER_CURRENT_BIAS                (GMP_SDPE_CALC_BIAS(CTRL_INVERTER_CURRENT_BIAS_V))

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SDPE_CTRL_INVERTER_CURRENT_IUVW_H_
