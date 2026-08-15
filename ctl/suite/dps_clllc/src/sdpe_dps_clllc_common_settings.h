/**
 * @file sdpe_dps_clllc_common_settings.h
 * @brief SDPE project bindings for DPS CLLLC / DAB Common Control.
 * @note Platform-independent Dioscuri CLLLC/DAB tank, sensing, modulation and control-loop contract.
 */

#ifndef _PROJECT_SDPE_DPS_CLLLC_COMMON_SETTINGS_H_
#define _PROJECT_SDPE_DPS_CLLLC_COMMON_SETTINGS_H_

#include <ctl/hardware_preset/current_sensor/tmcs1133_b5a.h>
#include <ctl/hardware_preset/resonant_tank/dioscuri_clllc_resonant_tank.h>
#include <ctl/hardware_preset/voltage_sensor/dioscuri_voltage_sensor.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Shared CLLLC/DAB control and physical hardware contract. Floating-point physical ranges are constrained by their SDPE parameter definitions rather than non-portable preprocessor arithmetic. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define DPS_CLLLC_COMMON_SDPE_PROJECT_ID "dps_clllc_common"
#define DPS_CLLLC_COMMON_SDPE_PROJECT_SUITE "dps_clllc"
#define DPS_CLLLC_COMMON_SDPE_PROJECT_VERSION "1.0.0"
#define DPS_CLLLC_COMMON_SDPE_PROJECT_UPDATED_AT "2026-08-15"

//=================================================================================================
/**
 * @brief Sensing.
 */

/**
 * @brief Calibrate the TMCS1133B5A zero-current bias while all PWM outputs are disabled.
 */
#define SPECIFY_ENABLE_ADC_CALIBRATE

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Enable the five ADC slots used by the CLLLC SIL input ABI.
 */
#ifndef GMP_PIL_RX_MASK
#define GMP_PIL_RX_MASK (31)
#endif // GMP_PIL_RX_MASK

/**
 * @brief Enable eight modulation slots and seven monitor slots used by the CLLLC SIL output ABI.
 */
#ifndef GMP_PIL_TX_MASK
#define GMP_PIL_TX_MASK (8323327)
#endif // GMP_PIL_TX_MASK

/**
 * @brief Nominal tank resonant frequency.
 */
#define CLLLC_F_RESONANT_HZ DIOSCURI_CLLLC_TANK_RESONANT_FREQUENCY_HZ

/**
 * @brief Lowest allowed hybrid modulation frequency.
 */
#define CLLLC_F_MIN_HZ real2param(75000.0)

/**
 * @brief Highest allowed hybrid modulation frequency.
 */
#define CLLLC_F_MAX_HZ real2param(150000.0)

/**
 * @brief Absolute complementary-switch dead time.
 */
#define CLLLC_DEADBAND_S real2param(200e-9)

/**
 * @brief Maximum signed primary-to-secondary bridge phase displacement, where 1 pu is 360 degrees.
 */
#define CLLLC_MAX_PHASE_SHIFT_PU real2param(0.25)

/**
 * @brief Transformer magnetizing inductance.
 */
#define CLLLC_LM_H DIOSCURI_CLLLC_TANK_MAGNETIZING_INDUCTANCE_H

/**
 * @brief Primary resonant inductance.
 */
#define CLLLC_LR_PRIMARY_H DIOSCURI_CLLLC_TANK_PRIMARY_RESONANT_INDUCTANCE_H

/**
 * @brief Secondary resonant inductance.
 */
#define CLLLC_LR_SECONDARY_H DIOSCURI_CLLLC_TANK_SECONDARY_RESONANT_INDUCTANCE_H

/**
 * @brief Primary resonant capacitance.
 */
#define CLLLC_CR_PRIMARY_F DIOSCURI_CLLLC_TANK_PRIMARY_RESONANT_CAPACITANCE_F

/**
 * @brief Secondary resonant capacitance.
 */
#define CLLLC_CR_SECONDARY_F DIOSCURI_CLLLC_TANK_SECONDARY_RESONANT_CAPACITANCE_F

/**
 * @brief Secondary-to-primary turns ratio.
 */
#define CLLLC_TRANSFORMER_NS_NP DIOSCURI_CLLLC_TANK_TRANSFORMER_RATIO_NS_NP

/**
 * @brief Equivalent output capacitance.
 */
#define CLLLC_COUT_F DIOSCURI_CLLLC_TANK_OUTPUT_CAPACITANCE_F

/**
 * @brief Minimum resistive load used for conservative tuning.
 */
#define CLLLC_RLOAD_MIN_OHM DIOSCURI_CLLLC_TANK_MINIMUM_LOAD_RESISTANCE_OHM

/**
 * @brief Equivalent resonant-tank series resistance.
 */
#define CLLLC_TANK_ESR_OHM DIOSCURI_CLLLC_TANK_TANK_ESR_OHM

/**
 * @brief Voltage per-unit base.
 */
#define CTRL_VOLTAGE_BASE real2param(120.0)

/**
 * @brief Current per-unit base, kept within the TMCS1133B5A rated measurement range.
 */
#define CTRL_CURRENT_BASE real2param(10.0)

/**
 * @brief AMC1311 board-level ADC sensitivity.
 */
#define CLLLC_VOLTAGE_SENSITIVITY_V_PER_V DIOSCURI_VOLTAGE_SENSOR_SENSITIVITY_V_PER_V

/**
 * @brief Unidirectional AMC1311 board-level ADC bias.
 */
#define CLLLC_VOLTAGE_BIAS_V DIOSCURI_VOLTAGE_SENSOR_BIAS_V

/**
 * @brief AMC1311 front-end full-scale voltage.
 */
#define CLLLC_VOLTAGE_SENSOR_RANGE_V DIOSCURI_VOLTAGE_SENSOR_RATED_VOLTAGE_V

/**
 * @brief TMCS1133B5A sensitivity.
 */
#define CLLLC_CURRENT_SENSITIVITY_V_PER_A TMCS1133_B5A_SENSITIVITY_V_PER_A

/**
 * @brief TMCS1133B5A zero-current output bias.
 */
#define CLLLC_CURRENT_BIAS_V TMCS1133_B5A_BIAS_V

/**
 * @brief TMCS1133B5A rated current range.
 */
#define CLLLC_CURRENT_SENSOR_RANGE_A TMCS1133_B5A_RANGE_A

/**
 * @brief Default secondary voltage reference.
 */
#define CLLLC_VOLTAGE_TARGET_PU real2param(0.40)

/**
 * @brief Current-loop reference and parallel current limit.
 */
#define CLLLC_CURRENT_TARGET_PU real2param(0.50)

/**
 * @brief Requested current-loop bandwidth, constrained by CLLLC auto tuning.
 */
#define CLLLC_CURRENT_LOOP_BW_HZ real2param(5000.0)

/**
 * @brief Requested voltage-loop bandwidth, constrained below the current loop.
 */
#define CLLLC_VOLTAGE_LOOP_BW_HZ real2param(400.0)

/**
 * @brief Voltage-reference slew rate.
 */
#define CLLLC_VOLTAGE_SLOPE_PU_S real2param(0.5)

/**
 * @brief Current-reference slew rate.
 */
#define CLLLC_CURRENT_SLOPE_PU_S real2param(1.0)

/**
 * @brief Maximum current-sensor offset calibration time.
 */
#define TIMEOUT_ADC_CALIB_MS (3000)

// User project tail code
// SDPE extension point: add before_footer code in the Project Requirement Code page if needed.

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_DPS_CLLLC_COMMON_SETTINGS_H_
