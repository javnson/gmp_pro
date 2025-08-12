/**
 * @file    steval_ihm023v3.h
 * @brief   Hardware Abstraction Layer (HAL) for the ST STEVAL-IHM023V3 evaluation board.
 * @version 1.0
 * @date    2025-08-12
 * @author  Gemini (based on user template and ST manual um1823)
 * @note    This file contains the hardware parameters for the STMicroelectronics
 * STEVAL-IHM023V3 motor control evaluation board. The values are derived
 * from the official user manual (um1823).
 * By defining 'STEVAL_IHM023V3_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/motor_control/controller_preset/controller_preset_general.h>

#ifndef MOTOR_DRIVER_HAL_STEVAL_IHM023V3_H
#define MOTOR_DRIVER_HAL_STEVAL_IHM023V3_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_steval_ihm023v3 STEVAL-IHM023V3 Hardware Abstraction Layer
 * @brief Parameters for the ST STEVAL-IHM023V3 motor driver board.
 * @details reference https://www.st.com/en/evaluation-tools/steval-ihm023v3.html
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_steval_ihm023v3_nameplate Nameplate
 * @ingroup hal_steval_ihm023v3
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define STEVAL_IHM023V3_NAME                 "ST STEVAL-IHM023V3"
#define STEVAL_IHM023V3_GATE_DRIVER_IC       "L6390"
#define STEVAL_IHM023V3_MOSFET_PART_NUMBER   "STGP10H60DF (IGBT)"
#define STEVAL_IHM023V3_CURRENT_SENSOR_MODEL "Low-side Shunt + L6390 Internal OPAMP"
#define STEVAL_IHM023V3_THERMAL_SENSOR_MODEL "NTC on heatsink"

/** @} */ // end of hal_steval_ihm023v3_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_steval_ihm023v3_limits Physical Operating Limits
 * @ingroup hal_steval_ihm023v3
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define STEVAL_IHM023V3_VBUS_MIN_V                                                                                     \
    (125.0f) // Minimum DC bus operating voltage (V), per manual section 1.1 (High Voltage DC)
#define STEVAL_IHM023V3_VBUS_MAX_V                                                                                     \
    (400.0f) // Maximum DC bus operating voltage (V), per manual section 1.1 (High Voltage DC)
#define STEVAL_IHM023V3_CURRENT_MAX_RMS_A (4.6f) // Maximum continuous phase current (RMS, A), derived from peak.
#define STEVAL_IHM023V3_CURRENT_MAX_PEAK_A                                                                             \
    (6.5f)                                 // Maximum allowed peak phase current (Peak, A), per manual section 2.3.6
#define STEVAL_IHM023V3_TEMP_MAX_C (70.0f) // Overtemperature shutdown at 70°„C, per manual section 2.3.8

/** @} */ // end of hal_steval_ihm023v3_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_steval_ihm023v3_topology Sensing Topology
 * @ingroup hal_steval_ihm023v3
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define STEVAL_IHM023V3_PH_CURRENT_SENSE_TYPE     (1) // SENSOR_TYPE_SHUNT (Three-shunt and single-shunt configurable)
#define STEVAL_IHM023V3_PH_CURRENT_SENSE_TOPOLOGY (1) // CS_TOPOLOGY_LOW_SIDE (Low-side current sensing)
#define STEVAL_IHM023V3_PH_VOLTAGE_SENSE_TYPE     (0) // VS_TYPE_NONE (No direct phase voltage sensing for FOC)
#define STEVAL_IHM023V3_DCBUS_VOLTAGE_SENSE_TYPE  (1) // VS_TYPE_PHASE_GND (Resistive divider for Vbus)
#define STEVAL_IHM023V3_DCBUS_CURRENT_SENSE_TYPE  (0) // SENSOR_NONE (No DC bus current sensor present)
#define STEVAL_IHM023V3_THERMAL_SENSE_TYPE        (1) // THERMAL_SENSOR_NTC (NTC on heatsink)

/** @} */ // end of hal_steval_ihm023v3_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_steval_ihm023v3_circuits Sensing Circuit Parameters
 * @ingroup hal_steval_ihm023v3
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
#if (STEVAL_IHM023V3_PH_CURRENT_SENSE_TYPE == 1) // SENSOR_TYPE_SHUNT
#define STEVAL_IHM023V3_PH_SHUNT_RESISTANCE_OHM                                                                        \
    (0.15f)                                    // Resistance of the shunt resistor (Ohm), per FOC SDK parameters table.
#define STEVAL_IHM023V3_PH_CSA_GAIN_V_V (1.7f) // Amplifying network gain, per FOC SDK parameters table.
#define STEVAL_IHM023V3_PH_CSA_BIAS_V   (1.7f) // Bias voltage, per manual section 2.3.6
#endif
#define STEVAL_IHM023V3_PH_CURRENT_SENSE_POLE_HZ (1000.0f) // Assumed filter bandwidth.

//--- 4.2: Phase Voltage Sensing ---
#if (STEVAL_IHM023V3_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
// Not applicable for this board
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (STEVAL_IHM023V3_DCBUS_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define STEVAL_IHM023V3_DCBUS_VOLTAGE_SENSE_GAIN                                                                       \
    (0.0075f) // Gain of the voltage sensing circuit (V/V), per manual section 2.3.1
#define STEVAL_IHM023V3_DCBUS_VOLTAGE_SENSE_BIAS_V  (0.0f)    // Bias of the voltage sensing circuit (V).
#define STEVAL_IHM023V3_DCBUS_VOLTAGE_SENSE_POLE_HZ (1000.0f) // Assumed filter bandwidth.
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (STEVAL_IHM023V3_DCBUS_CURRENT_SENSE_TYPE != 0)
// Not applicable for this board
#endif

//--- 4.5: Thermal Sensing ---
#if (STEVAL_IHM023V3_THERMAL_SENSE_TYPE == 1)                   // THERMAL_SENSOR_NTC
#define STEVAL_IHM023V3_THERMAL_PULLUP_RESISTANCE_OHM (3600.0f) // Pull-up resistor R44 (Ohm).
#define STEVAL_IHM023V3_THERMAL_NTC_BETA_VALUE        (3950.0f) // Assumed Beta value for typical 10k NTC.
#define STEVAL_IHM023V3_THERMAL_NTC_NOMINAL_R_OHM                                                                      \
    (10000.0f) // Nominal resistance of NTC RT1 at nominal temperature (Ohm).
#define STEVAL_IHM023V3_THERMAL_NTC_NOMINAL_TEMP_C (25.0f) // Nominal temperature (°„C).
#endif

/** @} */ // end of hal_steval_ihm023v3_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_steval_ihm023v3_mapping Default Parameter Mapping
 * @ingroup hal_steval_ihm023v3
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(STEVAL_IHM023V3_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 STEVAL_IHM023V3_NAME
#define MY_BOARD_GATE_DRIVER_IC       STEVAL_IHM023V3_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   STEVAL_IHM023V3_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL STEVAL_IHM023V3_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL STEVAL_IHM023V3_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         STEVAL_IHM023V3_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         STEVAL_IHM023V3_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  STEVAL_IHM023V3_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A STEVAL_IHM023V3_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         STEVAL_IHM023V3_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     STEVAL_IHM023V3_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY STEVAL_IHM023V3_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     STEVAL_IHM023V3_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  STEVAL_IHM023V3_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  STEVAL_IHM023V3_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        STEVAL_IHM023V3_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_SHUNT_RESISTANCE_OHM       STEVAL_IHM023V3_PH_SHUNT_RESISTANCE_OHM
#define MY_BOARD_PH_CSA_GAIN_V_V               STEVAL_IHM023V3_PH_CSA_GAIN_V_V
#define MY_BOARD_PH_CSA_BIAS_V                 STEVAL_IHM023V3_PH_CSA_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ      STEVAL_IHM023V3_PH_CURRENT_SENSE_POLE_HZ
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN         STEVAL_IHM023V3_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V       STEVAL_IHM023V3_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ      STEVAL_IHM023V3_PH_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN      STEVAL_IHM023V3_DCBUS_VOLTAGE_SENSE_GAIN
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V    STEVAL_IHM023V3_DCBUS_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ   STEVAL_IHM023V3_DCBUS_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_THERMAL_PULLUP_RESISTANCE_OHM STEVAL_IHM023V3_THERMAL_PULLUP_RESISTANCE_OHM
#define MY_BOARD_THERMAL_NTC_BETA_VALUE        STEVAL_IHM023V3_THERMAL_NTC_BETA_VALUE
#define MY_BOARD_THERMAL_NTC_NOMINAL_R_OHM     STEVAL_IHM023V3_THERMAL_NTC_NOMINAL_R_OHM
#define MY_BOARD_THERMAL_NTC_NOMINAL_TEMP_C    STEVAL_IHM023V3_THERMAL_NTC_NOMINAL_TEMP_C

#endif // STEVAL_IHM023V3_IS_DEFAULT_PARAM

/** @} */ // end of hal_steval_ihm023v3_mapping
//=================================================================================================

/** @} */ // end of hal_steval_ihm023v3

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_STEVAL_IHM023V3_H
