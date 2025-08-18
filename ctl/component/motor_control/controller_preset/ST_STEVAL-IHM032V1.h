/**
 * @file    steval_ihm032v1.h
 * @brief   Hardware Abstraction Layer (HAL) for the ST STEVAL-IHM032V1 evaluation board.
 * @version 1.0
 * @date    2025-08-12
 * @author  Gemini (based on user template and ST manual um1078)
 * @note    This file contains the hardware parameters for the STMicroelectronics
 * STEVAL-IHM032V1 motor control power board. The values are derived
 * from the official user manual (um1078).
 * By defining 'STEVAL_IHM032V1_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/motor_control/controller_preset/controller_preset_general.h>

#ifndef MOTOR_DRIVER_HAL_STEVAL_IHM032V1_H
#define MOTOR_DRIVER_HAL_STEVAL_IHM032V1_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_steval_ihm032v1 STEVAL-IHM032V1 Hardware Abstraction Layer
 * @ingroup CTL_MC_PRESET
 * @brief Parameters for the ST STEVAL-IHM032V1 motor driver board.
 * @details https://www.st.com/en/evaluation-tools/steval-ihm032v1.html
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_steval_ihm032v1_nameplate Nameplate
 * @ingroup hal_steval_ihm032v1
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define STEVAL_IHM032V1_NAME                 "ST STEVAL-IHM032V1"
#define STEVAL_IHM032V1_GATE_DRIVER_IC       "L639x (L6391/L6392)"
#define STEVAL_IHM032V1_MOSFET_PART_NUMBER   "STGD3HF60HD (IGBT)"
#define STEVAL_IHM032V1_CURRENT_SENSOR_MODEL "Single-Shunt + L6392 Internal OPAMP"
#define STEVAL_IHM032V1_THERMAL_SENSOR_MODEL "NTC on board"

/** @} */ // end of hal_steval_ihm032v1_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_steval_ihm032v1_limits Physical Operating Limits
 * @ingroup hal_steval_ihm032v1
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define STEVAL_IHM032V1_VBUS_MIN_V        (40.0f) // Minimum DC bus operating voltage (V), per FOC SDK parameters table 5.
#define STEVAL_IHM032V1_VBUS_MAX_V        (380.0f) // Maximum DC bus operating voltage (V), per FOC SDK parameters table 5.
#define STEVAL_IHM032V1_CURRENT_MAX_RMS_A (0.86f) // Maximum continuous phase current (RMS, A), derived from peak.
#define STEVAL_IHM032V1_CURRENT_MAX_PEAK_A                                                                             \
    (1.22f) // Maximum allowed peak phase current (Peak, A), per FOC SDK parameters table 5.
#define STEVAL_IHM032V1_TEMP_MAX_C (70.0f) // Max working temperature on sensor, per FOC SDK parameters table 5.

/** @} */ // end of hal_steval_ihm032v1_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_steval_ihm032v1_topology Sensing Topology
 * @ingroup hal_steval_ihm032v1
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define STEVAL_IHM032V1_PH_CURRENT_SENSE_TYPE     (1) // SENSOR_TYPE_SHUNT (Single-shunt topology)
#define STEVAL_IHM032V1_PH_CURRENT_SENSE_TOPOLOGY (2) // CS_TOPOLOGY_SINGLE_SHUNT (DC link current sensing)
#define STEVAL_IHM032V1_PH_VOLTAGE_SENSE_TYPE                                                                          \
    (0) // VS_TYPE_NONE (No direct phase voltage sensing for FOC, BEMF for scalar)
#define STEVAL_IHM032V1_DCBUS_VOLTAGE_SENSE_TYPE (1) // VS_TYPE_PHASE_GND (Resistive divider for Vbus)
#define STEVAL_IHM032V1_DCBUS_CURRENT_SENSE_TYPE (0) // SENSOR_NONE (No DC bus current sensor present)
#define STEVAL_IHM032V1_THERMAL_SENSE_TYPE       (1) // THERMAL_SENSOR_NTC (NTC on board)

/** @} */ // end of hal_steval_ihm032v1_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_steval_ihm032v1_circuits Sensing Circuit Parameters
 * @ingroup hal_steval_ihm032v1
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
#if (STEVAL_IHM032V1_PH_CURRENT_SENSE_TYPE == 1) // SENSOR_TYPE_SHUNT
#define STEVAL_IHM032V1_PH_SHUNT_RESISTANCE_OHM                                                                        \
    (0.45f) // Resistance of the shunt resistor (Ohm), per FOC SDK parameters table 5.
#define STEVAL_IHM032V1_PH_CSA_GAIN_V_V (2.91f) // Amplifying network gain, per FOC SDK parameters table 5.
#define STEVAL_IHM032V1_PH_CSA_BIAS_V   (1.86f) // Bias voltage, per manual section 9.4.
#endif
#define STEVAL_IHM032V1_PH_CURRENT_SENSE_POLE_HZ (1000.0f) // Assumed filter bandwidth.

//--- 4.2: Phase Voltage Sensing ---
#if (STEVAL_IHM032V1_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
// Not applicable for this board for FOC
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (STEVAL_IHM032V1_DCBUS_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define STEVAL_IHM032V1_DCBUS_VOLTAGE_SENSE_GAIN                                                                       \
    (0.008f) // Gain of the voltage sensing circuit (V/V), 1/125 per FOC SDK parameters table 5.
#define STEVAL_IHM032V1_DCBUS_VOLTAGE_SENSE_BIAS_V  (0.0f)    // Bias of the voltage sensing circuit (V).
#define STEVAL_IHM032V1_DCBUS_VOLTAGE_SENSE_POLE_HZ (3386.0f) // Approx. 1 / (2 * PI * 8.2k * 4.7nF) from schematic.
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (STEVAL_IHM032V1_DCBUS_CURRENT_SENSE_TYPE != 0)
// Not applicable for this board
#endif

//--- 4.5: Thermal Sensing ---
#if (STEVAL_IHM032V1_THERMAL_SENSE_TYPE == 1)                   // THERMAL_SENSOR_NTC
#define STEVAL_IHM032V1_THERMAL_PULLUP_RESISTANCE_OHM (4700.0f) // Pull-up resistor R68 (Ohm) from schematic.
#define STEVAL_IHM032V1_THERMAL_NTC_BETA_VALUE        (3950.0f) // Assumed Beta value for typical 10k NTC.
#define STEVAL_IHM032V1_THERMAL_NTC_NOMINAL_R_OHM                                                                      \
    (10000.0f) // Nominal resistance of NTC NTC2 at nominal temperature (Ohm).
#define STEVAL_IHM032V1_THERMAL_NTC_NOMINAL_TEMP_C (25.0f) // Nominal temperature (°„C).
#endif

/** @} */ // end of hal_steval_ihm032v1_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_steval_ihm032v1_mapping Default Parameter Mapping
 * @ingroup hal_steval_ihm032v1
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(STEVAL_IHM032V1_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 STEVAL_IHM032V1_NAME
#define MY_BOARD_GATE_DRIVER_IC       STEVAL_IHM032V1_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   STEVAL_IHM032V1_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL STEVAL_IHM032V1_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL STEVAL_IHM032V1_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         STEVAL_IHM032V1_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         STEVAL_IHM032V1_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  STEVAL_IHM032V1_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A STEVAL_IHM032V1_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         STEVAL_IHM032V1_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     STEVAL_IHM032V1_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY STEVAL_IHM032V1_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     STEVAL_IHM032V1_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  STEVAL_IHM032V1_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  STEVAL_IHM032V1_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        STEVAL_IHM032V1_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_SHUNT_RESISTANCE_OHM       STEVAL_IHM032V1_PH_SHUNT_RESISTANCE_OHM
#define MY_BOARD_PH_CSA_GAIN_V_V               STEVAL_IHM032V1_PH_CSA_GAIN_V_V
#define MY_BOARD_PH_CSA_BIAS_V                 STEVAL_IHM032V1_PH_CSA_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ      STEVAL_IHM032V1_PH_CURRENT_SENSE_POLE_HZ
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN         STEVAL_IHM032V1_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V       STEVAL_IHM032V1_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ      STEVAL_IHM032V1_PH_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN      STEVAL_IHM032V1_DCBUS_VOLTAGE_SENSE_GAIN
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V    STEVAL_IHM032V1_DCBUS_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ   STEVAL_IHM032V1_DCBUS_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_THERMAL_PULLUP_RESISTANCE_OHM STEVAL_IHM032V1_THERMAL_PULLUP_RESISTANCE_OHM
#define MY_BOARD_THERMAL_NTC_BETA_VALUE        STEVAL_IHM032V1_THERMAL_NTC_BETA_VALUE
#define MY_BOARD_THERMAL_NTC_NOMINAL_R_OHM     STEVAL_IHM032V1_THERMAL_NTC_NOMINAL_R_OHM
#define MY_BOARD_THERMAL_NTC_NOMINAL_TEMP_C    STEVAL_IHM032V1_THERMAL_NTC_NOMINAL_TEMP_C

#endif // STEVAL_IHM032V1_IS_DEFAULT_PARAM

/** @} */ // end of hal_steval_ihm032v1_mapping
//=================================================================================================

/** @} */ // end of hal_steval_ihm032v1

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_STEVAL_IHM032V1_H
