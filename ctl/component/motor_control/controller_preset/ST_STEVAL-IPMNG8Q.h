/**
 * @file    steval_ipmng8q.h
 * @brief   Hardware Abstraction Layer (HAL) for the ST STEVAL-IPMNG8Q evaluation board.
 * @version 1.0
 * @date    2025-08-12
 * @author  Gemini (based on user template and ST manual um2280)
 * @note    This file contains the hardware parameters for the STMicroelectronics
 * STEVAL-IPMNG8Q motor control power board. The values are derived
 * from the official user manual (um2280).
 * By defining 'STEVAL_IPMNG8Q_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/motor_control/controller_preset/controller_preset_general.h>

#ifndef MOTOR_DRIVER_HAL_STEVAL_IPMNG8Q_H
#define MOTOR_DRIVER_HAL_STEVAL_IPMNG8Q_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_steval_ipmng8q STEVAL-IPMNG8Q Hardware Abstraction Layer
 * @brief Parameters for the ST STEVAL-IPMNG8Q motor driver board.
 * @details reference https://www.st.com/en/evaluation-tools/steval-ipmng8q.html
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_steval_ipmng8q_nameplate Nameplate
 * @ingroup hal_steval_ipmng8q
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define STEVAL_IPMNG8Q_NAME                 "ST STEVAL-IPMNG8Q"
#define STEVAL_IPMNG8Q_GATE_DRIVER_IC       "Integrated in STGIPQ8C60T-HZ"
#define STEVAL_IPMNG8Q_MOSFET_PART_NUMBER   "STGIPQ8C60T-HZ (SLLIMM-nano 2nd series IPM)"
#define STEVAL_IPMNG8Q_CURRENT_SENSOR_MODEL "Low-side Shunt + TSV994 OPAMP / Internal IPM OPAMP"
#define STEVAL_IPMNG8Q_THERMAL_SENSOR_MODEL "IPM internal NTC"

/** @} */ // end of hal_steval_ipmng8q_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_steval_ipmng8q_limits Physical Operating Limits
 * @ingroup hal_steval_ipmng8q
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define STEVAL_IPMNG8Q_VBUS_MIN_V        (125.0f) // Minimum DC bus operating voltage (V), per manual section 1
#define STEVAL_IPMNG8Q_VBUS_MAX_V        (400.0f) // Maximum DC bus operating voltage (V), per manual section 1
#define STEVAL_IPMNG8Q_CURRENT_MAX_RMS_A (4.8f)   // Maximum continuous phase current (RMS, A), per manual section 1
#define STEVAL_IPMNG8Q_CURRENT_MAX_PEAK_A                                                                              \
    (10.2f)                                // Maximum allowed peak phase current (Peak, A), per manual section 4.3.2
#define STEVAL_IPMNG8Q_TEMP_MAX_C (100.0f) // Typical max operating temperature for IPM module.

/** @} */ // end of hal_steval_ipmng8q_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_steval_ipmng8q_topology Sensing Topology
 * @ingroup hal_steval_ipmng8q
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define STEVAL_IPMNG8Q_PH_CURRENT_SENSE_TYPE     (1) // SENSOR_TYPE_SHUNT (Single- or three-shunt configurable)
#define STEVAL_IPMNG8Q_PH_CURRENT_SENSE_TOPOLOGY (1) // CS_TOPOLOGY_LOW_SIDE (Low-side current sensing)
#define STEVAL_IPMNG8Q_PH_VOLTAGE_SENSE_TYPE     (0) // VS_TYPE_NONE (No direct phase voltage sensing for FOC)
#define STEVAL_IPMNG8Q_DCBUS_VOLTAGE_SENSE_TYPE  (1) // VS_TYPE_PHASE_GND (Resistive divider for Vbus)
#define STEVAL_IPMNG8Q_DCBUS_CURRENT_SENSE_TYPE  (0) // SENSOR_NONE (No DC bus current sensor present)
#define STEVAL_IPMNG8Q_THERMAL_SENSE_TYPE        (1) // THERMAL_SENSOR_NTC (Embedded in IPM)

/** @} */ // end of hal_steval_ipmng8q_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_steval_ipmng8q_circuits Sensing Circuit Parameters
 * @ingroup hal_steval_ipmng8q
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
#if (STEVAL_IPMNG8Q_PH_CURRENT_SENSE_TYPE == 1)         // SENSOR_TYPE_SHUNT
#define STEVAL_IPMNG8Q_PH_SHUNT_RESISTANCE_OHM (0.082f) // Resistance of the shunt resistor (Ohm), per manual table 1.
#define STEVAL_IPMNG8Q_PH_CSA_GAIN_V_V         (1.9f)   // Amplifying network gain, per manual section 5.
#define STEVAL_IPMNG8Q_PH_CSA_BIAS_V           (1.65f)  // Bias is VREF/2 (3.3V / 2).
#endif
#define STEVAL_IPMNG8Q_PH_CURRENT_SENSE_POLE_HZ (482000.0f) // Approx. 1 / (2 * PI * 1k * 330pF) from schematic.

//--- 4.2: Phase Voltage Sensing ---
#if (STEVAL_IPMNG8Q_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
// Not applicable for this board
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (STEVAL_IPMNG8Q_DCBUS_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define STEVAL_IPMNG8Q_DCBUS_VOLTAGE_SENSE_GAIN                                                                        \
    (0.008f) // Gain of the voltage sensing circuit (V/V), 1/125 per FOC SDK parameters table.
#define STEVAL_IPMNG8Q_DCBUS_VOLTAGE_SENSE_BIAS_V  (0.0f)   // Bias of the voltage sensing circuit (V).
#define STEVAL_IPMNG8Q_DCBUS_VOLTAGE_SENSE_POLE_HZ (132.0f) // Approx. 1 / (2 * PI * 120k * 10nF) from schematic.
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (STEVAL_IPMNG8Q_DCBUS_CURRENT_SENSE_TYPE != 0)
// Not applicable for this board
#endif

//--- 4.5: Thermal Sensing ---
#if (STEVAL_IPMNG8Q_THERMAL_SENSE_TYPE == 1)                   // THERMAL_SENSOR_NTC
#define STEVAL_IPMNG8Q_THERMAL_PULLUP_RESISTANCE_OHM (1000.0f) // Pull-up resistor R10 (Ohm) from schematic.
#define STEVAL_IPMNG8Q_THERMAL_NTC_BETA_VALUE        (3950.0f) // Assumed Beta value for typical NTC.
#define STEVAL_IPMNG8Q_THERMAL_NTC_NOMINAL_R_OHM                                                                       \
    (85000.0f) // Nominal resistance of NTC at 25°„C (Ohm), per manual section 6.1.
#define STEVAL_IPMNG8Q_THERMAL_NTC_NOMINAL_TEMP_C (25.0f) // Nominal temperature (°„C).
#endif

/** @} */ // end of hal_steval_ipmng8q_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_steval_ipmng8q_mapping Default Parameter Mapping
 * @ingroup hal_steval_ipmng8q
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(STEVAL_IPMNG8Q_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 STEVAL_IPMNG8Q_NAME
#define MY_BOARD_GATE_DRIVER_IC       STEVAL_IPMNG8Q_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   STEVAL_IPMNG8Q_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL STEVAL_IPMNG8Q_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL STEVAL_IPMNG8Q_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         STEVAL_IPMNG8Q_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         STEVAL_IPMNG8Q_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  STEVAL_IPMNG8Q_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A STEVAL_IPMNG8Q_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         STEVAL_IPMNG8Q_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     STEVAL_IPMNG8Q_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY STEVAL_IPMNG8Q_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     STEVAL_IPMNG8Q_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  STEVAL_IPMNG8Q_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  STEVAL_IPMNG8Q_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        STEVAL_IPMNG8Q_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_SHUNT_RESISTANCE_OHM       STEVAL_IPMNG8Q_PH_SHUNT_RESISTANCE_OHM
#define MY_BOARD_PH_CSA_GAIN_V_V               STEVAL_IPMNG8Q_PH_CSA_GAIN_V_V
#define MY_BOARD_PH_CSA_BIAS_V                 STEVAL_IPMNG8Q_PH_CSA_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ      STEVAL_IPMNG8Q_PH_CURRENT_SENSE_POLE_HZ
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN         STEVAL_IPMNG8Q_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V       STEVAL_IPMNG8Q_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ      STEVAL_IPMNG8Q_PH_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN      STEVAL_IPMNG8Q_DCBUS_VOLTAGE_SENSE_GAIN
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V    STEVAL_IPMNG8Q_DCBUS_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ   STEVAL_IPMNG8Q_DCBUS_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_THERMAL_PULLUP_RESISTANCE_OHM STEVAL_IPMNG8Q_THERMAL_PULLUP_RESISTANCE_OHM
#define MY_BOARD_THERMAL_NTC_BETA_VALUE        STEVAL_IPMNG8Q_THERMAL_NTC_BETA_VALUE
#define MY_BOARD_THERMAL_NTC_NOMINAL_R_OHM     STEVAL_IPMNG8Q_THERMAL_NTC_NOMINAL_R_OHM
#define MY_BOARD_THERMAL_NTC_NOMINAL_TEMP_C    STEVAL_IPMNG8Q_THERMAL_NTC_NOMINAL_TEMP_C

#endif // STEVAL_IPMNG8Q_IS_DEFAULT_PARAM

/** @} */ // end of hal_steval_ipmng8q_mapping
//=================================================================================================

/** @} */ // end of hal_steval_ipmng8q

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_STEVAL_IPMNG8Q_H
