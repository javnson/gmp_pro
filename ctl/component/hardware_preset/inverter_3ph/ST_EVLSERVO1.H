/**
 * @file    evlservo1.h
 * @brief   Hardware Abstraction Layer (HAL) for the ST EVLSERVO1 evaluation board.
 * @version 1.0
 * @date    2025-08-12
 * @author  Gemini (based on user template and ST manual um3375)
 * @note    This file contains the hardware parameters for the STMicroelectronics
 * EVLSERVO1 servo motor evaluation board. The values are derived
 * from the official user manual (um3375).
 * By defining 'EVLSERVO1_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/hardware_preset/inverter_3ph/inverter_3ph_general.h>

#ifndef MOTOR_DRIVER_HAL_EVLSERVO1_H
#define MOTOR_DRIVER_HAL_EVLSERVO1_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_evlservo1 EVLSERVO1 Hardware Abstraction Layer
 * @ingroup CTL_MC_PRESET
 * @brief Parameters for the ST EVLSERVO1 servo motor driver board.
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_evlservo1_nameplate Nameplate
 * @ingroup hal_evlservo1
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define EVLSERVO1_NAME                 "ST EVLSERVO1"
#define EVLSERVO1_GATE_DRIVER_IC       "STSPIN32G4 (System-in-Package)"
#define EVLSERVO1_MOSFET_PART_NUMBER   "STL160N10F8"
#define EVLSERVO1_CURRENT_SENSOR_MODEL "Low-side Shunt + TSV991AIQ OPAMP"
#define EVLSERVO1_THERMAL_SENSOR_MODEL "NTC on board"

/** @} */ // end of hal_evlservo1_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_evlservo1_limits Physical Operating Limits
 * @ingroup hal_evlservo1
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define EVLSERVO1_VBUS_MIN_V (10.0f) // Minimum DC bus operating voltage (V), per manual table 2
#define EVLSERVO1_VBUS_MAX_V (48.0f) // Maximum DC bus operating voltage (V), per manual table 2
#define EVLSERVO1_CURRENT_MAX_RMS_A                                                                                    \
    (42.0f) // Maximum continuous phase current (RMS, A) with passive cooling, per manual table 2
#define EVLSERVO1_CURRENT_MAX_PEAK_A (90.0f) // Maximum allowed peak phase current (Peak, A), per manual table 2
#define EVLSERVO1_TEMP_MAX_C         (85.0f) // Recommended maximum PCB operating temperature (°„C), safe default

/** @} */ // end of hal_evlservo1_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_evlservo1_topology Sensing Topology
 * @ingroup hal_evlservo1
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define EVLSERVO1_PH_CURRENT_SENSE_TYPE     (1) // SENSOR_TYPE_SHUNT (Triple-shunt topology)
#define EVLSERVO1_PH_CURRENT_SENSE_TOPOLOGY (1) // CS_TOPOLOGY_LOW_SIDE (Low-side current sensing)
#define EVLSERVO1_PH_VOLTAGE_SENSE_TYPE     (0) // VS_TYPE_NONE (No direct phase voltage sensing for FOC)
#define EVLSERVO1_DCBUS_VOLTAGE_SENSE_TYPE  (1) // VS_TYPE_PHASE_GND (Resistive divider for Vbus)
#define EVLSERVO1_DCBUS_CURRENT_SENSE_TYPE  (0) // SENSOR_NONE (No DC bus current sensor present)
#define EVLSERVO1_THERMAL_SENSE_TYPE        (1) // THERMAL_SENSOR_NTC (NTC on board)

/** @} */ // end of hal_evlservo1_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_evlservo1_circuits Sensing Circuit Parameters
 * @ingroup hal_evlservo1
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
#if (EVLSERVO1_PH_CURRENT_SENSE_TYPE == 1) // SENSOR_TYPE_SHUNT
#define EVLSERVO1_PH_SHUNT_RESISTANCE_OHM                                                                              \
    (0.0005f)                             // Resistance of the shunt resistor (500 uOhm), per manual section 5.5
#define EVLSERVO1_PH_CSA_GAIN_V_V (20.0f) // Amplifying network gain, per manual section 5.5
#define EVLSERVO1_PH_CSA_BIAS_V   (1.65f) // Bias is VREF+/2 (3.3V / 2).
#endif
#define EVLSERVO1_PH_CURRENT_SENSE_POLE_HZ (94000.0f) // Approx. 1 / (2 * PI * 51k * 33nF) from schematic.

//--- 4.2: Phase Voltage Sensing ---
#if (EVLSERVO1_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
// Not applicable for this board
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (EVLSERVO1_DCBUS_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define EVLSERVO1_DCBUS_VOLTAGE_SENSE_GAIN                                                                             \
    (0.04f) // Gain of the voltage sensing circuit (V/V), attenuation is 0.04 per manual section 5.7
#define EVLSERVO1_DCBUS_VOLTAGE_SENSE_BIAS_V  (0.0f)     // Bias of the voltage sensing circuit (V).
#define EVLSERVO1_DCBUS_VOLTAGE_SENSE_POLE_HZ (55000.0f) // Approx. 1 / (2 * PI * 2.89k * 1nF) from schematic.
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (EVLSERVO1_DCBUS_CURRENT_SENSE_TYPE != 0)
// Not applicable for this board
#endif

//--- 4.5: Thermal Sensing ---
#if (EVLSERVO1_THERMAL_SENSE_TYPE == 1)                   // THERMAL_SENSOR_NTC
#define EVLSERVO1_THERMAL_PULLUP_RESISTANCE_OHM (4700.0f) // Pull-up resistor R52 (Ohm) from schematic.
#define EVLSERVO1_THERMAL_NTC_BETA_VALUE        (3455.0f) // Beta value of the NTC (K), per manual section 5.8.
#define EVLSERVO1_THERMAL_NTC_NOMINAL_R_OHM                                                                            \
    (10000.0f) // Nominal resistance of NTC at nominal temperature (Ohm), per manual section 5.8.
#define EVLSERVO1_THERMAL_NTC_NOMINAL_TEMP_C (25.0f) // Nominal temperature (298K), per manual section 5.8.
#endif

/** @} */ // end of hal_evlservo1_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_evlservo1_mapping Default Parameter Mapping
 * @ingroup hal_evlservo1
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(EVLSERVO1_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 EVLSERVO1_NAME
#define MY_BOARD_GATE_DRIVER_IC       EVLSERVO1_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   EVLSERVO1_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL EVLSERVO1_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL EVLSERVO1_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         EVLSERVO1_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         EVLSERVO1_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  EVLSERVO1_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A EVLSERVO1_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         EVLSERVO1_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     EVLSERVO1_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY EVLSERVO1_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     EVLSERVO1_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  EVLSERVO1_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  EVLSERVO1_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        EVLSERVO1_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_SHUNT_RESISTANCE_OHM       EVLSERVO1_PH_SHUNT_RESISTANCE_OHM
#define MY_BOARD_PH_CSA_GAIN_V_V               EVLSERVO1_PH_CSA_GAIN_V_V
#define MY_BOARD_PH_CSA_BIAS_V                 EVLSERVO1_PH_CSA_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ      EVLSERVO1_PH_CURRENT_SENSE_POLE_HZ
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN         EVLSERVO1_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V       EVLSERVO1_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ      EVLSERVO1_PH_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN      EVLSERVO1_DCBUS_VOLTAGE_SENSE_GAIN
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V    EVLSERVO1_DCBUS_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ   EVLSERVO1_DCBUS_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_THERMAL_PULLUP_RESISTANCE_OHM EVLSERVO1_THERMAL_PULLUP_RESISTANCE_OHM
#define MY_BOARD_THERMAL_NTC_BETA_VALUE        EVLSERVO1_THERMAL_NTC_BETA_VALUE
#define MY_BOARD_THERMAL_NTC_NOMINAL_R_OHM     EVLSERVO1_THERMAL_NTC_NOMINAL_R_OHM
#define MY_BOARD_THERMAL_NTC_NOMINAL_TEMP_C    EVLSERVO1_THERMAL_NTC_NOMINAL_TEMP_C

#endif // EVLSERVO1_IS_DEFAULT_PARAM

/** @} */ // end of hal_evlservo1_mapping
//=================================================================================================

/** @} */ // end of hal_evlservo1

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_EVLSERVO1_H
