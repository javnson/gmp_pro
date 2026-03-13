/**
 * @file    zonri_drv8301_module.h
 * @brief   Hardware Abstraction Layer (HAL) for the ZonRi DRV8301 motor driver module.
 * @version 1.0
 * @date    2025-08-12
 * @author  Gemini (based on user template and ZonRi manual)
 * @note    This file contains the hardware parameters for the ZonRi DRV8301 Motor
 * Drive Module. The values are derived from the official user manual.
 * By defining 'ZONRI_DRV8301_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/hardware_preset/inverter_3ph/inverter_3ph_general.h>

#ifndef MOTOR_DRIVER_HAL_ZONRI_DRV8301_MODULE_H
#define MOTOR_DRIVER_HAL_ZONRI_DRV8301_MODULE_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_zonri_drv8301 ZonRi DRV8301 Module Hardware Abstraction Layer
 * @ingroup CTL_MC_PRESET
 * @brief Parameters for the ZonRi DRV8301 motor driver module.
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_zonri_drv8301_nameplate Nameplate
 * @ingroup hal_zonri_drv8301
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define ZONRI_DRV8301_NAME                 "ZonRi DRV8301 Motor Driver Module"
#define ZONRI_DRV8301_GATE_DRIVER_IC       "DRV8301"
#define ZONRI_DRV8301_MOSFET_PART_NUMBER   "N.A." // Not specified in the manual
#define ZONRI_DRV8301_CURRENT_SENSOR_MODEL "External Three-Phase Differential Amplifiers"
#define ZONRI_DRV8301_THERMAL_SENSOR_MODEL "Internal to DRV8301 (via nFAULT/nOCTW)"

/** @} */ // end of hal_zonri_drv8301_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_zonri_drv8301_limits Physical Operating Limits
 * @ingroup hal_zonri_drv8301
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define ZONRI_DRV8301_VBUS_MIN_V         (5.5f)  // Minimum DC bus operating voltage (V), per manual
#define ZONRI_DRV8301_VBUS_MAX_V         (45.0f) // Maximum DC bus operating voltage (V), per manual
#define ZONRI_DRV8301_CURRENT_MAX_RMS_A  (10.6f) // Maximum continuous phase current (RMS, A), 15A peak / sqrt(2)
#define ZONRI_DRV8301_CURRENT_MAX_PEAK_A (15.0f) // Maximum allowed peak phase current (Peak, A), per manual
#define ZONRI_DRV8301_TEMP_MAX_C         (85.0f) // Recommended maximum PCB operating temperature (°„C), safe default

/** @} */ // end of hal_zonri_drv8301_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_zonri_drv8301_topology Sensing Topology
 * @ingroup hal_zonri_drv8301
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define ZONRI_DRV8301_PH_CURRENT_SENSE_TYPE     (1) // SENSOR_TYPE_SHUNT (External Amplifiers)
#define ZONRI_DRV8301_PH_CURRENT_SENSE_TOPOLOGY (1) // CS_TOPOLOGY_LOW_SIDE (Assumed low-side sensing)
#define ZONRI_DRV8301_PH_VOLTAGE_SENSE_TYPE     (1) // VS_TYPE_PHASE_GND (Resistive divider to GND)
#define ZONRI_DRV8301_DCBUS_VOLTAGE_SENSE_TYPE  (1) // VS_TYPE_PHASE_GND (Resistive divider to GND)
#define ZONRI_DRV8301_DCBUS_CURRENT_SENSE_TYPE  (0) // SENSOR_NONE (No DC bus current sensor present)
#define ZONRI_DRV8301_THERMAL_SENSE_TYPE        (0) // SENSOR_NONE (No direct analog thermal sensor for MCU, only fault pin)

/** @} */ // end of hal_zonri_drv8301_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_zonri_drv8301_circuits Sensing Circuit Parameters
 * @ingroup hal_zonri_drv8301
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
// Note: This board uses external amplifiers. Full scale current is +/- 27A.
#if (ZONRI_DRV8301_PH_CURRENT_SENSE_TYPE == 1) // SENSOR_TYPE_SHUNT
#define ZONRI_DRV8301_PH_SHUNT_RESISTANCE_OHM                                                                          \
    (0.005f)                                   // Resistance of the shunt resistor (Ohm), assumed 5m¶∏ for calculation.
#define ZONRI_DRV8301_PH_CSA_GAIN_V_V (12.22f) // Gain of the Current Sense Amplifier (V/V), per manual.
#define ZONRI_DRV8301_PH_CSA_BIAS_V   (1.65f)  // Bias voltage is 1.65V with internal 3.3V reference.
#endif
#define ZONRI_DRV8301_PH_CURRENT_SENSE_POLE_HZ (150.0e3f) // Using a safe default value.

//--- 4.2: Phase Voltage Sensing ---
#if (ZONRI_DRV8301_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define ZONRI_DRV8301_PH_VOLTAGE_SENSE_GAIN                                                                            \
    (0.069767f) // Gain of the voltage sensing circuit (V/V), 5.1 / 73.1 per manual
#define ZONRI_DRV8301_PH_VOLTAGE_SENSE_BIAS_V  (0.0f) // Bias of the voltage sensing circuit (V), passive divider to GND.
#define ZONRI_DRV8301_PH_VOLTAGE_SENSE_POLE_HZ (400.0f) // Using a safe default value.
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (ZONRI_DRV8301_DCBUS_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define ZONRI_DRV8301_DCBUS_VOLTAGE_SENSE_GAIN                                                                         \
    (0.069767f) // Gain of the voltage sensing circuit (V/V), 5.1 / 73.1 per manual
#define ZONRI_DRV8301_DCBUS_VOLTAGE_SENSE_BIAS_V (0.0f) // Bias of the voltage sensing circuit (V)
#define ZONRI_DRV8301_DCBUS_VOLTAGE_SENSE_POLE_HZ                                                                      \
    (400.0f) // Filter bandwidth of the signal path (Hz), using a safe default.
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (ZONRI_DRV8301_DCBUS_CURRENT_SENSE_TYPE != 0)
// Not applicable for this board
#endif

//--- 4.5: Thermal Sensing ---
#if (ZONRI_DRV8301_THERMAL_SENSE_TYPE != 0)
// Not applicable for this board
#endif

/** @} */ // end of hal_zonri_drv8301_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_zonri_drv8301_mapping Default Parameter Mapping
 * @ingroup hal_zonri_drv8301
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(ZONRI_DRV8301_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 ZONRI_DRV8301_NAME
#define MY_BOARD_GATE_DRIVER_IC       ZONRI_DRV8301_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   ZONRI_DRV8301_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL ZONRI_DRV8301_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL ZONRI_DRV8301_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         ZONRI_DRV8301_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         ZONRI_DRV8301_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  ZONRI_DRV8301_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A ZONRI_DRV8301_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         ZONRI_DRV8301_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     ZONRI_DRV8301_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY ZONRI_DRV8301_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     ZONRI_DRV8301_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  ZONRI_DRV8301_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  ZONRI_DRV8301_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        ZONRI_DRV8301_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_SHUNT_RESISTANCE_OHM     ZONRI_DRV8301_PH_SHUNT_RESISTANCE_OHM
#define MY_BOARD_PH_CSA_GAIN_V_V             ZONRI_DRV8301_PH_CSA_GAIN_V_V
#define MY_BOARD_PH_CSA_BIAS_V               ZONRI_DRV8301_PH_CSA_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ    ZONRI_DRV8301_PH_CURRENT_SENSE_POLE_HZ
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN       ZONRI_DRV8301_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V     ZONRI_DRV8301_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ    ZONRI_DRV8301_PH_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN    ZONRI_DRV8301_DCBUS_VOLTAGE_SENSE_GAIN
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V  ZONRI_DRV8301_DCBUS_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ ZONRI_DRV8301_DCBUS_VOLTAGE_SENSE_POLE_HZ

#endif // ZONRI_DRV8301_IS_DEFAULT_PARAM

/** @} */ // end of hal_zonri_drv8301_mapping
//=================================================================================================

/** @} */ // end of hal_zonri_drv8301

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_ZONRI_DRV8301_MODULE_H
