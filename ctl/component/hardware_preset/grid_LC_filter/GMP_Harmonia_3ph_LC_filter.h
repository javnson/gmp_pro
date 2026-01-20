/**
 * @file    harmonia_3ph_lc_filter.h
 * @brief   Hardware Abstraction Layer (HAL) for the Harmonia 3-Phase Grid-Tied LC Filter Board.
 * @version 1.0
 * @date    2026-01-20
 * @author  Gemini (based on Harmonia 3ph LC filter schematic)
 * @note    This file contains the hardware parameters for the Harmonia 3Ph LC Filter.
 * It includes definitions for the TLE4971 current sensors and the AMC1350
 * based high-voltage sensing chain.
 * By defining 'HARMONIA_3PH_LC_FILTER_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/hardware_preset/inverter_3ph/inverter_3ph_general.h>

#ifndef MOTOR_DRIVER_HAL_HARMONIA_3PH_LC_FILTER_H
#define MOTOR_DRIVER_HAL_HARMONIA_3PH_LC_FILTER_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_harmonia_3ph_lc_filter Harmonia 3Ph LC Filter HAL
 * @ingroup CTL_MC_PRESET
 * @brief Parameters for the Harmonia 3-Phase Grid-Tied LC Filter Board.
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_harmonia_3ph_lc_filter_nameplate Nameplate
 * @ingroup hal_harmonia_3ph_lc_filter
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define HARMONIA_3PH_LC_FILTER_NAME                 "Harmonia 3Ph LC Filter"
#define HARMONIA_3PH_LC_FILTER_GATE_DRIVER_IC       "N.A. (Passive Filter Board)"
#define HARMONIA_3PH_LC_FILTER_MOSFET_PART_NUMBER   "N.A."
#define HARMONIA_3PH_LC_FILTER_CURRENT_SENSOR_MODEL "Infineon TLE4971A025"
#define HARMONIA_3PH_LC_FILTER_VOLTAGE_SENSOR_MODEL "ResDiv + AMC1350 + TLV9061"

/** @} */ // end of hal_harmonia_3ph_lc_filter_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_harmonia_3ph_lc_filter_limits Physical Operating Limits
 * @ingroup hal_harmonia_3ph_lc_filter
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define HARMONIA_3PH_LC_FILTER_VBUS_MIN_V        (0.0f)    // Passive device, no minimum voltage.
#define HARMONIA_3PH_LC_FILTER_VBUS_MAX_V        (1000.0f) // Maximum voltage (V), limited by Banana Connectors.
#define HARMONIA_3PH_LC_FILTER_CURRENT_MAX_RMS_A (24.0f) // Maximum continuous current (RMS, A), limited by connectors.
#define HARMONIA_3PH_LC_FILTER_CURRENT_MAX_PEAK_A                                                                      \
    (25.0f)                                       // Maximum allowed peak current (Peak, A), limited by Sensor Range.
#define HARMONIA_3PH_LC_FILTER_TEMP_MAX_C (85.0f) // Recommended maximum PCB operating temperature (°„C).

//--- Filter Parameters ---
#define HARMONIA_3PH_LC_FILTER_INDUCTANCE_H  (0.0015f)   // L = 1.5mH Main Filter Inductor.
#define HARMONIA_3PH_LC_FILTER_CAPACITANCE_F (0.000005f) // C = 5uF Main Filter Capacitor.

/** @} */ // end of hal_harmonia_3ph_lc_filter_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_harmonia_3ph_lc_filter_topology Sensing Topology
 * @ingroup hal_harmonia_3ph_lc_filter
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSE_TYPE     (2) // SENSOR_TYPE_HALL (TLE4971 Coreless Hall)
#define HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSE_TOPOLOGY (3) // CS_TOPOLOGY_INLINE (In-line phase current sensing)
#define HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_TYPE     (1) // VS_TYPE_PHASE_GND (Resistive divider to HV_PGND)
#define HARMONIA_3PH_LC_FILTER_DCBUS_VOLTAGE_SENSE_TYPE  (0) // SENSOR_NONE (Handled by Phase Voltage or external)
#define HARMONIA_3PH_LC_FILTER_DCBUS_CURRENT_SENSE_TYPE  (0) // SENSOR_NONE
#define HARMONIA_3PH_LC_FILTER_THERMAL_SENSE_TYPE        (0) // SENSOR_NONE

/** @} */ // end of hal_harmonia_3ph_lc_filter_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_harmonia_3ph_lc_filter_circuits Sensing Circuit Parameters
 * @ingroup hal_harmonia_3ph_lc_filter
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing (Infineon TLE4971A025) ---
#if (HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSE_TYPE == 2)                // SENSOR_TYPE_HALL
#define HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSITIVITY_MV_A (48.0f)     // Sensitivity for TLE4971A025 is 48 mV/A.
#define HARMONIA_3PH_LC_FILTER_PH_CURRENT_ZERO_BIAS_V      (1.65f)     // VREF/2 (3.3V/2) internal ref.
#define HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSE_POLE_HZ    (210000.0f) // Bandwidth is 210kHz per datasheet.
#endif

//--- 4.2: Phase Voltage Sensing (Divider + AMC1350 + OpAmp) ---
#if (HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE

// Resistor Configuration for Gain Calculation
// Stage 1: High Voltage Divider (7x 560k Top, 42k Bottom)
#define HARMONIA_3PH_LC_FILTER_VS_R_TOP_TOTAL_OHM (3920000.0f) // 7 * 560k Ohm
#define HARMONIA_3PH_LC_FILTER_VS_R_BOT_OHM       (42000.0f)   // 42k Ohm

// Stage 2: Isolation Amplifier (AMC1350)
#define HARMONIA_3PH_LC_FILTER_VS_ISO_GAIN_V_V (0.4f) // Fixed gain of AMC1350.

// Stage 3: Output Differential OpAmp (TLV9061)
// User Note: Gain is approx 1.65/2 (0.825). Adjust R_FB/R_IN to match.
#define HARMONIA_3PH_LC_FILTER_VS_OPAMP_R_FB_OHM (1650.0f) // Feedback Resistor
#define HARMONIA_3PH_LC_FILTER_VS_OPAMP_R_IN_OHM (2000.0f) // Input Resistor

// Calculated Total Gain and Bias
#define HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_GAIN                                                                   \
    ((HARMONIA_3PH_LC_FILTER_VS_R_BOT_OHM /                                                                            \
      (HARMONIA_3PH_LC_FILTER_VS_R_TOP_TOTAL_OHM + HARMONIA_3PH_LC_FILTER_VS_R_BOT_OHM)) *                             \
     HARMONIA_3PH_LC_FILTER_VS_ISO_GAIN_V_V *                                                                          \
     (HARMONIA_3PH_LC_FILTER_VS_OPAMP_R_FB_OHM / HARMONIA_3PH_LC_FILTER_VS_OPAMP_R_IN_OHM))

#define HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_BIAS_V  (1.65f)     // Target bias at ADC input.
#define HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_POLE_HZ (100000.0f) // Approx bandwidth dominated by IsoAmp or filter.
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (HARMONIA_3PH_LC_FILTER_DCBUS_VOLTAGE_SENSE_TYPE != 0)
// Not populated on this board definition
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (HARMONIA_3PH_LC_FILTER_DCBUS_CURRENT_SENSE_TYPE == 1)
// Not populated on this board definition
#endif

/** @} */ // end of hal_harmonia_3ph_lc_filter_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_harmonia_3ph_lc_filter_mapping Default Parameter Mapping
 * @ingroup hal_harmonia_3ph_lc_filter
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(HARMONIA_3PH_LC_FILTER_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 HARMONIA_3PH_LC_FILTER_NAME
#define MY_BOARD_GATE_DRIVER_IC       HARMONIA_3PH_LC_FILTER_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   HARMONIA_3PH_LC_FILTER_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL HARMONIA_3PH_LC_FILTER_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL HARMONIA_3PH_LC_FILTER_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         HARMONIA_3PH_LC_FILTER_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         HARMONIA_3PH_LC_FILTER_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  HARMONIA_3PH_LC_FILTER_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A HARMONIA_3PH_LC_FILTER_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         HARMONIA_3PH_LC_FILTER_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  HARMONIA_3PH_LC_FILTER_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  HARMONIA_3PH_LC_FILTER_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        HARMONIA_3PH_LC_FILTER_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_CURRENT_SENSITIVITY_MV_A HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSITIVITY_MV_A
#define MY_BOARD_PH_CURRENT_ZERO_BIAS_V      HARMONIA_3PH_LC_FILTER_PH_CURRENT_ZERO_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ    HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSE_POLE_HZ

// Voltage Sensing Mapping
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN    HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V  HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_POLE_HZ

// Extra: Filter Parameters
#define MY_BOARD_FILTER_INDUCTANCE_H  HARMONIA_3PH_LC_FILTER_INDUCTANCE_H
#define MY_BOARD_FILTER_CAPACITANCE_F HARMONIA_3PH_LC_FILTER_CAPACITANCE_F

#endif // HARMONIA_3PH_LC_FILTER_IS_DEFAULT_PARAM

/** @} */ // end of hal_harmonia_3ph_lc_filter_mapping
//=================================================================================================

/** @} */ // end of hal_harmonia_3ph_lc_filter

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_HARMONIA_3PH_LC_FILTER_H
