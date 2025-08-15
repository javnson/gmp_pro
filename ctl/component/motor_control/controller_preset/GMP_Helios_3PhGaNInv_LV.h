/**
 * @file    gmp_inn_lv3phgan.h
 * @brief   Hardware Abstraction Layer (HAL) for the GMP INN-LV3PHGAN motor driver board.
 * @version 1.0
 * @date    2025-08-12
 * @author  Gemini (based on user template and GMP schematic INN_LV3PHGAN.pdf)
 * @note    This file contains the hardware parameters for the Green Motion Plus
 * INN-LV3PHGAN motor driver board. The values are derived from the
 * official schematic.
 * By defining 'GMP_INN_LV3PHGAN_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/motor_control/controller_preset/controller_preset_general.h>

#ifndef MOTOR_DRIVER_HAL_GMP_INN_LV3PHGAN_H
#define MOTOR_DRIVER_HAL_GMP_INN_LV3PHGAN_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_gmp_inn_lv3phgan GMP INN-LV3PHGAN Hardware Abstraction Layer
 * @ingroup CTL_MC_PRESET
 * @brief Parameters for the GMP INN-LV3PHGAN motor driver board.
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_gmp_inn_lv3phgan_nameplate Nameplate
 * @ingroup hal_gmp_inn_lv3phgan
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define GMP_INN_LV3PHGAN_NAME                 "GMP Helios 3PhGaNInv LV"
#define GMP_INN_LV3PHGAN_GATE_DRIVER_IC       "INS2001FQ"
#define GMP_INN_LV3PHGAN_MOSFET_PART_NUMBER   "INN100E0016A (GaN FET)"
#define GMP_INN_LV3PHGAN_CURRENT_SENSOR_MODEL "Shunt + TMCS1133 Hall Sensor"
#define GMP_INN_LV3PHGAN_THERMAL_SENSOR_MODEL "N.A."

/** @} */ // end of hal_gmp_inn_lv3phgan_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_gmp_inn_lv3phgan_limits Physical Operating Limits
 * @ingroup hal_gmp_inn_lv3phgan
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define GMP_INN_LV3PHGAN_VBUS_MIN_V         (12.0f) // Minimum DC bus operating voltage (V), typical for this class.
#define GMP_INN_LV3PHGAN_VBUS_MAX_V         (90.0f) // Maximum DC bus operating voltage (V), per PCB marking.
#define GMP_INN_LV3PHGAN_CURRENT_MAX_RMS_A  (25.0f) // Maximum continuous phase current (RMS, A), derived from peak.
#define GMP_INN_LV3PHGAN_CURRENT_MAX_PEAK_A (35.0f) // Maximum allowed peak phase current (Peak, A), per PCB marking.
#define GMP_INN_LV3PHGAN_TEMP_MAX_C         (85.0f) // Recommended maximum PCB operating temperature (°„C), safe default.

/** @} */ // end of hal_gmp_inn_lv3phgan_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_gmp_inn_lv3phgan_topology Sensing Topology
 * @ingroup hal_gmp_inn_lv3phgan
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define GMP_INN_LV3PHGAN_PH_CURRENT_SENSE_TYPE     (2) // SENSOR_TYPE_HALL (TMCS1133)
#define GMP_INN_LV3PHGAN_PH_CURRENT_SENSE_TOPOLOGY (3) // CS_TOPOLOGY_INLINE (In-line phase current sensing)
#define GMP_INN_LV3PHGAN_PH_VOLTAGE_SENSE_TYPE     (1) // VS_TYPE_PHASE_GND (Resistive divider)
#define GMP_INN_LV3PHGAN_DCBUS_VOLTAGE_SENSE_TYPE  (1) // VS_TYPE_PHASE_GND (Resistive divider)
#define GMP_INN_LV3PHGAN_DCBUS_CURRENT_SENSE_TYPE  (1) // SENSOR_TYPE_SHUNT (AMC1311BDWR)
#define GMP_INN_LV3PHGAN_THERMAL_SENSE_TYPE        (0) // SENSOR_NONE

/** @} */ // end of hal_gmp_inn_lv3phgan_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_gmp_inn_lv3phgan_circuits Sensing Circuit Parameters
 * @ingroup hal_gmp_inn_lv3phgan
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
#if (GMP_INN_LV3PHGAN_PH_CURRENT_SENSE_TYPE == 2)             // SENSOR_TYPE_HALL
#define GMP_INN_LV3PHGAN_PH_CURRENT_SENSITIVITY_MV_A (100.0f) // Sensitivity for TMCS1133A4B is 100 mV/A.
#define GMP_INN_LV3PHGAN_PH_CURRENT_ZERO_BIAS_V      (1.65f)  // VREF/2 for TMCS1133 is 3.3V/2.
#endif
#define GMP_INN_LV3PHGAN_PH_CURRENT_SENSE_POLE_HZ (34000.0f) // Approx. 1 / (2 * PI * 49.9k * 1nF) from schematic.

//--- 4.2: Phase Voltage Sensing ---
#if (GMP_INN_LV3PHGAN_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define GMP_INN_LV3PHGAN_PH_VOLTAGE_SENSE_GAIN                                                                         \
    (0.0109f) // Gain of the voltage sensing circuit (V/V), R72 / (R71 + R72) = 3.3k / (300k + 3.3k)
#define GMP_INN_LV3PHGAN_PH_VOLTAGE_SENSE_BIAS_V                                                                       \
    (0.0f) // Bias of the voltage sensing circuit (V), passive divider to GND.
#define GMP_INN_LV3PHGAN_PH_VOLTAGE_SENSE_POLE_HZ (48200.0f) // Approx. 1 / (2 * PI * 3.3k * 1nF) from schematic.
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (GMP_INN_LV3PHGAN_DCBUS_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define GMP_INN_LV3PHGAN_DCBUS_VOLTAGE_SENSE_GAIN                                                                      \
    (0.0109f) // Gain of the voltage sensing circuit (V/V), R50 / (R47 + R50) = 3.3k / (300k + 3.3k)
#define GMP_INN_LV3PHGAN_DCBUS_VOLTAGE_SENSE_BIAS_V  (0.0f)     // Bias of the voltage sensing circuit (V).
#define GMP_INN_LV3PHGAN_DCBUS_VOLTAGE_SENSE_POLE_HZ (48200.0f) // Approx. 1 / (2 * PI * 3.3k * 1nF) from schematic.
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (GMP_INN_LV3PHGAN_DCBUS_CURRENT_SENSE_TYPE == 1)            // SENSOR_TYPE_SHUNT
#define GMP_INN_LV3PHGAN_DCBUS_SHUNT_RESISTANCE_OHM  (0.0003f)  // R42 is 0.3 mOhm.
#define GMP_INN_LV3PHGAN_DCBUS_CSA_GAIN_V_V          (8.2f)     // AMC1311 has a fixed gain of 8.2.
#define GMP_INN_LV3PHGAN_DCBUS_CSA_BIAS_V            (0.0f)     // AMC1311 is a differential output isolated amplifier.
#define GMP_INN_LV3PHGAN_DCBUS_CURRENT_SENSE_POLE_HZ (48200.0f) // Approx. 1 / (2 * PI * 3.3k * 1nF) from schematic.
#endif

//--- 4.5: Thermal Sensing ---
#if (GMP_INN_LV3PHGAN_THERMAL_SENSE_TYPE != 0)
// Not applicable for this board
#endif

/** @} */ // end of hal_gmp_inn_lv3phgan_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_gmp_inn_lv3phgan_mapping Default Parameter Mapping
 * @ingroup hal_gmp_inn_lv3phgan
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(GMP_INN_LV3PHGAN_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 GMP_INN_LV3PHGAN_NAME
#define MY_BOARD_GATE_DRIVER_IC       GMP_INN_LV3PHGAN_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   GMP_INN_LV3PHGAN_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL GMP_INN_LV3PHGAN_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL GMP_INN_LV3PHGAN_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         GMP_INN_LV3PHGAN_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         GMP_INN_LV3PHGAN_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  GMP_INN_LV3PHGAN_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A GMP_INN_LV3PHGAN_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         GMP_INN_LV3PHGAN_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     GMP_INN_LV3PHGAN_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY GMP_INN_LV3PHGAN_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     GMP_INN_LV3PHGAN_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  GMP_INN_LV3PHGAN_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  GMP_INN_LV3PHGAN_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        GMP_INN_LV3PHGAN_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_CURRENT_SENSITIVITY_MV_A GMP_INN_LV3PHGAN_PH_CURRENT_SENSITIVITY_MV_A
#define MY_BOARD_PH_CURRENT_ZERO_BIAS_V      GMP_INN_LV3PHGAN_PH_CURRENT_ZERO_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ    GMP_INN_LV3PHGAN_PH_CURRENT_SENSE_POLE_HZ
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN       GMP_INN_LV3PHGAN_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V     GMP_INN_LV3PHGAN_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ    GMP_INN_LV3PHGAN_PH_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN    GMP_INN_LV3PHGAN_DCBUS_VOLTAGE_SENSE_GAIN
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V  GMP_INN_LV3PHGAN_DCBUS_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ GMP_INN_LV3PHGAN_DCBUS_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_SHUNT_RESISTANCE_OHM  GMP_INN_LV3PHGAN_DCBUS_SHUNT_RESISTANCE_OHM
#define MY_BOARD_DCBUS_CSA_GAIN_V_V          GMP_INN_LV3PHGAN_DCBUS_CSA_GAIN_V_V
#define MY_BOARD_DCBUS_CSA_BIAS_V            GMP_INN_LV3PHGAN_DCBUS_CSA_BIAS_V
#define MY_BOARD_DCBUS_CURRENT_SENSE_POLE_HZ GMP_INN_LV3PHGAN_DCBUS_CURRENT_SENSE_POLE_HZ

#endif // GMP_INN_LV3PHGAN_IS_DEFAULT_PARAM

/** @} */ // end of hal_gmp_inn_lv3phgan_mapping
//=================================================================================================

/** @} */ // end of hal_gmp_inn_lv3phgan

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_GMP_INN_LV3PHGAN_H
