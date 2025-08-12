/**
 * @file    b_g431b_esc1_discovery.h
 * @brief   Hardware Abstraction Layer (HAL) for the ST B-G431B-ESC1 Discovery kit.
 * @version 1.0
 * @date    2025-08-12
 * @author  Gemini (based on user template and ST manual um2516)
 * @note    This file contains the hardware parameters for the STMicroelectronics
 * B-G431B-ESC1 Discovery kit. The values are derived from the official
 * user manual (um2516).
 * By defining 'B_G431B_ESC1_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/motor_control/controller_preset/controller_preset_general.h>

#ifndef MOTOR_DRIVER_HAL_B_G431B_ESC1_DISCOVERY_H
#define MOTOR_DRIVER_HAL_B_G431B_ESC1_DISCOVERY_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_b_g431b_esc1 B-G431B-ESC1 Discovery Hardware Abstraction Layer
 * @brief Parameters for the ST B-G431B-ESC1 motor driver board.
 * @details reference https://www.st.com/en/evaluation-tools/b-g431b-esc1.html
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_b_g431b_esc1_nameplate Nameplate
 * @ingroup hal_b_g431b_esc1
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define B_G431B_ESC1_NAME                 "ST B-G431B-ESC1 Discovery kit"
#define B_G431B_ESC1_GATE_DRIVER_IC       "L6387"
#define B_G431B_ESC1_MOSFET_PART_NUMBER   "STL180N6F7"
#define B_G431B_ESC1_CURRENT_SENSOR_MODEL "Low-side Shunt (Amplification via STM32G431 Internal OPAMP)"
#define B_G431B_ESC1_THERMAL_SENSOR_MODEL "NTC on board"

/** @} */ // end of hal_b_g431b_esc1_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_b_g431b_esc1_limits Physical Operating Limits
 * @ingroup hal_b_g431b_esc1
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define B_G431B_ESC1_VBUS_MIN_V         (9.0f)  // Minimum DC bus operating voltage (V), suitable for 3S LiPo battery.
#define B_G431B_ESC1_VBUS_MAX_V         (26.0f) // Maximum DC bus operating voltage (V), suitable for 6S LiPo battery.
#define B_G431B_ESC1_CURRENT_MAX_RMS_A  (28.0f) // Maximum continuous phase current (RMS, A), derived from peak.
#define B_G431B_ESC1_CURRENT_MAX_PEAK_A (40.0f) // Maximum allowed peak phase current (Peak, A), per manual section 1.
#define B_G431B_ESC1_TEMP_MAX_C         (85.0f) // Recommended maximum PCB operating temperature (°„C), safe default.

/** @} */ // end of hal_b_g431b_esc1_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_b_g431b_esc1_topology Sensing Topology
 * @ingroup hal_b_g431b_esc1
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define B_G431B_ESC1_PH_CURRENT_SENSE_TYPE     (1) // SENSOR_TYPE_SHUNT (3-shunt mode)
#define B_G431B_ESC1_PH_CURRENT_SENSE_TOPOLOGY (1) // CS_TOPOLOGY_LOW_SIDE (Low-side current sensing)
#define B_G431B_ESC1_PH_VOLTAGE_SENSE_TYPE     (1) // VS_TYPE_PHASE_GND (Resistive divider for BEMF)
#define B_G431B_ESC1_DCBUS_VOLTAGE_SENSE_TYPE  (1) // VS_TYPE_PHASE_GND (Resistive divider for Vbus)
#define B_G431B_ESC1_DCBUS_CURRENT_SENSE_TYPE  (0) // SENSOR_NONE (No DC bus current sensor present)
#define B_G431B_ESC1_THERMAL_SENSE_TYPE        (1) // THERMAL_SENSOR_NTC (NTC on board)

/** @} */ // end of hal_b_g431b_esc1_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_b_g431b_esc1_circuits Sensing Circuit Parameters
 * @ingroup hal_b_g431b_esc1
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
// Note: Amplification is handled by the STM32G431's internal, programmable op-amps.
#if (B_G431B_ESC1_PH_CURRENT_SENSE_TYPE == 1) // SENSOR_TYPE_SHUNT
#define B_G431B_ESC1_PH_SHUNT_RESISTANCE_OHM                                                                           \
    (0.001f) // Resistance of the shunt resistor (Ohm), assumed 1m¶∏ for this current range.
#define B_G431B_ESC1_PH_CSA_GAIN_V_V (1.0f)  // Gain is configured in MCU firmware.
#define B_G431B_ESC1_PH_CSA_BIAS_V   (1.65f) // Bias is typically VREF/2, configured in MCU firmware.
#endif
#define B_G431B_ESC1_PH_CURRENT_SENSE_POLE_HZ                                                                          \
    (250.0e3f) // Bandwidth is primarily limited by MCU's OPAMP, using a safe default.

//--- 4.2: Phase Voltage Sensing ---
// Note: Resistor values are assumed based on typical designs as they are not listed in the manual.
#if (B_G431B_ESC1_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define B_G431B_ESC1_PH_VOLTAGE_SENSE_GAIN                                                                             \
    (0.10714f) // Gain of the voltage sensing circuit (V/V), assumed R_high=100k, R_low=12k for 3.3V ADC.
#define B_G431B_ESC1_PH_VOLTAGE_SENSE_BIAS_V  (0.0f) // Bias of the voltage sensing circuit (V), passive divider to GND.
#define B_G431B_ESC1_PH_VOLTAGE_SENSE_POLE_HZ (200.0f) // Assumed filter bandwidth.
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (B_G431B_ESC1_DCBUS_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define B_G431B_ESC1_DCBUS_VOLTAGE_SENSE_GAIN                                                                          \
    (0.10714f) // Gain of the voltage sensing circuit (V/V), same as phase sense.
#define B_G431B_ESC1_DCBUS_VOLTAGE_SENSE_BIAS_V (0.0f) // Bias of the voltage sensing circuit (V).
#define B_G431B_ESC1_DCBUS_VOLTAGE_SENSE_POLE_HZ                                                                       \
    (200.0f) // Filter bandwidth of the signal path (Hz), same as phase sense.
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (B_G431B_ESC1_DCBUS_CURRENT_SENSE_TYPE != 0)
// Not applicable for this board
#endif

//--- 4.5: Thermal Sensing ---
// Note: NTC parameters are assumed based on typical values.
#if (B_G431B_ESC1_THERMAL_SENSE_TYPE == 1)                    // THERMAL_SENSOR_NTC
#define B_G431B_ESC1_THERMAL_PULLUP_RESISTANCE_OHM (10000.0f) // Assumed pull-up resistor (Ohm).
#define B_G431B_ESC1_THERMAL_NTC_BETA_VALUE        (3950.0f)  // Assumed Beta value of the NTC (K).
#define B_G431B_ESC1_THERMAL_NTC_NOMINAL_R_OHM     (10000.0f) // Assumed nominal resistance at nominal temperature (Ohm).
#define B_G431B_ESC1_THERMAL_NTC_NOMINAL_TEMP_C    (25.0f)    // Assumed nominal temperature (°„C).
#endif

/** @} */ // end of hal_b_g431b_esc1_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_b_g431b_esc1_mapping Default Parameter Mapping
 * @ingroup hal_b_g431b_esc1
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(B_G431B_ESC1_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 B_G431B_ESC1_NAME
#define MY_BOARD_GATE_DRIVER_IC       B_G431B_ESC1_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   B_G431B_ESC1_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL B_G431B_ESC1_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL B_G431B_ESC1_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         B_G431B_ESC1_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         B_G431B_ESC1_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  B_G431B_ESC1_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A B_G431B_ESC1_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         B_G431B_ESC1_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     B_G431B_ESC1_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY B_G431B_ESC1_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     B_G431B_ESC1_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  B_G431B_ESC1_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  B_G431B_ESC1_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        B_G431B_ESC1_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_SHUNT_RESISTANCE_OHM       B_G431B_ESC1_PH_SHUNT_RESISTANCE_OHM
#define MY_BOARD_PH_CSA_GAIN_V_V               B_G431B_ESC1_PH_CSA_GAIN_V_V
#define MY_BOARD_PH_CSA_BIAS_V                 B_G431B_ESC1_PH_CSA_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ      B_G431B_ESC1_PH_CURRENT_SENSE_POLE_HZ
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN         B_G431B_ESC1_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V       B_G431B_ESC1_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ      B_G431B_ESC1_PH_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN      B_G431B_ESC1_DCBUS_VOLTAGE_SENSE_GAIN
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V    B_G431B_ESC1_DCBUS_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ   B_G431B_ESC1_DCBUS_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_THERMAL_PULLUP_RESISTANCE_OHM B_G431B_ESC1_THERMAL_PULLUP_RESISTANCE_OHM
#define MY_BOARD_THERMAL_NTC_BETA_VALUE        B_G431B_ESC1_THERMAL_NTC_BETA_VALUE
#define MY_BOARD_THERMAL_NTC_NOMINAL_R_OHM     B_G431B_ESC1_THERMAL_NTC_NOMINAL_R_OHM
#define MY_BOARD_THERMAL_NTC_NOMINAL_TEMP_C    B_G431B_ESC1_THERMAL_NTC_NOMINAL_TEMP_C

#endif // B_G431B_ESC1_IS_DEFAULT_PARAM

/** @} */ // end of hal_b_g431b_esc1_mapping
//=================================================================================================

/** @} */ // end of hal_b_g431b_esc1

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_B_G431B_ESC1_DISCOVERY_H
