/**
 * @file    x_nucleo_ihm07m1.h
 * @brief   Hardware Abstraction Layer (HAL) for the ST X-NUCLEO-IHM07M1 expansion board.
 * @version 1.0
 * @date    2025-08-12
 * @author  Gemini (based on user template and ST manual um1943)
 * @note    This file contains the hardware parameters for the STMicroelectronics
 * X-NUCLEO-IHM07M1 motor driver expansion board. The values are derived
 * from the official user manual (um1943).
 * By defining 'X_NUCLEO_IHM07M1_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/motor_control/controller_preset/controller_preset_general.h>

#ifndef MOTOR_DRIVER_HAL_X_NUCLEO_IHM07M1_H
#define MOTOR_DRIVER_HAL_X_NUCLEO_IHM07M1_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_x_nucleo_ihm07m1 X-NUCLEO-IHM07M1 Hardware Abstraction Layer
 * @ingroup CTL_MC_PRESET
 * @brief Parameters for the ST X-NUCLEO-IHM07M1 motor driver board.
 * @details reference https://www.st.com.cn/zh/ecosystems/x-nucleo-ihm07m1.html
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_x_nucleo_ihm07m1_nameplate Nameplate
 * @ingroup hal_x_nucleo_ihm07m1
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define X_NUCLEO_IHM07M1_NAME                 "ST X-NUCLEO-IHM07M1"
#define X_NUCLEO_IHM07M1_GATE_DRIVER_IC       "L6230"
#define X_NUCLEO_IHM07M1_MOSFET_PART_NUMBER   "Integrated in L6230"
#define X_NUCLEO_IHM07M1_CURRENT_SENSOR_MODEL "Low-side Shunt + TSV994IPT OPAMP"
#define X_NUCLEO_IHM07M1_THERMAL_SENSOR_MODEL "NTC on board (RS model 742-8420)"

/** @} */ // end of hal_x_nucleo_ihm07m1_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_x_nucleo_ihm07m1_limits Physical Operating Limits
 * @ingroup hal_x_nucleo_ihm07m1
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define X_NUCLEO_IHM07M1_VBUS_MIN_V        (8.0f)  // Minimum DC bus operating voltage (V), per manual section 1.1
#define X_NUCLEO_IHM07M1_VBUS_MAX_V        (48.0f) // Maximum DC bus operating voltage (V), per manual section 1.1
#define X_NUCLEO_IHM07M1_CURRENT_MAX_RMS_A (1.4f)  // Maximum continuous phase current (RMS, A), per manual section 1.1
#define X_NUCLEO_IHM07M1_CURRENT_MAX_PEAK_A                                                                            \
    (2.8f)                                  // Maximum allowed peak phase current (Peak, A), per manual section 1.1
#define X_NUCLEO_IHM07M1_TEMP_MAX_C (85.0f) // Recommended maximum PCB operating temperature (°„C), safe default

/** @} */ // end of hal_x_nucleo_ihm07m1_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_x_nucleo_ihm07m1_topology Sensing Topology
 * @ingroup hal_x_nucleo_ihm07m1
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define X_NUCLEO_IHM07M1_PH_CURRENT_SENSE_TYPE     (1) // SENSOR_TYPE_SHUNT (Three-shunt and single-shunt configurable)
#define X_NUCLEO_IHM07M1_PH_CURRENT_SENSE_TOPOLOGY (1) // CS_TOPOLOGY_LOW_SIDE (Low-side current sensing)
#define X_NUCLEO_IHM07M1_PH_VOLTAGE_SENSE_TYPE     (1) // VS_TYPE_PHASE_GND (Resistive divider for BEMF)
#define X_NUCLEO_IHM07M1_DCBUS_VOLTAGE_SENSE_TYPE  (1) // VS_TYPE_PHASE_GND (Resistive divider for Vbus)
#define X_NUCLEO_IHM07M1_DCBUS_CURRENT_SENSE_TYPE  (0) // SENSOR_NONE (No DC bus current sensor present)
#define X_NUCLEO_IHM07M1_THERMAL_SENSE_TYPE        (1) // THERMAL_SENSOR_NTC (NTC on board)

/** @} */ // end of hal_x_nucleo_ihm07m1_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_x_nucleo_ihm07m1_circuits Sensing Circuit Parameters
 * @ingroup hal_x_nucleo_ihm07m1
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
#if (X_NUCLEO_IHM07M1_PH_CURRENT_SENSE_TYPE == 1) // SENSOR_TYPE_SHUNT
#define X_NUCLEO_IHM07M1_PH_SHUNT_RESISTANCE_OHM                                                                       \
    (0.33f) // Resistance of the shunt resistor (Ohm), per schematic (R43, R44, R45).
#define X_NUCLEO_IHM07M1_PH_CSA_GAIN_V_V (1.53f) // Overall Gain AV=1.53, per schematic.
#define X_NUCLEO_IHM07M1_PH_CSA_BIAS_V   (1.65f) // Bias is typically VREF/2 (3.3V / 2).
#endif
#define X_NUCLEO_IHM07M1_PH_CURRENT_SENSE_POLE_HZ (353000.0f) // 1 / (2 * PI * 680 * 680pF)

//--- 4.2: Phase Voltage Sensing ---
#if (X_NUCLEO_IHM07M1_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define X_NUCLEO_IHM07M1_PH_VOLTAGE_SENSE_GAIN                                                                         \
    (0.08333f) // Gain of the voltage sensing circuit (V/V), R40/(R39+R40) = 10k/(10k+10k) is not correct. Assumed 10k/110k for BEMF.
#define X_NUCLEO_IHM07M1_PH_VOLTAGE_SENSE_BIAS_V                                                                       \
    (0.0f) // Bias of the voltage sensing circuit (V), passive divider to GND.
#define X_NUCLEO_IHM07M1_PH_VOLTAGE_SENSE_POLE_HZ (1000.0f) // Assumed filter bandwidth.
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (X_NUCLEO_IHM07M1_DCBUS_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define X_NUCLEO_IHM07M1_DCBUS_VOLTAGE_SENSE_GAIN                                                                      \
    (0.0522f) // Gain of the voltage sensing circuit (V/V), R18 / (R17 + R18) = 9.31k / (169k + 9.31k).
#define X_NUCLEO_IHM07M1_DCBUS_VOLTAGE_SENSE_BIAS_V (0.0f) // Bias of the voltage sensing circuit (V).
#define X_NUCLEO_IHM07M1_DCBUS_VOLTAGE_SENSE_POLE_HZ                                                                   \
    (19280.0f) // Filter bandwidth 1/(2*pi*(R17||R18)*C14) = 1/(2*pi*8.83k*4.7nF)
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (X_NUCLEO_IHM07M1_DCBUS_CURRENT_SENSE_TYPE != 0)
// Not applicable for this board
#endif

//--- 4.5: Thermal Sensing ---
#if (X_NUCLEO_IHM07M1_THERMAL_SENSE_TYPE == 1)                   // THERMAL_SENSOR_NTC
#define X_NUCLEO_IHM07M1_THERMAL_PULLUP_RESISTANCE_OHM (4700.0f) // Pull-up resistor R20 (Ohm).
#define X_NUCLEO_IHM07M1_THERMAL_NTC_BETA_VALUE        (3950.0f) // Assumed Beta value for typical 10k NTC.
#define X_NUCLEO_IHM07M1_THERMAL_NTC_NOMINAL_R_OHM                                                                     \
    (10000.0f) // Nominal resistance of NTC R19 at nominal temperature (Ohm).
#define X_NUCLEO_IHM07M1_THERMAL_NTC_NOMINAL_TEMP_C (25.0f) // Assumed nominal temperature (°„C).
#endif

/** @} */ // end of hal_x_nucleo_ihm07m1_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_x_nucleo_ihm07m1_mapping Default Parameter Mapping
 * @ingroup hal_x_nucleo_ihm07m1
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(X_NUCLEO_IHM07M1_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 X_NUCLEO_IHM07M1_NAME
#define MY_BOARD_GATE_DRIVER_IC       X_NUCLEO_IHM07M1_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   X_NUCLEO_IHM07M1_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL X_NUCLEO_IHM07M1_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL X_NUCLEO_IHM07M1_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         X_NUCLEO_IHM07M1_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         X_NUCLEO_IHM07M1_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  X_NUCLEO_IHM07M1_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A X_NUCLEO_IHM07M1_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         X_NUCLEO_IHM07M1_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     X_NUCLEO_IHM07M1_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY X_NUCLEO_IHM07M1_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     X_NUCLEO_IHM07M1_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  X_NUCLEO_IHM07M1_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  X_NUCLEO_IHM07M1_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        X_NUCLEO_IHM07M1_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_SHUNT_RESISTANCE_OHM       X_NUCLEO_IHM07M1_PH_SHUNT_RESISTANCE_OHM
#define MY_BOARD_PH_CSA_GAIN_V_V               X_NUCLEO_IHM07M1_PH_CSA_GAIN_V_V
#define MY_BOARD_PH_CSA_BIAS_V                 X_NUCLEO_IHM07M1_PH_CSA_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ      X_NUCLEO_IHM07M1_PH_CURRENT_SENSE_POLE_HZ
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN         X_NUCLEO_IHM07M1_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V       X_NUCLEO_IHM07M1_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ      X_NUCLEO_IHM07M1_PH_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN      X_NUCLEO_IHM07M1_DCBUS_VOLTAGE_SENSE_GAIN
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V    X_NUCLEO_IHM07M1_DCBUS_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ   X_NUCLEO_IHM07M1_DCBUS_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_THERMAL_PULLUP_RESISTANCE_OHM X_NUCLEO_IHM07M1_THERMAL_PULLUP_RESISTANCE_OHM
#define MY_BOARD_THERMAL_NTC_BETA_VALUE        X_NUCLEO_IHM07M1_THERMAL_NTC_BETA_VALUE
#define MY_BOARD_THERMAL_NTC_NOMINAL_R_OHM     X_NUCLEO_IHM07M1_THERMAL_NTC_NOMINAL_R_OHM
#define MY_BOARD_THERMAL_NTC_NOMINAL_TEMP_C    X_NUCLEO_IHM07M1_THERMAL_NTC_NOMINAL_TEMP_C

#endif // X_NUCLEO_IHM07M1_IS_DEFAULT_PARAM

/** @} */ // end of hal_x_nucleo_ihm07m1_mapping
//=================================================================================================

/** @} */ // end of hal_x_nucleo_ihm07m1

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_X_NUCLEO_IHM07M1_H
