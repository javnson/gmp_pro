/**
 * @file    drv8304h_boosterpack.h
 * @brief   Hardware Abstraction Layer (HAL) for the BOOSTXL-DRV8304H motor driver board.
 * @version 1.0
 * @date    2025-08-12
 * @author  Gemini (based on user template and TI manual slvub97.pdf)
 * @note    This file contains the hardware parameters for the Texas Instruments
 * BOOSTXL-DRV8304H Motor Drive BoosterPack. The values are derived from
 * the official hardware user's guide (slvub97.pdf).
 * By defining 'BOOSTXL_DRV8304H_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/motor_control/controller_preset/controller_preset_general.h>

#ifndef MOTOR_DRIVER_HAL_DRV8304H_BOOSTERPACK_H
#define MOTOR_DRIVER_HAL_DRV8304H_BOOSTERPACK_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_boostxl_drv8304h BOOSTXL-DRV8304H Hardware Abstraction Layer
 * @brief Parameters for the TI BOOSTXL-DRV8304H motor driver board.
 * @details reference: https://www.ti.com.cn/tool/cn/BOOSTXL-DRV8304H
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_drv8304h_nameplate Nameplate
 * @ingroup hal_boostxl_drv8304h
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_DRV8304H_NAME                 "BOOSTXL-DRV8304H"
#define BOOSTXL_DRV8304H_GATE_DRIVER_IC       "DRV8304H" // (H=Hardware Interface)
#define BOOSTXL_DRV8304H_MOSFET_PART_NUMBER   "CSD88584Q5DC"
#define BOOSTXL_DRV8304H_CURRENT_SENSOR_MODEL "Low-side Shunt + DRV8304 Internal Amps"
#define BOOSTXL_DRV8304H_THERMAL_SENSOR_MODEL "Internal to DRV8304H (via nFAULT)"

/** @} */ // end of hal_drv8304h_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8304h_limits Physical Operating Limits
 * @ingroup hal_boostxl_drv8304h
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_DRV8304H_VBUS_MIN_V        (6.0f)  // Minimum DC bus operating voltage (V), per manual section 2.1.1
#define BOOSTXL_DRV8304H_VBUS_MAX_V        (38.0f) // Maximum DC bus operating voltage (V), per manual section 2.1.1
#define BOOSTXL_DRV8304H_CURRENT_MAX_RMS_A (10.0f) // Maximum continuous phase current (RMS, A), assumed typical value
#define BOOSTXL_DRV8304H_CURRENT_MAX_PEAK_A                                                                            \
    (15.0f)                                 // Maximum allowed peak phase current (Peak, A), assumed typical value
#define BOOSTXL_DRV8304H_TEMP_MAX_C (85.0f) // Recommended maximum PCB operating temperature (°„C), safe default

/** @} */ // end of hal_drv8304h_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8304h_topology Sensing Topology
 * @ingroup hal_boostxl_drv8304h
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_DRV8304H_PH_CURRENT_SENSE_TYPE     (1) // SENSOR_TYPE_SHUNT (Low-side shunts)
#define BOOSTXL_DRV8304H_PH_CURRENT_SENSE_TOPOLOGY (1) // CS_TOPOLOGY_LOW_SIDE (Low-side phase current sensing)
#define BOOSTXL_DRV8304H_PH_VOLTAGE_SENSE_TYPE     (1) // VS_TYPE_PHASE_GND (Resistive divider to GND)
#define BOOSTXL_DRV8304H_DCBUS_VOLTAGE_SENSE_TYPE  (1) // VS_TYPE_PHASE_GND (Resistive divider to GND)
#define BOOSTXL_DRV8304H_DCBUS_CURRENT_SENSE_TYPE  (0) // SENSOR_NONE (No DC bus current sensor present)
#define BOOSTXL_DRV8304H_THERMAL_SENSE_TYPE        (0) // SENSOR_NONE (No direct analog thermal sensor for MCU, only fault pin)

/** @} */ // end of hal_drv8304h_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8304h_circuits Sensing Circuit Parameters
 * @ingroup hal_boostxl_drv8304h
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
// Note: This board uses the internal amplifiers of the DRV8304.
// Shunt resistor value is assumed. Gain is hardware selectable.
#if (BOOSTXL_DRV8304H_PH_CURRENT_SENSE_TYPE == 1)         // SENSOR_TYPE_SHUNT
#define BOOSTXL_DRV8304H_PH_SHUNT_RESISTANCE_OHM (0.005f) // Resistance of the shunt resistor (Ohm), assumed 5m¶∏.
#define BOOSTXL_DRV8304H_PH_CSA_GAIN_V_V         (20.0f) // Assumed default gain (V/V). Can be 5, 10, 20, 40 via GAIN pin.
#define BOOSTXL_DRV8304H_PH_CSA_BIAS_V           (1.65f) // Bias voltage is typically VREF/2.
#endif
#define BOOSTXL_DRV8304H_PH_CURRENT_SENSE_POLE_HZ                                                                      \
    (250.0e3f) // Bandwidth is primarily limited by internal amplifier, using a safe default.

//--- 4.2: Phase Voltage Sensing ---
// Note: Resistor values are assumed based on typical designs as they are not listed in the manual.
#if (BOOSTXL_DRV8304H_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define BOOSTXL_DRV8304H_PH_VOLTAGE_SENSE_GAIN                                                                         \
    (0.053f) // Gain of the voltage sensing circuit (V/V), assumed R_high=100k, R_low=5.6k
#define BOOSTXL_DRV8304H_PH_VOLTAGE_SENSE_BIAS_V                                                                       \
    (0.0f) // Bias of the voltage sensing circuit (V), passive divider to GND.
#define BOOSTXL_DRV8304H_PH_VOLTAGE_SENSE_POLE_HZ (300.0f) // Approx. filter bandwidth with 0.1uF cap
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (BOOSTXL_DRV8304H_DCBUS_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define BOOSTXL_DRV8304H_DCBUS_VOLTAGE_SENSE_GAIN                                                                      \
    (0.053f) // Gain of the voltage sensing circuit (V/V), same as phase sense
#define BOOSTXL_DRV8304H_DCBUS_VOLTAGE_SENSE_BIAS_V (0.0f) // Bias of the voltage sensing circuit (V)
#define BOOSTXL_DRV8304H_DCBUS_VOLTAGE_SENSE_POLE_HZ                                                                   \
    (300.0f) // Filter bandwidth of the signal path (Hz), same as phase sense
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (BOOSTXL_DRV8304H_DCBUS_CURRENT_SENSE_TYPE != 0)
// Not applicable for this board
#endif

//--- 4.5: Thermal Sensing ---
#if (BOOSTXL_DRV8304H_THERMAL_SENSE_TYPE != 0)
// Not applicable for this board
#endif

/** @} */ // end of hal_drv8304h_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8304h_mapping Default Parameter Mapping
 * @ingroup hal_boostxl_drv8304h
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(BOOSTXL_DRV8304H_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 BOOSTXL_DRV8304H_NAME
#define MY_BOARD_GATE_DRIVER_IC       BOOSTXL_DRV8304H_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   BOOSTXL_DRV8304H_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL BOOSTXL_DRV8304H_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL BOOSTXL_DRV8304H_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         BOOSTXL_DRV8304H_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         BOOSTXL_DRV8304H_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  BOOSTXL_DRV8304H_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A BOOSTXL_DRV8304H_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         BOOSTXL_DRV8304H_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     BOOSTXL_DRV8304H_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY BOOSTXL_DRV8304H_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     BOOSTXL_DRV8304H_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  BOOSTXL_DRV8304H_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  BOOSTXL_DRV8304H_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        BOOSTXL_DRV8304H_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_SHUNT_RESISTANCE_OHM     BOOSTXL_DRV8304H_PH_SHUNT_RESISTANCE_OHM
#define MY_BOARD_PH_CSA_GAIN_V_V             BOOSTXL_DRV8304H_PH_CSA_GAIN_V_V
#define MY_BOARD_PH_CSA_BIAS_V               BOOSTXL_DRV8304H_PH_CSA_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ    BOOSTXL_DRV8304H_PH_CURRENT_SENSE_POLE_HZ
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN       BOOSTXL_DRV8304H_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V     BOOSTXL_DRV8304H_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ    BOOSTXL_DRV8304H_PH_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN    BOOSTXL_DRV8304H_DCBUS_VOLTAGE_SENSE_GAIN
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V  BOOSTXL_DRV8304H_DCBUS_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ BOOSTXL_DRV8304H_DCBUS_VOLTAGE_SENSE_POLE_HZ

#endif // BOOSTXL_DRV8304H_IS_DEFAULT_PARAM

/** @} */ // end of hal_drv8304h_mapping
//=================================================================================================

/** @} */ // end of hal_boostxl_drv8304h

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_DRV8304H_BOOSTERPACK_H
