/**
 * @file    drv8305_boosterpack.h
 * @brief   Hardware Abstraction Layer (HAL) for the BOOSTXL-DRV8305EVM motor driver board.
 * @version 1.0
 * @date    2025-08-12
 * @author  Gemini (based on user template and TI manual slvuai8a.pdf)
 * @note    This file contains the hardware parameters for the Texas Instruments
 * BOOSTXL-DRV8305EVM Motor Drive BoosterPack. The values are derived from
 * the official hardware user's guide (slvuai8a.pdf).
 * By defining 'BOOSTXL_DRV8305_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/motor_control/controller_preset/controller_preset_general.h>

#ifndef MOTOR_DRIVER_HAL_DRV8305_BOOSTERPACK_H
#define MOTOR_DRIVER_HAL_DRV8305_BOOSTERPACK_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_boostxl_drv8305 BOOSTXL-DRV8305EVM Hardware Abstraction Layer
 * @brief Parameters for the TI BOOSTXL-DRV8305EVM motor driver board.
 * @details reference: https://www.ti.com.cn/tool/cn/BOOSTXL-DRV8305EVM
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_drv8305_nameplate Nameplate
 * @ingroup hal_boostxl_drv8305
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_DRV8305_NAME                 "BOOSTXL-DRV8305EVM"
#define BOOSTXL_DRV8305_GATE_DRIVER_IC       "DRV8305"
#define BOOSTXL_DRV8305_MOSFET_PART_NUMBER   "CSD18540Q5B"
#define BOOSTXL_DRV8305_CURRENT_SENSOR_MODEL "Shunt + DRV8305 Internal Amps"
#define BOOSTXL_DRV8305_THERMAL_SENSOR_MODEL "Internal to DRV8305 (via nFAULT/SPI)"

/** @} */ // end of hal_drv8305_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8305_limits Physical Operating Limits
 * @ingroup hal_boostxl_drv8305
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_DRV8305_VBUS_MIN_V        (4.4f)  // Minimum DC bus operating voltage (V), per manual section 2.3
#define BOOSTXL_DRV8305_VBUS_MAX_V        (45.0f) // Maximum DC bus operating voltage (V), per manual section 2.3
#define BOOSTXL_DRV8305_CURRENT_MAX_RMS_A (15.0f) // Maximum continuous phase current (RMS, A), per manual section 2.1
#define BOOSTXL_DRV8305_CURRENT_MAX_PEAK_A                                                                             \
    (20.0f)                                 // Maximum allowed peak phase current (Peak, A), per manual section 2.1
#define BOOSTXL_DRV8305_TEMP_MAX_C (125.0f) // Maximum operating temperature (°„C), per manual section 2.3

/** @} */ // end of hal_drv8305_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8305_topology Sensing Topology
 * @ingroup hal_boostxl_drv8305
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_DRV8305_PH_CURRENT_SENSE_TYPE     (1) // SENSOR_TYPE_SHUNT (Low-side shunts, manual section 5.2)
#define BOOSTXL_DRV8305_PH_CURRENT_SENSE_TOPOLOGY (1) // CS_TOPOLOGY_LOW_SIDE (Per phase low-side sensing)
#define BOOSTXL_DRV8305_PH_VOLTAGE_SENSE_TYPE     (1) // VS_TYPE_PHASE_GND (Resistive divider to GND, manual section 5.1)
#define BOOSTXL_DRV8305_DCBUS_VOLTAGE_SENSE_TYPE  (1) // VS_TYPE_PHASE_GND (Resistive divider to GND, manual section 5.1)
#define BOOSTXL_DRV8305_DCBUS_CURRENT_SENSE_TYPE  (0) // SENSOR_NONE (No DC bus current sensor present)
#define BOOSTXL_DRV8305_THERMAL_SENSE_TYPE        (0) // SENSOR_NONE (No direct analog thermal sensor for MCU, only fault pin)

/** @} */ // end of hal_drv8305_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8305_circuits Sensing Circuit Parameters
 * @ingroup hal_boostxl_drv8305
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
#if (BOOSTXL_DRV8305_PH_CURRENT_SENSE_TYPE == 1) // SENSOR_TYPE_SHUNT
#define BOOSTXL_DRV8305_PH_SHUNT_RESISTANCE_OHM                                                                        \
    (0.007f)                                    // Resistance of the shunt resistor (Ohm), per manual section 5.2
#define BOOSTXL_DRV8305_PH_CSA_GAIN_V_V (10.0f) // Gain of the Current Sense Amplifier (V/V), per manual section 5.2
#define BOOSTXL_DRV8305_PH_CSA_BIAS_V   (1.65f) // Bias voltage of the amplifier's output (V), per manual section 5.2
#endif
#define BOOSTXL_DRV8305_PH_CURRENT_SENSE_POLE_HZ                                                                       \
    (150.0e3f) // Bandwidth of the internal CSA is high, using a safe default.

//--- 4.2: Phase Voltage Sensing ---
#if (BOOSTXL_DRV8305_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define BOOSTXL_DRV8305_PH_VOLTAGE_SENSE_GAIN                                                                          \
    (0.074488f) // Gain of the voltage sensing circuit (V/V), R_low / (R_high + R_low) = 4.99k / (62.0k + 4.99k)
#define BOOSTXL_DRV8305_PH_VOLTAGE_SENSE_BIAS_V                                                                        \
    (0.0f) // Bias of the voltage sensing circuit (V), passive divider to GND.
#define BOOSTXL_DRV8305_PH_VOLTAGE_SENSE_POLE_HZ                                                                       \
    (344.6f) // Filter bandwidth, 1/(2*pi*R_thevenin*C) = 1/(2*pi*4.618k*0.1uF)
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (BOOSTXL_DRV8305_DCBUS_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define BOOSTXL_DRV8305_DCBUS_VOLTAGE_SENSE_GAIN                                                                       \
    (0.074488f) // Gain of the voltage sensing circuit (V/V), same as phase sense
#define BOOSTXL_DRV8305_DCBUS_VOLTAGE_SENSE_BIAS_V (0.0f) // Bias of the voltage sensing circuit (V)
#define BOOSTXL_DRV8305_DCBUS_VOLTAGE_SENSE_POLE_HZ                                                                    \
    (344.6f) // Filter bandwidth of the signal path (Hz), same as phase sense
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (BOOSTXL_DRV8305_DCBUS_CURRENT_SENSE_TYPE != 0)
// Not applicable for this board
#endif

//--- 4.5: Thermal Sensing ---
#if (BOOSTXL_DRV8305_THERMAL_SENSE_TYPE != 0)
// Not applicable for this board
#endif

/** @} */ // end of hal_drv8305_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8305_mapping Default Parameter Mapping
 * @ingroup hal_boostxl_drv8305
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(BOOSTXL_DRV8305_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 BOOSTXL_DRV8305_NAME
#define MY_BOARD_GATE_DRIVER_IC       BOOSTXL_DRV8305_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   BOOSTXL_DRV8305_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL BOOSTXL_DRV8305_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL BOOSTXL_DRV8305_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         BOOSTXL_DRV8305_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         BOOSTXL_DRV8305_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  BOOSTXL_DRV8305_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A BOOSTXL_DRV8305_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         BOOSTXL_DRV8305_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     BOOSTXL_DRV8305_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY BOOSTXL_DRV8305_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     BOOSTXL_DRV8305_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  BOOSTXL_DRV8305_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  BOOSTXL_DRV8305_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        BOOSTXL_DRV8305_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_SHUNT_RESISTANCE_OHM     BOOSTXL_DRV8305_PH_SHUNT_RESISTANCE_OHM
#define MY_BOARD_PH_CSA_GAIN_V_V             BOOSTXL_DRV8305_PH_CSA_GAIN_V_V
#define MY_BOARD_PH_CSA_BIAS_V               BOOSTXL_DRV8305_PH_CSA_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ    BOOSTXL_DRV8305_PH_CURRENT_SENSE_POLE_HZ
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN       BOOSTXL_DRV8305_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V     BOOSTXL_DRV8305_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ    BOOSTXL_DRV8305_PH_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN    BOOSTXL_DRV8305_DCBUS_VOLTAGE_SENSE_GAIN
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V  BOOSTXL_DRV8305_DCBUS_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ BOOSTXL_DRV8305_DCBUS_VOLTAGE_SENSE_POLE_HZ

#endif // BOOSTXL_DRV8305_IS_DEFAULT_PARAM

/** @} */ // end of hal_drv8305_mapping
//=================================================================================================

/** @} */ // end of hal_boostxl_drv8305

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_DRV8305_BOOSTERPACK_H
