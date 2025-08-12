/**
 * @file    drv8320rs_boosterpack.h
 * @brief   Hardware Abstraction Layer (HAL) for the BOOSTXL-DRV8320RS motor driver board.
 * @version 1.0
 * @date    2025-08-12
 * @author  Gemini (based on user template and TI manual zhcu828b.pdf)
 * @note    This file contains the hardware parameters for the Texas Instruments
 * BOOSTXL-DRV8320RS Motor Drive BoosterPack. The values are derived from
 * the official hardware user's guide (zhcu828b.pdf).
 * By defining 'BOOSTXL_DRV8320RS_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/motor_control/controller_preset/controller_preset_general.h>

#ifndef MOTOR_DRIVER_HAL_DRV8320RS_BOOSTERPACK_H
#define MOTOR_DRIVER_HAL_DRV8320RS_BOOSTERPACK_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_boostxl_drv8320rs BOOSTXL-DRV8320RS Hardware Abstraction Layer
 * @brief Parameters for the TI BOOSTXL-DRV8320RS motor driver board.
 * @details reference https://www.ti.com.cn/tool/cn/BOOSTXL-DRV8320RS
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_drv8320rs_nameplate Nameplate
 * @ingroup hal_boostxl_drv8320rs
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_DRV8320RS_NAME                 "BOOSTXL-DRV8320RS"
#define BOOSTXL_DRV8320RS_GATE_DRIVER_IC       "DRV8320RS"
#define BOOSTXL_DRV8320RS_MOSFET_PART_NUMBER   "CSD88584Q5DC"
#define BOOSTXL_DRV8320RS_CURRENT_SENSOR_MODEL "Low-side Shunt (Amplification via MCU PGA)"
#define BOOSTXL_DRV8320RS_THERMAL_SENSOR_MODEL "Internal to DRV8320RS (via nFAULT/SPI)"

/** @} */ // end of hal_drv8320rs_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8320rs_limits Physical Operating Limits
 * @ingroup hal_boostxl_drv8320rs
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_DRV8320RS_VBUS_MIN_V        (6.0f)  // Minimum DC bus operating voltage (V), per manual section 2.1
#define BOOSTXL_DRV8320RS_VBUS_MAX_V        (54.0f) // Maximum DC bus operating voltage (V), per manual section 2.1
#define BOOSTXL_DRV8320RS_CURRENT_MAX_RMS_A (10.0f) // Maximum continuous phase current (RMS, A), derived from peak
#define BOOSTXL_DRV8320RS_CURRENT_MAX_PEAK_A                                                                           \
    (15.0f)                                  // Maximum allowed peak phase current (Peak, A), per manual section 2.1
#define BOOSTXL_DRV8320RS_TEMP_MAX_C (85.0f) // Recommended maximum PCB operating temperature (°„C), safe default

/** @} */ // end of hal_drv8320rs_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8320rs_topology Sensing Topology
 * @ingroup hal_boostxl_drv8320rs
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_DRV8320RS_PH_CURRENT_SENSE_TYPE     (1) // SENSOR_TYPE_SHUNT (Low-side shunts)
#define BOOSTXL_DRV8320RS_PH_CURRENT_SENSE_TOPOLOGY (1) // CS_TOPOLOGY_LOW_SIDE (Low-side phase current sensing)
#define BOOSTXL_DRV8320RS_PH_VOLTAGE_SENSE_TYPE     (1) // VS_TYPE_PHASE_GND (Resistive divider to GND)
#define BOOSTXL_DRV8320RS_DCBUS_VOLTAGE_SENSE_TYPE  (1) // VS_TYPE_PHASE_GND (Resistive divider to GND)
#define BOOSTXL_DRV8320RS_DCBUS_CURRENT_SENSE_TYPE  (0) // SENSOR_NONE (No DC bus current sensor present)
#define BOOSTXL_DRV8320RS_THERMAL_SENSE_TYPE                                                                           \
    (0) // SENSOR_NONE (No direct analog thermal sensor for MCU, only fault pin)

/** @} */ // end of hal_drv8320rs_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8320rs_circuits Sensing Circuit Parameters
 * @ingroup hal_boostxl_drv8320rs
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
// Note: This board uses low-side shunts, but the amplification is handled by the Programmable
// Gain Amplifiers (PGA) inside the C2000 MCU, not by an amplifier on the driver board.
// The gain/bias parameters are set to unity/zero here, as they are configured in the MCU.
#if (BOOSTXL_DRV8320RS_PH_CURRENT_SENSE_TYPE == 1) // SENSOR_TYPE_SHUNT
#define BOOSTXL_DRV8320RS_PH_SHUNT_RESISTANCE_OHM                                                                      \
    (0.002f) // Resistance of the shunt resistor (Ohm), assumed from typical design (2m¶∏).
#define BOOSTXL_DRV8320RS_PH_CSA_GAIN_V_V (1.0f) // Gain is handled by MCU's internal PGA.
#define BOOSTXL_DRV8320RS_PH_CSA_BIAS_V   (0.0f) // Bias is handled by MCU's internal PGA.
#endif
#define BOOSTXL_DRV8320RS_PH_CURRENT_SENSE_POLE_HZ                                                                     \
    (250.0e3f) // Bandwidth is primarily limited by MCU's ADC/PGA, using a safe default.

//--- 4.2: Phase Voltage Sensing ---
// Note: Resistor values are based on typical designs for this board as they are not listed in the manual.
#if (BOOSTXL_DRV8320RS_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define BOOSTXL_DRV8320RS_PH_VOLTAGE_SENSE_GAIN                                                                        \
    (0.053f) // Gain of the voltage sensing circuit (V/V), assumed R_high=100k, R_low=5.6k
#define BOOSTXL_DRV8320RS_PH_VOLTAGE_SENSE_BIAS_V                                                                      \
    (0.0f) // Bias of the voltage sensing circuit (V), passive divider to GND.
#define BOOSTXL_DRV8320RS_PH_VOLTAGE_SENSE_POLE_HZ (300.0f) // Approx. filter bandwidth with 0.1uF cap
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (BOOSTXL_DRV8320RS_DCBUS_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define BOOSTXL_DRV8320RS_DCBUS_VOLTAGE_SENSE_GAIN                                                                     \
    (0.053f) // Gain of the voltage sensing circuit (V/V), same as phase sense
#define BOOSTXL_DRV8320RS_DCBUS_VOLTAGE_SENSE_BIAS_V (0.0f) // Bias of the voltage sensing circuit (V)
#define BOOSTXL_DRV8320RS_DCBUS_VOLTAGE_SENSE_POLE_HZ                                                                  \
    (300.0f) // Filter bandwidth of the signal path (Hz), same as phase sense
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (BOOSTXL_DRV8320RS_DCBUS_CURRENT_SENSE_TYPE != 0)
// Not applicable for this board
#endif

//--- 4.5: Thermal Sensing ---
#if (BOOSTXL_DRV8320RS_THERMAL_SENSE_TYPE != 0)
// Not applicable for this board
#endif

/** @} */ // end of hal_drv8320rs_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8320rs_mapping Default Parameter Mapping
 * @ingroup hal_boostxl_drv8320rs
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(BOOSTXL_DRV8320RS_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 BOOSTXL_DRV8320RS_NAME
#define MY_BOARD_GATE_DRIVER_IC       BOOSTXL_DRV8320RS_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   BOOSTXL_DRV8320RS_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL BOOSTXL_DRV8320RS_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL BOOSTXL_DRV8320RS_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         BOOSTXL_DRV8320RS_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         BOOSTXL_DRV8320RS_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  BOOSTXL_DRV8320RS_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A BOOSTXL_DRV8320RS_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         BOOSTXL_DRV8320RS_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     BOOSTXL_DRV8320RS_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY BOOSTXL_DRV8320RS_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     BOOSTXL_DRV8320RS_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  BOOSTXL_DRV8320RS_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  BOOSTXL_DRV8320RS_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        BOOSTXL_DRV8320RS_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_SHUNT_RESISTANCE_OHM     BOOSTXL_DRV8320RS_PH_SHUNT_RESISTANCE_OHM
#define MY_BOARD_PH_CSA_GAIN_V_V             BOOSTXL_DRV8320RS_PH_CSA_GAIN_V_V
#define MY_BOARD_PH_CSA_BIAS_V               BOOSTXL_DRV8320RS_PH_CSA_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ    BOOSTXL_DRV8320RS_PH_CURRENT_SENSE_POLE_HZ
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN       BOOSTXL_DRV8320RS_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V     BOOSTXL_DRV8320RS_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ    BOOSTXL_DRV8320RS_PH_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN    BOOSTXL_DRV8320RS_DCBUS_VOLTAGE_SENSE_GAIN
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V  BOOSTXL_DRV8320RS_DCBUS_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ BOOSTXL_DRV8320RS_DCBUS_VOLTAGE_SENSE_POLE_HZ

#endif // BOOSTXL_DRV8320RS_IS_DEFAULT_PARAM

/** @} */ // end of hal_drv8320rs_mapping
//=================================================================================================

/** @} */ // end of hal_boostxl_drv8320rs

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_DRV8320RS_BOOSTERPACK_H
