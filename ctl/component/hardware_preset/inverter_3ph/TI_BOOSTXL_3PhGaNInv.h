/**
 * @file    3phganinv_boosterpack.h
 * @brief   Hardware Abstraction Layer (HAL) for the BOOSTXL-3PhGaNInv motor driver board.
 * @version 1.0
 * @date    2025-08-12
 * @author  Gemini (based on user template and TI manual sluubp1a.pdf)
 * @note    This file contains the hardware parameters for the Texas Instruments
 * BOOSTXL-3PhGaNInv Three-Phase GaN Inverter. The values are derived from
 * the official hardware user's guide (sluubp1a.pdf).
 * By defining 'BOOSTXL_3PHGANINV_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/hardware_preset/inverter_3ph/inverter_3ph_general.h>

#ifndef MOTOR_DRIVER_HAL_3PHGANINV_BOOSTERPACK_H
#define MOTOR_DRIVER_HAL_3PHGANINV_BOOSTERPACK_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_boostxl_3phganinv BOOSTXL-3PhGaNInv Hardware Abstraction Layer
 * @ingroup CTL_MC_PRESET
 * @brief Parameters for the TI BOOSTXL-3PhGaNInv motor driver board.
 * @details reference https://www.ti.com/tool/BOOSTXL-3PHGANINV
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_3phganinv_nameplate Nameplate
 * @ingroup hal_boostxl_3phganinv
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_3PHGANINV_NAME                 "BOOSTXL-3PhGaNInv"
#define BOOSTXL_3PHGANINV_GATE_DRIVER_IC       "LMG5200 (Integrated GaN Power Stage)"
#define BOOSTXL_3PHGANINV_MOSFET_PART_NUMBER   "LMG5200"
#define BOOSTXL_3PHGANINV_CURRENT_SENSOR_MODEL "In-line Shunt + INA240"
#define BOOSTXL_3PHGANINV_THERMAL_SENSOR_MODEL "TMP302 (PCB Overtemp Alert)"

/** @} */ // end of hal_3phganinv_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_3phganinv_limits Physical Operating Limits
 * @ingroup hal_boostxl_3phganinv
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_3PHGANINV_VBUS_MIN_V        (12.0f) // Minimum DC bus operating voltage (V), per manual section 1.1
#define BOOSTXL_3PHGANINV_VBUS_MAX_V        (60.0f) // Maximum DC bus operating voltage (V), per manual section 1.1
#define BOOSTXL_3PHGANINV_CURRENT_MAX_RMS_A (7.0f)  // Maximum continuous phase current (RMS, A), derived from peak
#define BOOSTXL_3PHGANINV_CURRENT_MAX_PEAK_A                                                                           \
    (10.0f)                                  // Maximum allowed peak phase current (Peak, A), per manual section 1.1
#define BOOSTXL_3PHGANINV_TEMP_MAX_C (85.0f) // Recommended maximum PCB operating temperature (°„C), safe default

/** @} */ // end of hal_3phganinv_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_3phganinv_topology Sensing Topology
 * @ingroup hal_boostxl_3phganinv
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_3PHGANINV_PH_CURRENT_SENSE_TYPE     (1) // SENSOR_TYPE_SHUNT (In-line shunts, manual section 1.3.1)
#define BOOSTXL_3PHGANINV_PH_CURRENT_SENSE_TOPOLOGY (3) // CS_TOPOLOGY_INLINE (In-line phase current sensing)
#define BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_TYPE     (1) // VS_TYPE_PHASE_GND (Resistive divider to GND)
#define BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_TYPE  (1) // VS_TYPE_PHASE_GND (Resistive divider to GND)
#define BOOSTXL_3PHGANINV_DCBUS_CURRENT_SENSE_TYPE  (0) // SENSOR_NONE (No DC bus current sensor present)
#define BOOSTXL_3PHGANINV_THERMAL_SENSE_TYPE                                                                           \
    (0) // SENSOR_NONE (No direct analog thermal sensor for MCU, only fault pin)

/** @} */ // end of hal_3phganinv_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_3phganinv_circuits Sensing Circuit Parameters
 * @ingroup hal_boostxl_3phganinv
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
#if (BOOSTXL_3PHGANINV_PH_CURRENT_SENSE_TYPE == 1) // SENSOR_TYPE_SHUNT
#define BOOSTXL_3PHGANINV_PH_SHUNT_RESISTANCE_OHM                                                                      \
    (0.005f) // Resistance of the shunt resistor (Ohm), per manual section 1.1
#define BOOSTXL_3PHGANINV_PH_CSA_GAIN_V_V                                                                              \
    (20.0f) // Gain of the INA240 Current Sense Amplifier (V/V), per manual section 1.3.2
#define BOOSTXL_3PHGANINV_PH_CSA_BIAS_V (1.65f) // Bias voltage of the amplifier's output (V), per manual section 1.3.2
#endif
#define BOOSTXL_3PHGANINV_PH_CURRENT_SENSE_POLE_HZ (250.0e3f) // Bandwidth of the INA240 is high, using a safe default.

//--- 4.2: Phase Voltage Sensing ---
// Note: Resistor values (100k/6.8k) are based on typical designs for this board as they are not listed in the manual.
#if (BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_GAIN                                                                        \
    (0.06367f) // Gain of the voltage sensing circuit (V/V), R_low / (R_high + R_low) = 6.8k / (100k + 6.8k)
#define BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_BIAS_V                                                                      \
    (0.0f) // Bias of the voltage sensing circuit (V), passive divider to GND.
#define BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_POLE_HZ (250.0f) // Approx. filter bandwidth with 0.1uF cap
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_GAIN                                                                     \
    (0.06367f) // Gain of the voltage sensing circuit (V/V), same as phase sense
#define BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_BIAS_V (0.0f) // Bias of the voltage sensing circuit (V)
#define BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_POLE_HZ                                                                  \
    (250.0f) // Filter bandwidth of the signal path (Hz), same as phase sense
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (BOOSTXL_3PHGANINV_DCBUS_CURRENT_SENSE_TYPE != 0)
// Not applicable for this board
#endif

//--- 4.5: Thermal Sensing ---
#if (BOOSTXL_3PHGANINV_THERMAL_SENSE_TYPE != 0)
// Not applicable for this board
#endif

/** @} */ // end of hal_3phganinv_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_3phganinv_mapping Default Parameter Mapping
 * @ingroup hal_boostxl_3phganinv
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(BOOSTXL_3PHGANINV_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 BOOSTXL_3PHGANINV_NAME
#define MY_BOARD_GATE_DRIVER_IC       BOOSTXL_3PHGANINV_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   BOOSTXL_3PHGANINV_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL BOOSTXL_3PHGANINV_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL BOOSTXL_3PHGANINV_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         BOOSTXL_3PHGANINV_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         BOOSTXL_3PHGANINV_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  BOOSTXL_3PHGANINV_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A BOOSTXL_3PHGANINV_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         BOOSTXL_3PHGANINV_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     BOOSTXL_3PHGANINV_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY BOOSTXL_3PHGANINV_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  BOOSTXL_3PHGANINV_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        BOOSTXL_3PHGANINV_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_SHUNT_RESISTANCE_OHM     BOOSTXL_3PHGANINV_PH_SHUNT_RESISTANCE_OHM
#define MY_BOARD_PH_CSA_GAIN_V_V             BOOSTXL_3PHGANINV_PH_CSA_GAIN_V_V
#define MY_BOARD_PH_CSA_BIAS_V               BOOSTXL_3PHGANINV_PH_CSA_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ    BOOSTXL_3PHGANINV_PH_CURRENT_SENSE_POLE_HZ
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN       BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V     BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ    BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN    BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_GAIN
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V  BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_POLE_HZ

#endif // BOOSTXL_3PHGANINV_IS_DEFAULT_PARAM

/** @} */ // end of hal_3phganinv_mapping
//=================================================================================================

/** @} */ // end of hal_boostxl_3phganinv

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_3PHGANINV_BOOSTERPACK_H
