/**
 * @file    TI_BOOSTXL_DRV8301.h
 * @brief   Hardware Abstraction Layer (HAL) for the BOOSTXL-DRV8301 motor driver board.
 * @version 2.2
 * @date    2025-08-12
 * @author  Gemini (based on user template and TI manual slvu974.pdf)
 * @note    This file contains the hardware parameters for the Texas Instruments
 * BOOSTXL-DRV8301 Motor Drive BoosterPack. The values are derived from
 * the official hardware user's guide (slvu974.pdf).
 * By defining 'BOOSTXL_DRV8301_IS_DEFAULT_PARAM', these parameters can be
 * mapped to the generic 'MY_BOARD_' macros for general use.
 */

#include <ctl/component/motor_control/controller_preset/controller_preset_general.h>

#ifndef MOTOR_DRIVER_HAL_DRV8301_BOOSTERPACK_H
#define MOTOR_DRIVER_HAL_DRV8301_BOOSTERPACK_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_boostxl_drv8301 BOOSTXL-DRV8301 Hardware Abstraction Layer
 * @ingroup CTL_MC_PRESET
 * @brief Parameters for the TI BOOSTXL-DRV8301 motor driver board.
 * @details reference: https://www.ti.com.cn/tool/cn/BOOSTXL-DRV8301
 * hardware document: https://www.ti.com.cn/lit/ug/slvu974/slvu974.pdf
 * @{
 */

//=================================================================================================
/**
 * @defgroup hal_drv8301_nameplate Nameplate
 * @ingroup hal_boostxl_drv8301
 * @brief This section records the key hardware models of the board for identification and traceability.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_DRV8301_NAME                 "BOOSTXL-DRV8301 REVB"
#define BOOSTXL_DRV8301_GATE_DRIVER_IC       "DRV8301"
#define BOOSTXL_DRV8301_MOSFET_PART_NUMBER   "CSD18533Q5A"
#define BOOSTXL_DRV8301_CURRENT_SENSOR_MODEL "Shunt + DRV8301 Internal Amp / OPA2374"
#define BOOSTXL_DRV8301_THERMAL_SENSOR_MODEL "Internal to DRV8301 (via nOCTW pin)"

/** @} */ // end of hal_drv8301_nameplate
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8301_limits Physical Operating Limits
 * @ingroup hal_boostxl_drv8301
 * @brief This section defines the boundary conditions for the safe physical operation of the board.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_DRV8301_VBUS_MIN_V        (6.0f)  // Minimum DC bus operating voltage (V), per manual section 2.1
#define BOOSTXL_DRV8301_VBUS_MAX_V        (24.0f) // Maximum DC bus operating voltage (V), per manual section 2.1
#define BOOSTXL_DRV8301_CURRENT_MAX_RMS_A (10.0f) // Maximum continuous phase current (RMS, A), per manual section 2.1
#define BOOSTXL_DRV8301_CURRENT_MAX_PEAK_A                                                                             \
    (14.0f) // Maximum allowed peak phase current (Peak, A), per manual section 2.1
#define BOOSTXL_DRV8301_TEMP_MAX_C                                                                                     \
    (85.0f) // Recommended maximum operating temperature for the power stage (°„C). Not specified, using a safe default.

/** @} */ // end of hal_drv8301_limits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8301_topology Sensing Topology
 * @ingroup hal_boostxl_drv8301
 * @brief This section describes the type and layout of the onboard sensors.
 * @{
 */
//-------------------------------------------------------------------------------------------------

#define BOOSTXL_DRV8301_PH_CURRENT_SENSE_TYPE     (1) // SENSOR_TYPE_SHUNT (Low-side shunts, manual section 5.2)
#define BOOSTXL_DRV8301_PH_CURRENT_SENSE_TOPOLOGY (1) // CS_TOPOLOGY_LOW_SIDE (Per phase low-side sensing)
#define BOOSTXL_DRV8301_PH_VOLTAGE_SENSE_TYPE     (1) // VS_TYPE_PHASE_GND (Resistive divider to GND, manual section 5.1)
#define BOOSTXL_DRV8301_DCBUS_VOLTAGE_SENSE_TYPE  (1) // VS_TYPE_PHASE_GND (Resistive divider to GND, manual section 5.1)
#define BOOSTXL_DRV8301_DCBUS_CURRENT_SENSE_TYPE  (0) // SENSOR_NONE (No DC bus current sensor present)
#define BOOSTXL_DRV8301_THERMAL_SENSE_TYPE        (0) // SENSOR_NONE (No direct analog thermal sensor for MCU, only fault pin)

/** @} */ // end of hal_drv8301_topology
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8301_circuits Sensing Circuit Parameters
 * @ingroup hal_boostxl_drv8301
 * @brief This section defines the detailed electrical parameters of all sensing circuits.
 * @{
 */
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
#if (BOOSTXL_DRV8301_PH_CURRENT_SENSE_TYPE == 1)        // SENSOR_TYPE_SHUNT
#define BOOSTXL_DRV8301_PH_SHUNT_RESISTANCE_OHM (0.01f) // Resistance of the shunt resistor (R34), Ohm.
#define BOOSTXL_DRV8301_PH_CSA_GAIN_V_V                                                                                \
    (10.0f) // Gain of the Current Sense Amplifier (V/V). Set to 10 for both DRV8301 internal and external OPA2374 (R43/R44).
#define BOOSTXL_DRV8301_PH_CSA_BIAS_V (1.65f) // Bias voltage of the amplifier's output (V), 3.3V / 2.
#endif
#define BOOSTXL_DRV8301_PH_CURRENT_SENSE_POLE_HZ                                                                       \
    (79500.0f) // Filter bandwidth. Calculated from schematic: 1/(2*pi*(R44+R45)*C31) = 1/(2*pi*2k*1nF)

//--- 4.2: Phase Voltage Sensing ---
#if (BOOSTXL_DRV8301_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define BOOSTXL_DRV8301_PH_VOLTAGE_SENSE_GAIN                                                                          \
    (0.1254f) // Gain of the voltage sensing circuit (V/V), R38 / (R37 + R38) = 4.99k / (34.8k + 4.99k)
#define BOOSTXL_DRV8301_PH_VOLTAGE_SENSE_BIAS_V                                                                        \
    (0.0f) // Bias of the voltage sensing circuit (V), passive divider to GND.
#define BOOSTXL_DRV8301_PH_VOLTAGE_SENSE_POLE_HZ                                                                       \
    (364.692f) // Filter bandwidth of the signal path (Hz), as stated in manual section 5.1.
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (BOOSTXL_DRV8301_DCBUS_VOLTAGE_SENSE_TYPE != 0)            // Not SENSOR_NONE
#define BOOSTXL_DRV8301_DCBUS_VOLTAGE_SENSE_GAIN    (0.1254f)  // Gain of the voltage sensing circuit (V/V)
#define BOOSTXL_DRV8301_DCBUS_VOLTAGE_SENSE_BIAS_V  (0.0f)     // Bias of the voltage sensing circuit (V)
#define BOOSTXL_DRV8301_DCBUS_VOLTAGE_SENSE_POLE_HZ (364.692f) // Filter bandwidth of the signal path (Hz)
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (BOOSTXL_DRV8301_DCBUS_CURRENT_SENSE_TYPE == 1)           // SENSOR_TYPE_SHUNT
#define BOOSTXL_DRV8301_DCBUS_SHUNT_RESISTANCE_OHM  (0.0f)    // Not applicable
#define BOOSTXL_DRV8301_DCBUS_CSA_GAIN_V_V          (0.0f)    // Not applicable
#define BOOSTXL_DRV8301_DCBUS_CSA_BIAS_V            (0.0f)    // Not applicable
#define BOOSTXL_DRV8301_DCBUS_CURRENT_SENSE_POLE_HZ (0.0f)    // Not applicable
#elif (BOOSTXL_DRV8301_DCBUS_CURRENT_SENSE_TYPE == 2)         // SENSOR_TYPE_HALL
#define BOOSTXL_DRV8301_DCBUS_CURRENT_SENSITIVITY_MV_A (0.0f) // Not applicable
#define BOOSTXL_DRV8301_DCBUS_CURRENT_ZERO_BIAS_V      (0.0f) // Not applicable
#define BOOSTXL_DRV8301_DCBUS_CURRENT_SENSE_POLE_HZ    (0.0f) // Not applicable
#endif

//--- 4.5: Thermal Sensing ---
#if (BOOSTXL_DRV8301_THERMAL_SENSE_TYPE == 1)                // THERMAL_SENSOR_NTC
#define BOOSTXL_DRV8301_THERMAL_PULLUP_RESISTANCE_OHM (0.0f) // Not applicable
#define BOOSTXL_DRV8301_THERMAL_NTC_BETA_VALUE        (0.0f) // Not applicable
#define BOOSTXL_DRV8301_THERMAL_NTC_NOMINAL_R_OHM     (0.0f) // Not applicable
#define BOOSTXL_DRV8301_THERMAL_NTC_NOMINAL_TEMP_C    (0.0f) // Not applicable
#elif (BOOSTXL_DRV8301_THERMAL_SENSE_TYPE == 3)              // THERMAL_SENSOR_IC
#define BOOSTXL_DRV8301_THERMAL_IC_SENSITIVITY_MV_C (0.0f)   // Not applicable
#define BOOSTXL_DRV8301_THERMAL_IC_OFFSET_V         (0.0f)   // Not applicable
#endif

/** @} */ // end of hal_drv8301_circuits
//=================================================================================================

//=================================================================================================
/**
 * @defgroup hal_drv8301_mapping Default Parameter Mapping
 * @ingroup hal_boostxl_drv8301
 * @brief This section maps the board-specific parameters to the generic 'MY_BOARD_' macros.
 * @{
 */
//-------------------------------------------------------------------------------------------------
#if defined(BOOSTXL_DRV8301_IS_DEFAULT_PARAM)

// Nameplate
#define MY_BOARD_NAME                 BOOSTXL_DRV8301_NAME
#define MY_BOARD_GATE_DRIVER_IC       BOOSTXL_DRV8301_GATE_DRIVER_IC
#define MY_BOARD_MOSFET_PART_NUMBER   BOOSTXL_DRV8301_MOSFET_PART_NUMBER
#define MY_BOARD_CURRENT_SENSOR_MODEL BOOSTXL_DRV8301_CURRENT_SENSOR_MODEL
#define MY_BOARD_THERMAL_SENSOR_MODEL BOOSTXL_DRV8301_THERMAL_SENSOR_MODEL

// Operating Limits
#define MY_BOARD_VBUS_MIN_V         BOOSTXL_DRV8301_VBUS_MIN_V
#define MY_BOARD_VBUS_MAX_V         BOOSTXL_DRV8301_VBUS_MAX_V
#define MY_BOARD_CURRENT_MAX_RMS_A  BOOSTXL_DRV8301_CURRENT_MAX_RMS_A
#define MY_BOARD_CURRENT_MAX_PEAK_A BOOSTXL_DRV8301_CURRENT_MAX_PEAK_A
#define MY_BOARD_TEMP_MAX_C         BOOSTXL_DRV8301_TEMP_MAX_C

// Sensing Topology
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     BOOSTXL_DRV8301_PH_CURRENT_SENSE_TYPE
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY BOOSTXL_DRV8301_PH_CURRENT_SENSE_TOPOLOGY
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     BOOSTXL_DRV8301_PH_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  BOOSTXL_DRV8301_DCBUS_VOLTAGE_SENSE_TYPE
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  BOOSTXL_DRV8301_DCBUS_CURRENT_SENSE_TYPE
#define MY_BOARD_THERMAL_SENSE_TYPE        BOOSTXL_DRV8301_THERMAL_SENSE_TYPE

// Sensing Circuit Parameters
#define MY_BOARD_PH_SHUNT_RESISTANCE_OHM     BOOSTXL_DRV8301_PH_SHUNT_RESISTANCE_OHM
#define MY_BOARD_PH_CSA_GAIN_V_V             BOOSTXL_DRV8301_PH_CSA_GAIN_V_V
#define MY_BOARD_PH_CSA_BIAS_V               BOOSTXL_DRV8301_PH_CSA_BIAS_V
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ    BOOSTXL_DRV8301_PH_CURRENT_SENSE_POLE_HZ
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN       BOOSTXL_DRV8301_PH_VOLTAGE_SENSE_GAIN
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V     BOOSTXL_DRV8301_PH_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ    BOOSTXL_DRV8301_PH_VOLTAGE_SENSE_POLE_HZ
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN    BOOSTXL_DRV8301_DCBUS_VOLTAGE_SENSE_GAIN
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V  BOOSTXL_DRV8301_DCBUS_VOLTAGE_SENSE_BIAS_V
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ BOOSTXL_DRV8301_DCBUS_VOLTAGE_SENSE_POLE_HZ

#endif // BOOSTXL_DRV8301_IS_DEFAULT_PARAM

/** @} */ // end of hal_drv8301_mapping
//=================================================================================================

/** @} */ // end of hal_boostxl_drv8301

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_DRV8301_BOOSTERPACK_H
