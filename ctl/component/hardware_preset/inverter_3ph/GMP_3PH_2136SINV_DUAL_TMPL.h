/**
 * @file    GMP_3PH_2136SINV_DUAL_TMPL.h
 * @brief   A generic Hardware Abstraction Layer (HAL) template for motor driver boards.
 * @version 2.0
 * @date    2025-08-12
 * @author  javnson(javnson@zju.edu.cn)
 * @note    This template is intended to provide a standard structure for defining hardware
 * parameters for any motor driver board. It is optimized for Doxygen documentation
 * generation while preventing namespace pollution.
 *
 * To use it, copy this file, rename it for your specific board (e.g., "my_driver_board.h"),
 * and then fill in all macro definitions according to your board's hardware manual.
 */

 #include <ctl/component/motor_control/controller_preset/controller_preset_general.h>

#ifndef GMP_3PH_2136SINV_DUAL_TMPL_H
#define GMP_3PH_2136SINV_DUAL_TMPL_H

#ifdef __cplusplus
extern "C"
{
#endif

//=================================================================================================
// Section 1: Hardware Nameplate
// Description: This section is for recording the key hardware models of the board,
//              which is useful for identification and traceability.
//-------------------------------------------------------------------------------------------------

#define MY_BOARD_NAME                 "GMP 3PH 2136SINV DUAL"
#define MY_BOARD_GATE_DRIVER_IC       "IR2136S"
#define MY_BOARD_MOSFET_PART_NUMBER   "BSC040N10NS5"
#define MY_BOARD_CURRENT_SENSOR_MODEL "Shunt + MCP604"
#define MY_BOARD_THERMAL_SENSOR_MODEL "NULL"

//=================================================================================================
// Section 2: Physical Operating Limits
// Description: This section defines the boundary conditions for the safe physical
//              operation of the board.
//-------------------------------------------------------------------------------------------------

#define MY_BOARD_VBUS_MIN_V         (10.0f)  // Minimum DC bus operating voltage (V)
#define MY_BOARD_VBUS_MAX_V         (100.0f) // Maximum DC bus operating voltage (V)
#define MY_BOARD_CURRENT_MAX_RMS_A  (30.0f) // Maximum continuous phase current (RMS, A)
#define MY_BOARD_CURRENT_MAX_PEAK_A (50.0f) // Maximum allowed peak phase current (Peak, A)
#define MY_BOARD_TEMP_MAX_C         (85.0f) // Maximum allowed operating temperature of the power stage (��C)

//=================================================================================================
// Section 3: Sensing Topology Configuration
// Description: This section describes the type and layout of the onboard sensors.
//              The upper-level software will select appropriate algorithms based on these settings.
//-------------------------------------------------------------------------------------------------


//--- 3.1: Select Topology Configuration for Your Board ---
// Description: Choose and define the correct topology for your board below.
//              Use the integer values. The corresponding names are for documentation.
#define MY_BOARD_PH_CURRENT_SENSE_TYPE     (1) // SENSOR_TYPE_SHUNT
#define MY_BOARD_PH_CURRENT_SENSE_TOPOLOGY (1) // CS_TOPOLOGY_LOW_SIDE
#define MY_BOARD_PH_VOLTAGE_SENSE_TYPE     (1) // VS_TYPE_PHASE_GND
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE  (1) // VS_TYPE_PHASE_GND
#define MY_BOARD_DCBUS_CURRENT_SENSE_TYPE  (0) // SENSOR_NONE
#define MY_BOARD_THERMAL_SENSE_TYPE        (1) // THERMAL_SENSOR_NTC

//=================================================================================================
// Section 4: Sensing Circuit Parameters
// Description: This section defines the detailed electrical parameters of all sensing circuits.
//-------------------------------------------------------------------------------------------------

//--- 4.1: Phase Current Sensing ---
#if (MY_BOARD_PH_CURRENT_SENSE_TYPE == 1) // SENSOR_TYPE_SHUNT
// If using a shunt resistor, define the following parameters:
#define MY_BOARD_PH_SHUNT_RESISTANCE_OHM (0.005f) // Resistance of the shunt resistor (Ohm)
#define MY_BOARD_PH_CSA_GAIN_V_V         (15.0f)  // Gain of the Current Sense Amplifier (CSA) (V/V)
#define MY_BOARD_PH_CSA_BIAS_V           (1.65f)  // Bias voltage of the amplifier's output (V)
#elif (MY_BOARD_PH_CURRENT_SENSE_TYPE == 2 ||                                                                          \
       MY_BOARD_PH_CURRENT_SENSE_TYPE == 3)           // SENSOR_TYPE_HALL or SENSOR_TYPE_DIRECT
// If using a Hall-effect or other direct voltage output sensor, define the following:
#define MY_BOARD_PH_CURRENT_SENSITIVITY_MV_A (100.0f) // Sensor sensitivity (mV/A)
#define MY_BOARD_PH_CURRENT_ZERO_BIAS_V      (2.5f)   // Sensor output voltage at zero current (V)
#endif
#define MY_BOARD_PH_CURRENT_SENSE_POLE_HZ (150.0e3f) // Filter bandwidth of the signal path (Hz)

//--- 4.2: Phase Voltage Sensing ---
#if (MY_BOARD_PH_VOLTAGE_SENSE_TYPE != 0) // Not SENSOR_NONE
#define MY_BOARD_PH_VOLTAGE_SENSE_GAIN                                                                                 \
    (0.02083f) // Gain of the voltage sensing circuit (V/V), i.e., (R_low / (R_high + R_low))
#define MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V                                                                               \
    (0.0f) // Bias of the voltage sensing circuit (V), typically 0 for passive dividers
#define MY_BOARD_PH_VOLTAGE_SENSE_POLE_HZ (400.0f) // Filter bandwidth of the signal path (Hz)
#endif

//--- 4.3: DC Bus Voltage Sensing ---
#if (MY_BOARD_DCBUS_VOLTAGE_SENSE_TYPE != 0)           // Not SENSOR_NONE
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN    (0.02083f) // Gain of the voltage sensing circuit (V/V)
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V  (0.0f)    // Bias of the voltage sensing circuit (V)
#define MY_BOARD_DCBUS_VOLTAGE_SENSE_POLE_HZ (400.0f)  // Filter bandwidth of the signal path (Hz)
#endif

//--- 4.4: DC Bus Current Sensing ---
#if (MY_BOARD_DCBUS_CURRENT_SENSE_TYPE == 1) // SENSOR_TYPE_SHUNT
#define MY_BOARD_DCBUS_SHUNT_RESISTANCE_OHM  (0.005f)
#define MY_BOARD_DCBUS_CSA_GAIN_V_V          (15.0f)
#define MY_BOARD_DCBUS_CSA_BIAS_V            (1.65f)
#define MY_BOARD_DCBUS_CURRENT_SENSE_POLE_HZ (150.0e3f)
#elif (MY_BOARD_DCBUS_CURRENT_SENSE_TYPE == 2) // SENSOR_TYPE_HALL
#define MY_BOARD_DCBUS_CURRENT_SENSITIVITY_MV_A (100.0f)
#define MY_BOARD_DCBUS_CURRENT_ZERO_BIAS_V      (2.5f)
#define MY_BOARD_DCBUS_CURRENT_SENSE_POLE_HZ    (150.0e3f)
#endif

//--- 4.5: Thermal Sensing ---
#if (MY_BOARD_THERMAL_SENSE_TYPE == 1) // THERMAL_SENSOR_NTC
// If using an NTC thermistor, define the following parameters:
#define MY_BOARD_THERMAL_PULLUP_RESISTANCE_OHM (10000.0f) // Pull-up resistor in the NTC voltage divider circuit (Ohm)
#define MY_BOARD_THERMAL_NTC_BETA_VALUE        (3950.0f)  // Beta value of the NTC (K)
#define MY_BOARD_THERMAL_NTC_NOMINAL_R_OHM     (10000.0f) // Nominal resistance of the NTC at nominal temperature (Ohm)
#define MY_BOARD_THERMAL_NTC_NOMINAL_TEMP_C    (25.0f)    // Nominal temperature of the NTC (��C)
#elif (MY_BOARD_THERMAL_SENSE_TYPE == 3)                  // THERMAL_SENSOR_IC
// If using an IC temperature sensor, define the following parameters:
#define MY_BOARD_THERMAL_IC_SENSITIVITY_MV_C (10.0f)      // Sensor sensitivity (mV/��C)
#define MY_BOARD_THERMAL_IC_OFFSET_V         (0.5f)       // Sensor output voltage at 0��C (V)
#endif

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_HAL_TEMPLATE_H
