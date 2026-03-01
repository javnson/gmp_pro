
#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_

//=================================================================================================
// Incremental Debug Options

// BUILD_LEVEL 1: hardware validate, VF, voltage open loop
// BUILD_LEVEL 2: IF, current close loop
// BUILD_LEVEL 3: current loop with actual angle
// BUILD_LEVEL 4: speed loop
// BUILD_LEVEL 5: position loop
// BUILD_LEVEL 6: communication mode
#define BUILD_LEVEL (2)

//=================================================================================================
// Controller basic parameters

// Startup Delay, ms
#define CTRL_STARTUP_DELAY (100)

// Controller Frequency
#define CONTROLLER_FREQUENCY (20e3)

// PWM depth
#define CTRL_PWM_CMP_MAX (2500 - 1)

// PWM dead band
#define CTRL_PWM_DEADBAND_CMP (100)

// ADC Voltage Reference
#define CTRL_ADC_VOLTAGE_REF (3.3f)

//=================================================================================================
// Hardware parameters

#define BOOSTXL_3PHGANINV_IS_DEFAULT_PARAM

// invoke motor parameters
#include <ctl/component/hardware_preset/pmsm_motor/TYI_5008_KV335.h>

// invoke motor controller parameters
#include <ctl/component/hardware_preset/inverter_3ph/TI_BOOSTXL_3PhGaNInv.h>

///////////////////////////////////////////////////////////
// Encoder Properties

// Encoder Full scale
#define CTRL_POS_ENC_FS (16384)

// Encoder Bias
#define CTRL_POS_ENC_BIAS (0.0999145508f)

// Speed division
#define CTRL_SPD_DIV (5)

// Position division
#define CTRL_POS_DIV (5)

///////////////////////////////////////////////////////////
// Controller Base value

// DC bus voltage
#define CTRL_DCBUS_VOLTAGE (80.0f)

// phase voltage base, SVPWM modulation
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

// voltage base, SPWM modulation
//#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 2.0f)

// Current base, 10 A
#define CTRL_CURRENT_BASE (10.0f)

///////////////////////////////////////////////////////////
// inverter side sensor

// Current sensor sensitivity, TMCS1133A2B, V/A
#define CTRL_INVERTER_CURRENT_SENSITIVITY (MY_BOARD_PH_SHUNT_RESISTANCE_OHM * MY_BOARD_PH_CSA_GAIN_V_V)

// Current sensor bias, V
#define CTRL_INVERTER_CURRENT_BIAS (MY_BOARD_PH_CSA_BIAS_V)

// Voltage sensor sensitivity, V/V
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (MY_BOARD_PH_VOLTAGE_SENSE_GAIN)

// Voltage sensor bias, V
#define CTRL_INVERTER_VOLTAGE_BIAS (MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V)

///////////////////////////////////////////////////////////
// DC Bus side sensor

// Current sensor sensitivity, V/A
#define CTRL_DC_CURRENT_SENSITIVITY (MY_BOARD_DCBUS_CURRENT_SENSE_GAIN)

// Current sensor bias, V
#define CTRL_DC_CURRENT_BIAS (MY_BOARD_DCBUS_CURRENT_SENSE_BIAS_V)

// Voltage sensor sensitivity, maximum 120V, V/V
#define CTRL_DC_VOLTAGE_SENSITIVITY (MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN)

// Voltage sensor bias, V
#define CTRL_DC_VOLTAGE_BIAS (MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V)


//=================================================================================================
// Controller Settings

// Use discrete PID controller
// Discrete controller may bring more smooth response.
//#define PMSM_CTRL_USING_DISCRETE_CTRL

// Enable Discrete PID controller anti-saturation algorithm
#define _USE_DEBUG_DISCRETE_PID

// Enable ADC Calibrate
#define SPECIFY_ENABLE_ADC_CALIBRATE

// Using negative modulator logic
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC (1)

// Using three level modulator or two level modulator
//#define USING_NPC_MODULATOR

// ADC Calibrate time ms
#define TIMEOUT_ADC_CALIB_MS 10000

// Motor Current Sample phases
#define MC_CURRENT_SAMPLE_PHASE_MODE (2)

// Enable Motor Fault protection
#define ENABLE_MOTOR_FAULT_PROTECTION

// Enable SMO
#define ENABLE_SMO

#endif // _FILE_CTRL_SETTINGS_H_
