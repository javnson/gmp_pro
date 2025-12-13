
#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_

//// Select Board and Motor
//#define BOOSTXL_3PHGANINV_IS_DEFAULT_PARAM
//
//// invoke motor parameters
//#include <ctl/component/motor_control/motor_preset/GBM2804H_100T.h>
//
//// invoke motor controller parameters
//#include <ctl/component/motor_control/controller_preset/TI_BOOSTXL_3PhGaNInv.h>


// invoke motor parameters
#include <ctl/component/motor_control/motor_preset/SM060R20B30MNAD.h>


// Controller Frequency
#define CONTROLLER_FREQUENCY (10000)
#define CTRL_FS (10000)

// Startup Delay, ms
#define CTRL_STARTUP_DELAY (100)

// enable QEP encoder
#define PMSM_CTRL_USING_QEP_ENCODER

// QEP Encoder Configuration
#ifdef PMSM_CTRL_USING_QEP_ENCODER

// Encoer Lines
#define MTR_ENCODER_LINES (10000)

// Encoder Offsets
#define MTR_ENCODER_OFFSET (0.0226000007)

#endif // PMSM_CTRL_USING_QEP_ENCODER


// PWM depth
#define CONTROLLER_PWM_CMP_MAX (5000)

// ADC reference Voltage
#define ADC_REFERENCE (3.3)

// Speed controller Division
#define SPD_CONTROLLER_PWM_DIVISION (5)

//// Controller Base Voltage and Base Current
//#define MTR_CTRL_VOLTAGE_BASE ((MOTOR_PARAM_MAX_DC_VOLTAGE))
//#define MTR_CTRL_CURRENT_BASE ((MOTOR_PARAM_RATED_CURRENT))
//
//// Current ADC module default per unit parameter
//#define MTR_CTRL_CURRENT_GAIN (MY_BOARD_CURRENT_MAX_PEAK_A * 2 / MTR_CTRL_CURRENT_BASE)
//#define MTR_CTRL_CURRENT_BIAS (MY_BOARD_PH_CSA_BIAS_V / ADC_REFERENCE)
//
//
//// Voltage ADC module default per unit parameter
//#define MTR_CTRL_VOLTAGE_GAIN ((ADC_REFERENCE / MY_BOARD_PH_VOLTAGE_SENSE_GAIN / MTR_CTRL_VOLTAGE_BASE))
//#define MTR_CTRL_VOLTAGE_BIAS ((0.0))
//
//// Current Bandwidth
//#define MTR_CTRL_CURRENT_LOOP_BW ((100))
//
//// Speed Bandwidth
//#define MTR_CTRL_SPEED_LOOP_BW ((20))

// Current sensor
#define MTR_CTRL_CURRENT_GAIN (2.0)
#define MTR_CTRL_CURRENT_BIAS (1.65 / ADC_REFERENCE)

// Voltage sensor
#define MTR_CTRL_VOLTAGE_GAIN (0.1)
#define MTR_CTRL_VOLTAGE_BIAS (0.0)

// Current Bandwidth
#define MTR_CTRL_CURRENT_LOOP_BW ((50))

// Speed Bandwidth
#define MTR_CTRL_SPEED_LOOP_BW ((20))

// Controller Base Voltage and Base Current
#define MTR_CTRL_VOLTAGE_BASE ((MOTOR_PARAM_MAX_DC_VOLTAGE))
#define MTR_CTRL_CURRENT_BASE ((MOTOR_PARAM_RATED_CURRENT))



// BUILD_LEVEL 1: Voltage Open loop
// BUILD_LEVEL 2: Current Open loop
// BUILD_LEVEL 3: Actual Current loop
// BUILD_LEVEL 4: Speed Close loop
#define BUILD_LEVEL (4)

//
// Controller Settings
//

// Use discrete PID controller
// Discrete controller may bring more smooth response.
//#define PMSM_CTRL_USING_DISCRETE_CTRL

// Use QEP as Encoder input
//#define PMSM_CTRL_USING_QEP_ENCODER

// Enable ADC Calibrate
#define SPECIFY_ENABLE_ADC_CALIBRATE

//
// System Tick subsystem
// System tick will increase itself every 1ms.
//
#define DSP_C2000_DSP_TIME_DIV ((CONTROLLER_FREQUENCY/1000))

#endif // _FILE_CTRL_SETTINGS_H_
