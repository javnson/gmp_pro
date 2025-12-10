
#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_

// invoke motor parameters
#include <ctl/component/motor_control/motor_preset/GBM2804H_100T.h>

// Given 3.3V voltage reference
#define ADC_REFERENCE ((3.3))

// Controller Frequency
#define CTRL_FS (10000)

// PWM depth
#define CONTROLLER_PWM_CMP_MAX (8399)

// Speed controller Division
#define SPD_CONTROLLER_PWM_DIVISION (5)

// Current sensor
#define MTR_CTRL_CURRENT_GAIN (10.0)
#define MTR_CTRL_CURRENT_BIAS (1.65 / ADC_REFERENCE)

// Voltage sensor
#define MTR_CTRL_VOLTAGE_GAIN (0.1)
#define MTR_CTRL_VOLTAGE_BIAS (0.0)

// Current Bandwidth
#define MTR_CTRL_CURRENT_LOOP_BW ((50))

// Speed Bandwidth
#define MTR_CTRL_SPEED_LOOP_BW ((10))

// Controller Base Voltage and Base Current
#define MTR_CTRL_VOLTAGE_BASE ((MOTOR_PARAM_MAX_DC_VOLTAGE))
#define MTR_CTRL_CURRENT_BASE ((MOTOR_PARAM_RATED_CURRENT))

// Startup Delay, ms
#define CTRL_STARTUP_DELAY (100)

// enable QEP encoder 
#define PMSM_CTRL_USING_QEP_ENCODER

// QEP Encoder Configuration
#ifdef PMSM_CTRL_USING_QEP_ENCODER

// Encoer Lines
#define MTR_ENCODER_LINES (8000)

// Encoder Offsets
#define MTR_ENCODER_OFFSET (0)

#endif // PMSM_CTRL_USING_QEP_ENCODER

// BUILD_LEVEL 1: Voltage Open loop
// BUILD_LEVEL 2: Current Open loop
// BUILD_LEVEL 3: Current Open loop with actual position
// BUILD_LEVEL 4: Speed Close loop
#define BUILD_LEVEL (1)

//
// Controller Settings
//

// Use discrete PID controller
// Discrete controller may bring more smooth response.
// #define PMSM_CTRL_USING_DISCRETE_CTRL

#endif // _FILE_CTRL_SETTINGS_H_
