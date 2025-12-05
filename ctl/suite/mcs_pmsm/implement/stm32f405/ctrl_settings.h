
#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_

// invoke motor parameters
#include <ctl/component/motor_control/motor_preset/GBM2804H_100T.h>

// Given 3.3V voltage reference
#define ADC_REFERENCE ((3.3))

// Controller Frequency
#define CONTROLLER_FREQUENCY (10000)

// PWM depth
#define CONTROLLER_PWM_CMP_MAX (8500)

// Speed controller Division
#define SPD_CONTROLLER_PWM_DIVISION (5)

// Current sensor
#define MTR_CTRL_CURRENT_GAIN (10.0)
#define MTR_CTRL_CURRENT_BIAS (1.65 / ADC_REFERENCE)

// Voltage sensor
#define MTR_CTRL_VOLTAGE_GAIN (0.1)
#define MTR_CTRL_VOLTAGE_BIAS (0.0)

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
