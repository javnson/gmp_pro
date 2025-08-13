
#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_

// invoke motor parameters
#include <ctl/component/motor_control/motor_preset/GBM2804H_100T.h>

// invoke motor controller parameters
#include <ctl/component/motor_control/controller_preset/TI_BOOSTXL_3PhGaNInv.h>

// Controller Frequency
#define CTRL_FS (10000)

// PWM depth
#define CTRL_PWM_CMP_MAX (6000)

// ADC Voltae Reference
#define CTRL_ADC_VOLTAGE_REF (3.3f)

// Speed controller Division
#define SPD_CONTROLLER_PWM_DIVISION (5)

// Controller Base Voltage and Base Current
#define MTR_CTRL_VOLTAGE_BASE ((MOTOR_PARAM_MAX_DC_VOLTAGE))
#define MTR_CTRL_CURRENT_BASE ((MOTOR_PARAM_RATED_CURRENT))

// Current Bandwidth
#define MTR_CTRL_CURRENT_LOOP_BW ((50))

// Speed Bandwidth
#define MTR_CTRL_SPEED_LOOP_BW ((10))

// BUILD_LEVEL 1: Voltage Open loop
// BUILD_LEVEL 2: Current Open loop
// BUILD_LEVEL 3: Actual Current loop
// BUILD_LEVEL 4: Speed Close loop
#define BUILD_LEVEL (1)

// 
// Controller Settings
//

// Use discrete PID controller
// Discrete controller may bring more smooth response.
//#define PMSM_CTRL_USING_DISCRETE_CTRL

// Enable Discrete PID controller anti-saturation algorithm
#define _USE_DEBUG_DISCRETE_PID

// Enable ADC Calibrate
#define SPECIFY_ENABLE_ADC_CALIBRATE


#endif // _FILE_CTRL_SETTINGS_H_
 