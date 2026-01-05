
#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_

// invoke motor controller parameters
#include <ctl/component/motor_control/controller_preset/TI_BOOSTXL_3PhGaNInv.h>

// Startup Delay, ms
#define CTRL_STARTUP_DELAY (100)

// Controller Frequency
#define CONTROLLER_FREQUENCY (20e3)

// PWM depth
#define CTRL_PWM_CMP_MAX (6000)

// ADC Voltae Reference
#define CTRL_ADC_VOLTAGE_REF (3.3f)

// DC bus voltage, voltage base
#define CTRL_VOLTAGE_BASE (60.0f)

// Current base
#define CTRL_CURRENT_BASE (10.0f)

// Speed controller Division
#define SPD_CONTROLLER_PWM_DIVISION (5)

// Controller Base Voltage and Base Current
#define MTR_CTRL_VOLTAGE_BASE ((MOTOR_PARAM_MAX_DC_VOLTAGE))
#define MTR_CTRL_CURRENT_BASE ((MOTOR_PARAM_RATED_CURRENT))

// Current Bandwidth
#define MTR_CTRL_CURRENT_LOOP_BW ((50))

// SPLL Close loop criteria
#define CTRL_SPLL_EPSILON ((float2ctrl(0.005)))

// BUILD_LEVEL 1: inverter, voltage open loop
// BUILD_LEVEL 2: inverter, current loop
// BUILD_LEVEL 3: inverter, current loop, feed forward control
// BUILD_LEVEL 4: inverter, current loop, feed forward control, negative current control
// BUILD_LEVEL 5: inverter, current loop, ff, neg, harmonic control
// BUILD_LEVEL 6: inverter, voltage loop, current loop, ff
// BUILD_LEVEL 7: inverter, voltage loop, current loop, ff, negative voltage control
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
 
