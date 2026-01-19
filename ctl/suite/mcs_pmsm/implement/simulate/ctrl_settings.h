
#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_



// invoke motor parameters
//#include <ctl/component/motor_control/motor_preset/GBM2804H_100T.h>

// invoke motor controller parameters
//#include <ctl/component/motor_control/controller_preset/TI_BOOSTXL_3PhGaNInv.h>

// invoke motor parameters
#include <ctl/component/hardware_preset/pmsm_motor/SM060R20B30MNAD.h>

// Startup Delay, ms
#define CTRL_STARTUP_DELAY (100)

// Controller Frequency
#define CONTROLLER_FREQUENCY (10000)
#define CTRL_FS (10000)

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
#define CTRL_PWM_CMP_MAX (5000)

// ADC Voltae Reference
#define CTRL_ADC_VOLTAGE_REF (3.3f)

// ADC reference Voltage
#define ADC_REFERENCE (3.3)

// Speed controller Division
#define SPD_CONTROLLER_PWM_DIVISION (5)

// Current sensor
#define MTR_CTRL_CURRENT_GAIN (8.0)
#define MTR_CTRL_CURRENT_BIAS (1.65 / ADC_REFERENCE)

// Voltage sensor
#define MTR_CTRL_VOLTAGE_GAIN (4.3889)
#define MTR_CTRL_VOLTAGE_BIAS (0.0)

// Current Bandwidth
#define MTR_CTRL_CURRENT_LOOP_BW ((500))

// Speed Bandwidth
#define MTR_CTRL_SPEED_LOOP_BW ((200))

// Controller Base Voltage and Base Current
#define MTR_CTRL_VOLTAGE_BASE ((MOTOR_PARAM_MAX_DC_VOLTAGE))
#define MTR_CTRL_CURRENT_BASE ((MOTOR_PARAM_RATED_CURRENT))


// Controller Base Voltage and Base Current
//#define MTR_CTRL_VOLTAGE_BASE ((MOTOR_PARAM_MAX_DC_VOLTAGE))
//#define MTR_CTRL_CURRENT_BASE ((MOTOR_PARAM_RATED_CURRENT))

//// Current Bandwidth
//#define MTR_CTRL_CURRENT_LOOP_BW ((50))
//
//// Speed Bandwidth
//#define MTR_CTRL_SPEED_LOOP_BW ((10))

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

// Enable Discrete PID controller anti-saturation algorithm
#define _USE_DEBUG_DISCRETE_PID

// Enable ADC Calibrate
#define SPECIFY_ENABLE_ADC_CALIBRATE


#endif // _FILE_CTRL_SETTINGS_H_
 