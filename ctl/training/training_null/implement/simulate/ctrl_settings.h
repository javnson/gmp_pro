
#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_

// invoke motor parameters
#include <ctl/component/motor_control/motor_preset/GBM2804H_100T.h>

// invoke motor controller parameters
#include <ctl/component/motor_control/controller_preset/TI_3PH_GAN_INV.h>

// Controller Frequency
#define CONTROLLER_FREQUENCY (20000)

// PWM depth
#define CONTROLLER_PWM_CMP_MAX (4200-1)

// Voltage per unit base value, 100 A
#define CTRL_VOLTAGE_BASE 100

// Current Per unit base value, 10A
#define CTRL_CURRENT_BASE 10

// Voltage reference, 2.5V
#define CTRL_ADC_VOLTAGE_REF (2.5)

// Voltage gain
#define CTRL_VOLTAGE_SENSOR_GAIN (3.3/203.3)

// Voltage Bias, V
#define CTRL_VOLTAGE_SENSOR_BIAS (0.9966)

// Current Gain
#define CTRL_CURRENT_SENSOR_GAIN (0.005*20)

// Current Bias, V
#define CTRL_CURRENT_SENSOR_BIAS (0.9)

// Speed controller Division
#define SPD_CONTROLLER_PWM_DIVISION (5)

// Controller Base Voltage and Base Current
#define MTR_CTRL_VOLTAGE_BASE ((MOTOR_PARAM_MAX_DC_VOLTAGE))
#define MTR_CTRL_CURRENT_BASE ((MOTOR_PARAM_RATED_CURRENT))

// Current ADC module default per unit parameter
#define MTR_CTRL_CURRENT_GAIN (ADC_FULLSCALE_CURRENT * 2 / MTR_CTRL_CURRENT_BASE)
#define MTR_CTRL_CURRENT_BIAS (ADC_CURRENT_REFERENCE_VOLTAGE / ADC_REFERENCE)


// Voltage ADC module default per unit parameter
#define MTR_CTRL_VOLTAGE_GAIN ((ADC_FULLSCALE_VOLTAGE / MTR_CTRL_VOLTAGE_BASE))
#define MTR_CTRL_VOLTAGE_BIAS ((0.0))

// Current Bandwidth
#define MTR_CTRL_CURRENT_LOOP_BW ((50))

// Speed Bandwidth
#define MTR_CTRL_SPEED_LOOP_BW ((10))

// BUILD_LEVEL 1: Voltage Open loop
// BUILD_LEVEL 2: Current Close loop
// BUILD_LEVEL 3: Voltage Close loop
#define BUILD_LEVEL (3)

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
 