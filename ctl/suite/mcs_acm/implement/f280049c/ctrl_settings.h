
#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_


// invoke motor parameters
#include <ctl/component/hardware_preset/acm_motor/ACM_4P24V.h>

// invoke motor controller parameters
#include <ctl/component/hardware_preset/inverter_3ph/GMP_3PH_2136SINV_DUAL.h>
//#include <ctl/component/hardware_preset/inverter_3ph/TI_3PH_GAN_INV.h>


// Controller Frequency, unit Hz
#define CONTROLLER_FREQUENCY (10000)

// PWM depth
#define CONTROLLER_PWM_CMP_MAX (6000)

// Encoder lines
#define MTR_ENCODER_LINES  ((10000))

// Encoder Offset
#define MTR_ENCODER_OFFSET ((1000))

// Speed controller Division
#define SPD_CONTROLLER_PWM_DIVISION (5)

// Controller Base Voltage and Base Current
#define MTR_CTRL_VOLTAGE_BASE ((MOTOR_PARAM_MAX_DC_VOLTAGE))
#define MTR_CTRL_CURRENT_BASE ((MOTOR_PARAM_RATED_CURRENT))

// Current ADC module default per unit parameter
#define MTR_CTRL_CURRENT_GAIN (ADC_FULLSCALE_CURRENT * 2 / MTR_CTRL_CURRENT_BASE)
#define MTR_CTRL_CURRENT_BIAS (ADC_FULLSCALE_CURRENT / ADC_REFERENCE)


// Voltage ADC module default per unit parameter
#define MTR_CTRL_VOLTAGE_GAIN ((ADC_FULLSCALE_VOLTAGE / MTR_CTRL_VOLTAGE_BASE))
#define MTR_CTRL_VOLTAGE_BIAS ((0.0))

// Current Bandwidth
#define MTR_CTRL_CURRENT_LOOP_BW ((5))

// Speed Bandwidth
#define MTR_CTRL_SPEED_LOOP_BW ((0.1))

// BUILD_LEVEL 1: Voltage Open loop, VF control
// BUILD_LEVEL 2: Current Open loop, IF control
// BUILD_LEVEL 3: Speed Close loop
#define BUILD_LEVEL (3)

//
// Controller Settings
//

// Use discrete PID controller
// Discrete controller may bring more smooth response.
#define ACM_CTRL_USING_DISCRETE_CTRL

// Use QEP as Encoder input
#define PMSM_CTRL_USING_QEP_ENCODER

// Enable ADC Calibrate
#define SPECIFY_ENABLE_ADC_CALIBRATE

// Enable DLOG module
#define DLOG_ENABLE

//
// System Tick subsystem
// System tick will increase itself every 1ms.
//
#define DSP_C2000_DSP_TIME_DIV ((CONTROLLER_FREQUENCY / 1000))

#endif // _FILE_CTRL_SETTINGS_H_
