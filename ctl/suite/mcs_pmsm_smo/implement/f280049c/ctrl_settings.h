
#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_


// invoke motor parameters
#include <ctl/component/hardware_preset/pmsm_motor/GBM2804H_100T.h>

// invoke motor controller parameters
#include <ctl/component/hardware_preset/inverter_3ph/TI_BOOSTXL_3PhGaNInv.h>

// Controller Frequency, unit Hz
#define CONTROLLER_FREQUENCY (10000)

// PWM depth
#define CONTROLLER_PWM_CMP_MAX (6000)

// Speed controller Division
#define SPD_CONTROLLER_PWM_DIVISION (5)

// Controller Base Voltage and Base Current
#define MTR_CTRL_VOLTAGE_BASE ((MOTOR_PARAM_MAX_DC_VOLTAGE))
#define MTR_CTRL_CURRENT_BASE ((MOTOR_PARAM_RATED_CURRENT))

#define ADC_FULLSCALE_CURRENT (BOOSTXL_3PHGANINV_PH_CSA_BIAS_V / BOOSTXL_3PHGANINV_PH_CSA_GAIN_V_V / BOOSTXL_3PHGANINV_PH_SHUNT_RESISTANCE_OHM)
// Current ADC module default per unit parameter
#define MTR_CTRL_CURRENT_GAIN (ADC_FULLSCALE_CURRENT * 2 / MTR_CTRL_CURRENT_BASE)
#define MTR_CTRL_CURRENT_BIAS (ADC_FULLSCALE_CURRENT / BOOSTXL_3PHGANINV_PH_CSA_BIAS_V)

#define ADC_FULLSCALE_VOLTAGE (3.3 / BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_GAIN)
// Voltage ADC module default per unit parameter
#define MTR_CTRL_VOLTAGE_GAIN ((ADC_FULLSCALE_VOLTAGE / MTR_CTRL_VOLTAGE_BASE))
#define MTR_CTRL_VOLTAGE_BIAS ((0.0))

// Current Bandwidth
#define MTR_CTRL_CURRENT_LOOP_BW ((100))

// Speed Bandwidth
#define MTR_CTRL_SPEED_LOOP_BW ((20))

// BUILD_LEVEL 1: Voltage Open loop
// BUILD_LEVEL 2: Current Open loop
// BUILD_LEVEL 3: SMO with speed loop
// BUILD_LEVEL 4: Speed Close loop
#define BUILD_LEVEL (3)

//
// Controller Settings
//

// Use discrete PID controller
// Discrete controller may bring more smooth response.
#define PMSM_CTRL_USING_DISCRETE_CTRL

// Use QEP as Encoder input
#define PMSM_CTRL_USING_QEP_ENCODER

//
// System Tick subsystem
// System tick will increase itself every 1ms.
//
#define DSP_C2000_DSP_TIME_DIV ((CONTROLLER_FREQUENCY / 1000))

#endif // _FILE_CTRL_SETTINGS_H_
