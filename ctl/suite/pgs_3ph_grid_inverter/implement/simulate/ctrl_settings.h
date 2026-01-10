
#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_


// BUILD_LEVEL 1: inverter, voltage open loop
// BUILD_LEVEL 2: inverter, current loop
// BUILD_LEVEL 3: inverter, current loop, feed forward control
// BUILD_LEVEL 4: inverter, current loop, feed forward control, negative current control
// BUILD_LEVEL 5: inverter, current loop, ff, neg, harmonic control
// BUILD_LEVEL 6: inverter, voltage loop, current loop, ff
// BUILD_LEVEL 7: inverter, voltage loop, current loop, ff, negative voltage control
#define BUILD_LEVEL (7)

// low voltage half bridge parameters
//#include <ctl/component/digital_power/hardware_preset/gmp_lvhb_v1.h>

// Startup Delay, ms
#define CTRL_STARTUP_DELAY (100)

// Controller Frequency
#define CONTROLLER_FREQUENCY (20e3)

// PWM depth
#define CTRL_PWM_CMP_MAX (4200 - 1)

// DC bus voltage, voltage base
#define CTRL_VOLTAGE_BASE (100.0f)

// Current base, 10 A
#define CTRL_CURRENT_BASE (10.0f)

// Voltage reference, 2.5V
#define CTRL_ADC_VOLTAGE_REF (2.5)

// ADC resolution
#define CTRL_ADC_RESOLUTION (12)

///////////////////////////////////////////////////////////
// Grid side sensor

// Current sensor sensitivity, V/A
#define CTRL_GRID_CURRENT_SENSITIVITY (48e-3f)

// Current sensor bias, V
#define CTRL_GRID_CURRENT_BIAS (1.65f)

// Voltage sensor sensitivity, V/V
#define CTRL_GRID_VOLTAGE_SENSITIVITY (0.0106f)

// Voltage sensor bias, V
#define CTRL_GRID_VOLTAGE_BIAS (0.0f)

///////////////////////////////////////////////////////////
// inverter side sensor

// Current sensor sensitivity, V/A
#define CTRL_INVERTER_CURRENT_SENSITIVITY (50e-3f)

// Current sensor bias, V
#define CTRL_INVERTER_CURRENT_BIAS (1.65f)

// Voltage sensor sensitivity, V/V
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (0.02738589f)

// Voltage sensor bias, V
#define CTRL_INVERTER_VOLTAGE_BIAS (0.0f)

///////////////////////////////////////////////////////////
// DC Bus side sensor

// Current sensor sensitivity, V/A
#define CTRL_DC_CURRENT_SENSITIVITY (24.75e-3f)

// Current sensor bias, V
#define CTRL_DC_CURRENT_BIAS (1.65f)

// Voltage sensor sensitivity, V/V
#define CTRL_DC_VOLTAGE_SENSITIVITY (0.02738589f)

// Voltage sensor bias, V
#define CTRL_DC_VOLTAGE_BIAS (0.0f)

// SPLL Close loop criteria
#define CTRL_SPLL_EPSILON ((float2ctrl(0.005)))

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
 