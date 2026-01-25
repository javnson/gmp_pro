
#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_

//=================================================================================================
// Incremental Debug Options

// BUILD_LEVEL 1: inverter, voltage open loop
// BUILD_LEVEL 2: inverter, current loop
// BUILD_LEVEL 3: inverter, current loop, grid connected
// BUILD_LEVEL 4: inverter, current loop, grid connected, all feed forward on.
#define BUILD_LEVEL (2)

//=================================================================================================
// Select Board Pin definition
#define LAUNCHPAD 0
#define GMP_IRIS  1

#define BOARD_SELECTION GMP_IRIS

//=================================================================================================
// Controller basic parameters

// Startup Delay, ms
#define CTRL_STARTUP_DELAY (100)

// Controller Frequency
//#define CONTROLLER_FREQUENCY (10e3)
#define CONTROLLER_FREQUENCY (20e3)

// PWM depth
//#define CTRL_PWM_CMP_MAX (6000-1)
#define CTRL_PWM_CMP_MAX (3000-1)

// PWM deadband
#define CTRL_PWM_DEADBAND_CMP (50)

// System tick
#define DSP_C2000_DSP_TIME_DIV (120000/CTRL_PWM_CMP_MAX/2)

// ADC Voltage Reference
#define CTRL_ADC_VOLTAGE_REF (3.3f)

//=================================================================================================
// Hardware parameters

#include <ctl/component/hardware_preset/inverter_3ph/GMP_Helios_3PhGaNInv_LV.h>
#include <ctl/component/hardware_preset/grid_LC_filter/GMP_Harmonia_3ph_LC_filter.h>

///////////////////////////////////////////////////////////
// Controller Base value

// DC bus voltage
#define CTRL_DCBUS_VOLTAGE  (80.0f)

// phase voltage base, SVPWM modulation
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE/1.73205081f)

// voltage base, SPWM modulation
//#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE/2.0f)

// Current base, 10 A
#define CTRL_CURRENT_BASE (10.0f)

///////////////////////////////////////////////////////////
// Grid side sensor

// Current sensor sensitivity, V/A
#define CTRL_GRID_CURRENT_SENSITIVITY (HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSITIVITY_MV_A*0.001f)

// Current sensor bias, V
#define CTRL_GRID_CURRENT_BIAS (HARMONIA_3PH_LC_FILTER_PH_CURRENT_ZERO_BIAS_V)

// Voltage sensor sensitivity, V/V
#define CTRL_GRID_VOLTAGE_SENSITIVITY (HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_GAIN)

// Voltage sensor bias, V
#define CTRL_GRID_VOLTAGE_BIAS (HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_BIAS_V)

///////////////////////////////////////////////////////////
// inverter side sensor

// Current sensor sensitivity, TMCS1133A2B, V/A
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

// Voltage sensor sensitivity, maximum 120V, V/V
#define CTRL_DC_VOLTAGE_SENSITIVITY (0.02738589f)

// Voltage sensor bias, V
#define CTRL_DC_VOLTAGE_BIAS (0.0f)

//=================================================================================================
// Controller Settings

// Use discrete PID controller
// Discrete controller may bring more smooth response.
//#define PMSM_CTRL_USING_DISCRETE_CTRL

// Enable Discrete PID controller anti-saturation algorithm
#define _USE_DEBUG_DISCRETE_PID

// Enable ADC Calibrate
#define SPECIFY_ENABLE_ADC_CALIBRATE

// SPLL Close loop criteria
#define CTRL_SPLL_EPSILON ((float2ctrl(0.005)))

// Using three level modulator or two level modulator
#define USING_NPC_MODULATOR

//=================================================================================================
// Board peripheral mapping


// Launchpad Board Pin Mapping
#if BOARD_SELECTION == LAUNCHPAD
#ifndef BOARD_PIN_MAPPING
#define BOARD_PIN_MAPPING

// PWM Channels
#define PHASE_U_BASE EPWM_J4_PHASE_U_BASE
#define PHASE_V_BASE EPWM_J4_PHASE_V_BASE
#define PHASE_W_BASE EPWM_J4_PHASE_W_BASE

// PWM Enable
#define PWM_ENABLE_PORT ENABLE_GATE
#define PWM_RESET_PORT  RESET_GATE

// DC Bus Voltage & Current
#define INV_VBUS_RESULT_BASE J3_VDC_RESULT_BASE
#define INV_IBUS_RESULT_BASE J7_VDC_RESULT_BASE

#define INV_VBUS J3_VDC
#define INV_IBUS J7_VDC

// Grid side Voltage & Current
#define INV_UA_RESULT_BASE J7_VU_RESULT_BASE
#define INV_UB_RESULT_BASE J7_VV_RESULT_BASE
#define INV_UC_RESULT_BASE J7_VW_RESULT_BASE

#define INV_UA J7_VU
#define INV_UB J7_VV
#define INV_UC J7_VW

#define INV_IA_RESULT_BASE J7_IU_RESULT_BASE
#define INV_IB_RESULT_BASE J7_IV_RESULT_BASE
#define INV_IC_RESULT_BASE J7_IW_RESULT_BASE

#define INV_IA J7_IU
#define INV_IB J7_IV
#define INV_IC J7_IW

// Converter side Voltage & Current
#define INV_UU_RESULT_BASE J3_VU_RESULT_BASE
#define INV_UV_RESULT_BASE J3_VV_RESULT_BASE
#define INV_UW_RESULT_BASE J3_VW_RESULT_BASE

#define INV_UU J3_VU
#define INV_UV J3_VV
#define INV_UW J3_VW

#define INV_IU_RESULT_BASE J3_IU_RESULT_BASE
#define INV_IV_RESULT_BASE J3_IV_RESULT_BASE
#define INV_IW_RESULT_BASE J3_IW_RESULT_BASE

#define INV_IU J3_IU
#define INV_IV J3_IV
#define INV_IW J3_IW

// System LED
#define SYSTEM_LED LED_R
#define CONTROLLER_LED LED_G

#endif //BOARD_PIN_MAPPING

#else // BOARD_SELECTION == GMP_IRIS

#ifndef BOARD_PIN_MAPPING
#define BOARD_PIN_MAPPING

// PWM Channels
#define PHASE_U_BASE IRIS_EPWM1_BASE
#define PHASE_V_BASE IRIS_EPWM2_BASE
#define PHASE_W_BASE IRIS_EPWM3_BASE

// PWM Enable
#define PWM_ENABLE_PORT IRIS_GPIO1
#define PWM_RESET_PORT  IRIS_GPIO3

// Vbus Voltage Channels
//#define MOTOR_VBUS_RESULT_BASE IRIS_ADCA_RESULT_BASE
//#define MOTOR_VBUS

// ADC Voltage Channels
//#define MOTOR_VU_RESULT_BASE IRIS_ADCA_RESULT_BASE
//#define MOTOR_VV_RESULT_BASE IRIS_ADCB_RESULT_BASE
//#define MOTOR_VW_RESULT_BASE IRIS_ADCC_RESULT_BASE

//#define MOTOR_VU
//#define MOTOR_VV
//#define MOTOR_VW

// ADC Current Channels
//#define MOTOR_IU_RESULT_BASE IRIS_ADCA_RESULT_BASE
//#define MOTOR_IV_RESULT_BASE IRIS_ADCB_RESULT_BASE
//#define MOTOR_IW_RESULT_BASE IRIS_ADCC_RESULT_BASE

//#define MOTOR_IU
//#define MOTOR_IV
//#define MOTOR_IW

// System LED
#define SYSTEM_LED     IRIS_LED1
#define CONTROLLER_LED IRIS_LED2

#endif //BOARD_PIN_MAPPING

#endif // BOARD_PIN_MAPPING


#endif // _FILE_CTRL_SETTINGS_H_
 
