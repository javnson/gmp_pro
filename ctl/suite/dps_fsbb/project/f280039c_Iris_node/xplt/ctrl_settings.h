/**
 * @file ctrl_settings.h
 * @brief Global Settings for the Single-Phase Inverter System.
 */

#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_

//=================================================================================================
// Incremental Debug Options (BUILD_LEVEL)

// BUILD_LEVEL 1: Modulation only, Hardware check
// BUILD_LEVEL 2: Current loop
// BUILD_LEVEL 3: Voltage loop
#define BUILD_LEVEL (1)

//=================================================================================================
// Controller Basic Parameters

// Startup Delay, ms
#define CTRL_STARTUP_DELAY (100)

// Controller Frequency
//#define CONTROLLER_FREQUENCY (10e3)
#define CONTROLLER_FREQUENCY (20e3)

// PWM depth
//#define CTRL_PWM_CMP_MAX (6000-1)
#define CTRL_PWM_CMP_MAX (3000 - 1)

// PWM deadband
#define CTRL_PWM_DEADBAND_CMP (50)

// System tick
#define DSP_C2000_DSP_TIME_DIV (120000 / CTRL_PWM_CMP_MAX / 2)

// ADC Voltage Reference
#define CTRL_ADC_VOLTAGE_REF (3.3f)

// Resistance Load, Omega
#define FSBB_RLOAD_MIN (20.0f)

// Capacitor Input
#define FSBB_CIN (440e-6f)

// Capacitor Output
#define FSBB_COUT (440e-6f)
#define FSBB_COUT_ESR (0.1f)

// Inductor for FSBB
#define FSBB_L (1.5e-3f)
#define FSBB_L_ESR (0.05f)

// Input voltage for FSBB
#define FSBB_INPUT_VOLTAGE (24.0f)

// minimum voltage input for FSBB
#define FSBB_INPUT_VOLTAGE_MIN (12.0f)

// maximum output current for FSBB
#define FSBB_OUTPUT_CURRENT_LIM (10.0f)

// Output voltage for FSBB
#define FSBB_OUTPUT_VOLTAGE (12.0f)

// Maximum output voltage for FSBB
#define FSBB_OUTPUT_VOLTAGE_MAX (72.0f)

// Minimum output voltage for FSBB
#define FSBB_OUTPUT_VOLTAGE_MIN (3.0f)

// Protection parameters, iL max
#define FSBB_PROTECT_IL_MAX (25.0f)

// Protection parameters, iL min
#define FSBB_PROTECT_IL_MIN (-2.0f)

//=================================================================================================
// Power System Ratings (24Vrms, 10A, 60Vdc)

#define CTRL_DCBUS_VOLTAGE     (60.0f) // 额定直流母线电压
//#define CTRL_GRID_VOLTAGE_RMS  (24.0f) // 额定交流有效值
//#define CTRL_RATED_CURRENT_RMS (10.0f) // 额定交流电流有效值

// 单相逆变器标幺化基准值 (使用峰值作为计算 Base)
#define CTRL_VOLTAGE_BASE (34.0f)  // 24Vrms * 1.414
#define CTRL_CURRENT_BASE (14.14f) // 10Arms * 1.414

//=================================================================================================
// Hardware Abstraction Mapping

#include <ctl/component/hardware_preset/inverter_HB/GMP_LVFB_150_2ph_v2.h>
#include <ctl/component/hardware_preset/current_sensor/GMP_Quad_Sensor_Docker.h>

// ---------------------------------------------------------
// Voltage Sensing
// ---------------------------------------------------------
#define CTRL_VIN_VOLTAGE_SENSITIVITY  QUAD_SENSOR_V_SENSITIVE
#define CTRL_VIN_VOLTAGE_BIAS         QUAD_SENSOR_BASE_BIAS_V

#define CTRL_VOUT_VOLTAGE_SENSITIVITY GMP_LVFB_SENSOR_V_SENSITIVE
#define CTRL_VOUT_VOLTAGE_BIAS        GMP_LVFB_VOLTAGE_BIAS_V

// ---------------------------------------------------------
// Current Sensing
// ---------------------------------------------------------
// 10Arms 峰值为 14.1A。选用 TMCS1133-B2A (±31.0A 量程，50mV/A 灵敏度)
#define CTRL_INDUCTOR_CURRENT_SENSITIVITY GMP_LVFB_SENSOR_I_SENSITIVE
#define CTRL_INDUCTOR_CURRENT_BIAS        GMP_LVFB_CURRENT_BIAS_V

#define CTRL_LOAD_CURRENT_SENSITIVITY QUAD_SENSOR_I_SENSITIVE
#define CTRL_LOAD_CURRENT_BIAS        QUAD_SENSOR_BASE_BIAS_V

//=================================================================================================
// System Protection Bounds (Derived from Hardware)

#define CTRL_MAX_HW_VOLTAGE GMP_LVFB_VBUS_MAX_V
#define CTRL_MAX_HW_CURRENT GMP_LVFB_CURRENT_MAX_RMS_A

//#define CTRL_PROT_VBUS_MAX (100.0f)

//=================================================================================================
// Controller Settings

// Enable Discrete PID controller anti-saturation algorithm
#define _USE_DEBUG_DISCRETE_PID

// Enable ADC Calibrate
#define SPECIFY_ENABLE_ADC_CALIBRATE
#define TIMEOUT_ADC_CALIB_MS (3000)

// SPLL Close loop criteria
#define CTRL_SPLL_EPSILON ((float2ctrl(0.005)))

// Using negative modulator logic
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC (0)

// Enable PIL simulation function
// This macro will disable all the controller output.
//#define ENBALE_GMP_DL_PIL_SIM

// Enable Cia402 Debug Information
//#define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

//=================================================================================================
// Board peripheral mapping

// PWM Channels
#define PHASE_BUCK_BASE IRIS_EPWM1_BASE
#define PHASE_BOOST_BASE IRIS_EPWM2_BASE

// PWM Enable
#define PWM_ENABLE_PORT IRIS_GPIO1
#define PWM_RESET_PORT  IRIS_GPIO3

// System LED
#define SYSTEM_LED     IRIS_LED1
#define CONTROLLER_LED IRIS_LED2

// ADC channel Mapping
#define FSBB_VIN_ADC_BASE  ADC_CH1_ADC_BASE
#define FSBB_VIN           ADC_CH1

#define FSBB_VOUT_ADC_BASE ADC_CH2_ADC_BASE
#define FSBB_VOUT          ADC_CH2

#define FSBB_IL_ADC_BASE   ADC_CH3_ADC_BASE
#define FSBB_IL            ADC_CH3

#define FSBB_IOUT_ADC_BASE ADC_CH4_ADC_BASE
#define FSBB_IOUT          ADC_CH4

#endif // _FILE_CTRL_SETTINGS_H_

