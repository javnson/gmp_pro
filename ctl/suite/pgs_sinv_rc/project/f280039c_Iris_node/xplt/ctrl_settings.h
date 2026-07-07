/**
 * @file ctrl_settings.h
 * @brief Global Settings for the Single-Phase Inverter System.
 */

#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_

//=================================================================================================
// Incremental Debug Options (BUILD_LEVEL)

// BUILD_LEVEL 1: 离网开环/纯电阻负载测试 (关闭 FDRC，关闭前馈，验证 QPR 与 SPWM 发波)
// BUILD_LEVEL 2: 并网闭环控制 (引入电网电压前馈，实现基础 PQ 控制，FDRC 关闭)
// BUILD_LEVEL 3: 并网电能质量优化 (延时切入 FDRC，消除死区与电网背景谐波，全功能运行)
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

//=================================================================================================
// Power System Ratings (24Vrms, 10A, 60Vdc)

#define CTRL_DCBUS_VOLTAGE     (60.0f) // 额定直流母线电压
#define CTRL_GRID_VOLTAGE_RMS  (24.0f) // 额定交流有效值
#define CTRL_RATED_CURRENT_RMS (10.0f) // 额定交流电流有效值

// 单相逆变器标幺化基准值 (使用峰值作为计算 Base)
#define CTRL_VOLTAGE_BASE (34.0f)  // 24Vrms * 1.414
#define CTRL_CURRENT_BASE (14.14f) // 10Arms * 1.414

//=================================================================================================
// Hardware Abstraction Mapping

#include <ctl/component/hardware_preset/inverter_HB/GMP_LVFB_150_2ph_v2.h>
#include <ctl/component/hardware_preset/current_sensor/GMP_Quad_Sensor_Docker.h>

// ---------------------------------------------------------
// AC Voltage Sensing (Grid/Load Voltage)
// ---------------------------------------------------------

#define CTRL_AC_VOLTAGE_SENSITIVITY QUAD_SENSOR_V_SENSITIVE
#define CTRL_AC_VOLTAGE_BIAS        QUAD_SENSOR_BASE_BIAS_V

// ---------------------------------------------------------
// AC Current Sensing (Inverter Current)
// ---------------------------------------------------------

#define CTRL_AC_CURRENT_SENSITIVITY GMP_LVFB_SENSOR_I_SENSITIVE
#define CTRL_AC_CURRENT_BIAS        GMP_LVFB_CURRENT_BIAS_V

// ---------------------------------------------------------
// DC Bus Voltage Sensing
// ---------------------------------------------------------

#define CTRL_DC_VOLTAGE_SENSITIVITY GMP_LVFB_SENSOR_V_SENSITIVE
#define CTRL_DC_VOLTAGE_BIAS        GMP_LVFB_VOLTAGE_BIAS_V

//=================================================================================================
// System Protection Bounds (Derived from Hardware)

#define CTRL_MAX_HW_VOLTAGE GMP_LVFB_VBUS_MAX_V
#define CTRL_MAX_HW_CURRENT GMP_LVFB_CURRENT_MAX_RMS_A

#define CTRL_PROT_VBUS_MAX (100.0f)

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
#define PHASE_L_BASE IRIS_EPWM3_BASE
#define PHASE_N_BASE IRIS_EPWM4_BASE

// PWM Enable
#define PWM_ENABLE_PORT IRIS_GPIO1
#define PWM_RESET_PORT  IRIS_GPIO3

// System LED
#define SYSTEM_LED     IRIS_LED1
#define CONTROLLER_LED IRIS_LED2

// Grid side Current Measurement
#define INV_IAC_RESULT_BASE ADC_CH1_RESULT_BASE
#define INV_IAC             ADC_CH1

// Grid side Voltage Measurement
#define INV_VAC_RESULT_BASE ADC_CH2_RESULT_BASE
#define INV_VAC             ADC_CH2

// DC bus voltage measurement
#define INV_VBUS_RESULT_BASE ADC_CH3_RESULT_BASE
#define INV_VBUS             ADC_CH3



#endif // _FILE_CTRL_SETTINGS_H_

