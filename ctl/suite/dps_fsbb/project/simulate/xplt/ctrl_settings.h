/**
 * @file ctrl_settings.h
 * @brief Global Settings for the FSBB (Four-Switch Buck-Boost) DC-DC Converter.
 */

#ifndef _FILE_CTRL_SETTINGS_H_
#define _FILE_CTRL_SETTINGS_H_

//=================================================================================================
// Incremental Debug Options (BUILD_LEVEL)

// BUILD_LEVEL 1: hardware validate, voltage open loop (验证发波极性与死区)
// BUILD_LEVEL 2: current close loop (验证电感电流内环的阶跃响应与抗饱和)
// BUILD_LEVEL 3: voltage loop (全功能双闭环，带负载前馈与斜率软启动)
#define BUILD_LEVEL (2)

//=================================================================================================
// Controller Basic Parameters

// Startup Delay, ms
#define CTRL_STARTUP_DELAY (100)

// Controller Frequency
#define CONTROLLER_FREQUENCY (20e3)

// PWM depth (e.g., 120MHz / 20kHz / 2 = 2500 for Up-Down count)
#define CTRL_PWM_CMP_MAX (3000 - 1)

// PWM dead band
#define CTRL_PWM_DEADBAND_CMP (100)

// System tick divider (Assuming 120MHz SYSCLK for Timer calculation, adjust as needed)
#define DSP_C2000_DSP_TIME_DIV (120000 / CTRL_PWM_CMP_MAX / 2)

// ADC Voltage Reference
#define CTRL_ADC_VOLTAGE_REF (3.3f)

//=================================================================================================
// Power System Ratings (12V-30V Input, 0-60V Output)

#define CTRL_DCBUS_VOLTAGE_MAX (60.0f) // 最大输出电压
#define CTRL_DCBUS_VOLTAGE_NOM (48.0f) // 典型测试电压
#define CTRL_VIN_NOM           (24.0f) // 典型输入电压 (12V~30V中间值)
#define CTRL_RATED_CURRENT     (15.0f) // 额定负载电流

// DC-DC 标幺化基准值 (Per-Unit Base)
// 基准值应略大于物理极限，以防标幺值溢出 1.0
#define CTRL_VOLTAGE_BASE (80.0f)
#define CTRL_CURRENT_BASE (30.0f)

//=================================================================================================
// Hardware Abstraction Mapping

#include <ctl/component/hardware_preset/inverter_HB/GMP_LVFB_150_2ph_v2.h>
#include <ctl/component/hardware_preset/current_sensor/GMP_Quad_Sensor_Docker.h>

// ---------------------------------------------------------
// Inductor Current Sensing (From LVFB Onboard Shunt)
// ---------------------------------------------------------
// 假设 H 桥板载采样电阻为 10mΩ (0.01R)
#define CTRL_INDUCTOR_CURRENT_SENSITIVITY GMP_LVFB_150_2PH_CALC_I_GAIN(0.01f)
#define CTRL_INDUCTOR_CURRENT_BIAS        GMP_LVFB_150_2PH_ONBOARD_I_BIAS_V

// ---------------------------------------------------------
// Input Voltage Sensing (Vin)
// ---------------------------------------------------------
// 使用 56kΩ 降压电阻 (适合 ±154V 量程，覆盖 30V 绰绰有余且精度良好)
#define CTRL_VIN_VOLTAGE_SENSITIVITY QUAD_SENSOR_CALC_V_GAIN(56.0f)
#define CTRL_VIN_VOLTAGE_BIAS        QUAD_SENSOR_BASE_BIAS_V

// ---------------------------------------------------------
// Output Voltage Sensing (Vout)
// ---------------------------------------------------------
// 使用 56kΩ 降压电阻 (覆盖 60V 最大输出)
#define CTRL_VOUT_VOLTAGE_SENSITIVITY QUAD_SENSOR_CALC_V_GAIN(56.0f)
#define CTRL_VOUT_VOLTAGE_BIAS        QUAD_SENSOR_BASE_BIAS_V

// ---------------------------------------------------------
// Load Current Sensing (I_load)
// ---------------------------------------------------------
// 最大负载按 20A 考虑。选用 TMCS1133-B2A (±31.0A 量程，50mV/A 灵敏度)
#define CTRL_LOAD_CURRENT_SENSITIVITY QUAD_SENSOR_CALC_I_GAIN(TMCS1133_B2A_MV_A)
#define CTRL_LOAD_CURRENT_BIAS        QUAD_SENSOR_BASE_BIAS_V

//=================================================================================================
// System Protection Bounds (Derived from Hardware)

// 从传感器硬件极限反推保护阈值
#define CTRL_MAX_HW_VOLTAGE QUAD_SENSOR_CALC_V_MAX(56.0f)   // ~154V
#define CTRL_MAX_HW_CURRENT QUAD_SENSOR_CALC_I_MAX_A(50.0f) // ~33A (ADC 饱和点)

// 系统软件保护阈值设定
#define CTRL_PROT_VOUT_MAX (65.0f) // 允许 60V 稍微超调，超过 65V 立即触发硬件保护
#define CTRL_PROT_VIN_MAX  (40.0f) // 防止输入端接错高压源
#define CTRL_PROT_IL_MAX   (25.0f) // 电感绝对最大电流保护

//=================================================================================================
// Controller Settings

// Enable Discrete PID controller anti-saturation algorithm
#define _USE_DEBUG_DISCRETE_PID

// Enable ADC Calibrate (零点漂移校准非常重要，特别是电流环)
#define SPECIFY_ENABLE_ADC_CALIBRATE
#define TIMEOUT_ADC_CALIB_MS (3000)

// Using negative modulator logic
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC (0)

//=================================================================================================
// Board peripheral mapping (Example for C2000)

// PWM Channels for FSBB (Leg 1: Buck, Leg 2: Boost)
#define PHASE_BUCK_BASE  IRIS_EPWM1_BASE
#define PHASE_BOOST_BASE IRIS_EPWM2_BASE

// PWM Enable & Reset Ports
#define PWM_ENABLE_PORT IRIS_GPIO1
#define PWM_RESET_PORT  IRIS_GPIO3

// System LEDs
#define SYSTEM_LED     IRIS_LED1
#define CONTROLLER_LED IRIS_LED2

#endif // _FILE_CTRL_SETTINGS_H_
