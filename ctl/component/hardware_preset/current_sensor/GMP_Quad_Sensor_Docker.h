/**
 * @file QuadSensorDocker.h
 * @brief Hardware Preset for the 4-Channel Isolated Sensor Docker.
 * @note  Based on schematic QuadSensorDocker.pdf
 * Voltage Sensor: AMC1350 + TLV9061
 * Current Sensor: TMCS1133 Series
 */

#ifndef _QUAD_SENSOR_DOCKER_H_
#define _QUAD_SENSOR_DOCKER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_quad_sensor Quad Channel Sensor Docker Preset
 * @{
 */

// 统一的 3.3V 系统绝对中心偏置
#define QUAD_SENSOR_BASE_BIAS_V (1.65f)

// ============================================================================
// 1. Voltage Sensing Configuration (AMC1350 + TLV9061)
// ============================================================================
// 电压传感器模拟链路带宽 (系统主导极点)
#define QUAD_SENSOR_VOLTAGE_BW_HZ (408.0e3f) // 408 kHz

/**
 * @brief 动态计算电压传感器最大物理量程 (V)
 * @param r_high_kohm 单个高压侧分压电阻的阻值 (kΩ) 
 * @note  根据安规标定：选择 56kΩ 时，最大不失真测量范围为 ±154V
 */
#define QUAD_SENSOR_CALC_V_MAX(r_high_kohm) (154.0f * ((r_high_kohm) / 56.0f))

/**
 * @brief 动态计算电压传感器总灵敏度 (V/V)
 * @note  最大测量电压映射到 1.65V 的 ADC 摆幅
 */
#define QUAD_SENSOR_CALC_V_GAIN(r_high_kohm) (QUAD_SENSOR_BASE_BIAS_V / QUAD_SENSOR_CALC_V_MAX(r_high_kohm))

// ============================================================================
// 2. Current Sensing Configuration (TMCS1133 Series)
// ============================================================================
// TMCS1133 霍尔电流传感器带宽
#define QUAD_SENSOR_CURRENT_BW_HZ (1.0e6f) // 1 MHz

// 常见 TMCS1133 芯片后缀对应的灵敏度 (mV/A)
#define TMCS1133_B7A_MV_A (20.0f)  // Typ Range: ±77.5A
#define TMCS1133_B1A_MV_A (25.0f)  // Typ Range: ±62.0A
#define TMCS1133_B8A_MV_A (33.0f)  // Typ Range: ±47.0A
#define TMCS1133_B2A_MV_A (50.0f)  // Typ Range: ±31.0A
#define TMCS1133_B3A_MV_A (75.0f)  // Typ Range: ±20.7A
#define TMCS1133_B5A_MV_A (150.0f) // Typ Range: ±10.3A

/**
 * @brief 动态计算电流传感器灵敏度 (V/A)
 */
#define QUAD_SENSOR_CALC_I_GAIN(sensitivity_mv_a) ((sensitivity_mv_a) / 1000.0f)

/**
 * @brief 动态计算电流传感器 ADC 级最大物理量程 (A)
 * @note  超出此值将导致 3.3V ADC 钳位饱和
 */
#define QUAD_SENSOR_CALC_I_MAX_A(sensitivity_mv_a) (QUAD_SENSOR_BASE_BIAS_V / QUAD_SENSOR_CALC_I_GAIN(sensitivity_mv_a))

/** @} */

#ifdef __cplusplus
}
#endif

#endif // _QUAD_SENSOR_DOCKER_H_
