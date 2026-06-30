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
#ifndef QUAD_SENSOR_BASE_BIAS_V
#define QUAD_SENSOR_BASE_BIAS_V (1.65f)
#endif // QUAD_SENSOR_BASE_BIAS_V

// ============================================================================
// 1. Voltage Sensing Configuration (AMC1350 + TLV9061)
// ============================================================================

// 电压传感器模拟链路带宽 (系统主导极点)
#define QUAD_SENSOR_VOLTAGE_BW_HZ (100.0e3f)

#define QUAD_SENSOR_CALC_V_SENSITIVE(_r_low,_r_high) ((((_r_low)/((_r_low)+(_r_high*6)))*0.4))

#define QUAD_SENSOR_CALC_V_MAX(_r_low,_r_high) (QUAD_SENSOR_BASE_BIAS_V/QUAD_SENSOR_CALC_V_SENSITIVE(_r_low,_r_high))

#define QUAD_SENSOR_DEFAULT_R_LOW ((15e3))

#define QUAD_SENSOR_DEFAULT_R_HIGH ((71.6e3))

#define QUAD_SENSOR_V_SENSITIVE QUAD_SENSOR_CALC_V_SENSITIVE(QUAD_SENSOR_DEFAULT_R_LOW, QUAD_SENSOR_DEFAULT_R_HIGH)

#define QUAD_SENSOR_V_MAX QUAD_SENSOR_CALC_V_MAX(QUAD_SENSOR_DEFAULT_R_LOW, QUAD_SENSOR_DEFAULT_R_HIGH)


// ============================================================================
// 2. Current Sensing Configuration (TMCS1133 Series)
// ============================================================================
// TMCS1133 霍尔电流传感器带宽
#define QUAD_SENSOR_CURRENT_BW_HZ (1.0e6f) // 1 MHz

// 常见 TMCS1133 芯片后缀对应的灵敏度 (mV/A)
#define TMCS1133_B7A_S_MV_A (20.0f)  // Typ Range: ±77.5A
#define TMCS1133_B1A_S_MV_A (25.0f)  // Typ Range: ±62.0A
#define TMCS1133_B8A_S_MV_A (33.0f)  // Typ Range: ±47.0A
#define TMCS1133_B2A_S_MV_A (50.0f)  // Typ Range: ±31.0A
#define TMCS1133_B3A_S_MV_A (75.0f)  // Typ Range: ±20.7A
#define TMCS1133_B5A_S_MV_A (150.0f) // Typ Range: ±10.3A


#define QUAD_SENSOR_CALC_I_SENSITIVE(sensitivity_mv_a) ((sensitivity_mv_a) / 1000.0f)

#define QUAD_SENSOR_CALC_I_MAX_A(sensitivity_mv_a) (QUAD_SENSOR_BASE_BIAS_V / QUAD_SENSOR_CALC_I_GAIN(sensitivity_mv_a))

#define QUAD_SENSOR_I_MAX_A     QUAD_SENSOR_CALC_I_MAX_A(TMCS1133_B5A_S_MV_A)

#define QUAD_SENSOR_I_SENSITIVE QUAD_SENSOR_CALC_I_SENSATIVE(TMCS1133_B5A_S_MV_A)

/** @} */

#ifdef __cplusplus
}
#endif

#endif // _QUAD_SENSOR_DOCKER_H_
