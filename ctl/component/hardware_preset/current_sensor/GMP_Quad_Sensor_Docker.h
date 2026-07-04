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
 * @brief Configuration constants and calculations for the Quad Channel Sensor hardware layer.
 */

/** @brief The official identification name of the hardware board. */
#define QUAD_SENSOR_BOARD_NAME "Quad Channel Sensor V6.2"

/** @brief Hardware production or release date encoded as YYYYMMDD. */
#define QUAD_SENSOR_BOARD_DATE 20260630U

#ifndef QUAD_SENSOR_BASE_BIAS_V
/** @brief Absolute center bias voltage for the unified 3.3V system. */
#define QUAD_SENSOR_BASE_BIAS_V (1.65f)
#endif // QUAD_SENSOR_BASE_BIAS_V

// ============================================================================
// 1. Voltage Sensing Configuration (AMC1350 + TLV9061)
// ============================================================================

/** @brief Bandwidth of the voltage sensor analog signal chain representing the dominant system pole. */
#define QUAD_SENSOR_VOLTAGE_BW_HZ (100.0e3f)

/**
 * @brief Macro to calculate the specific voltage sensitivity.
 * @param _r_low  Low-side resistor value.
 * @param _r_high High-side resistor value.
 */
#define QUAD_SENSOR_CALC_V_SENSITIVE(_r_low,_r_high) ((((_r_low)/((_r_low)+(_r_high*6)))*0.4))

/**
 * @brief Macro to calculate the maximum measurable voltage based on base bias and sensitivity.
 * @param _r_low  Low-side resistor value.
 * @param _r_high High-side resistor value.
 */
#define QUAD_SENSOR_CALC_V_MAX(_r_low,_r_high) (QUAD_SENSOR_BASE_BIAS_V/QUAD_SENSOR_CALC_V_SENSITIVE(_r_low,_r_high))

/** @brief Default low-side resistance value for the voltage divider. */
#define QUAD_SENSOR_DEFAULT_R_LOW ((15e3))

/** @brief Default high-side resistance value for the voltage divider. */
#define QUAD_SENSOR_DEFAULT_R_HIGH ((71.6e3))

/** @brief Evaluated default voltage sensitivity using standard presets. */
#define QUAD_SENSOR_V_SENSITIVE QUAD_SENSOR_CALC_V_SENSITIVE(QUAD_SENSOR_DEFAULT_R_LOW, QUAD_SENSOR_DEFAULT_R_HIGH)

/** @brief Evaluated maximum measurable voltage using standard presets. */
#define QUAD_SENSOR_V_MAX QUAD_SENSOR_CALC_V_MAX(QUAD_SENSOR_DEFAULT_R_LOW, QUAD_SENSOR_DEFAULT_R_HIGH)


// ============================================================================
// 2. Current Sensing Configuration (TMCS1133 Series)
// ============================================================================

#ifndef QUAD_SENSOR_CURRENT_BW_HZ
/** @brief Bandwidth of the TMCS1133 Hall-effect current sensor. */
#define QUAD_SENSOR_CURRENT_BW_HZ (1.0e6f) // 1 MHz
#endif // QUAD_SENSOR_CURRENT_BW_HZ

#ifndef TMCS1133_B7A_S_MV_A
/** @brief Sensitivity of TMCS1133 B7A variant in mV/A (Typical Range: +/-77.5A). */
#define TMCS1133_B7A_S_MV_A (20.0f)
#endif // TMCS1133_B7A_S_MV_A

#ifndef TMCS1133_B1A_S_MV_A
/** @brief Sensitivity of TMCS1133 B1A variant in mV/A (Typical Range: +/-62.0A). */
#define TMCS1133_B1A_S_MV_A (25.0f)
#endif // TMCS1133_B1A_S_MV_A

#ifndef TMCS1133_B8A_S_MV_A
/** @brief Sensitivity of TMCS1133 B8A variant in mV/A (Typical Range: +/-47.0A). */
#define TMCS1133_B8A_S_MV_A (33.0f)
#endif // TMCS1133_B8A_S_MV_A

#ifndef TMCS1133_B2A_S_MV_A
/** @brief Sensitivity of TMCS1133 B2A variant in mV/A (Typical Range: +/-31.0A). */
#define TMCS1133_B2A_S_MV_A (50.0f)
#endif // TMCS1133_B2A_S_MV_A

#ifndef TMCS1133_B3A_S_MV_A
/** @brief Sensitivity of TMCS1133 B3A variant in mV/A (Typical Range: +/-20.7A). */
#define TMCS1133_B3A_S_MV_A (75.0f)
#endif // TMCS1133_B3A_S_MV_A

#ifndef TMCS1133_B5A_S_MV_A
/** @brief Sensitivity of TMCS1133 B5A variant in mV/A (Typical Range: +/-10.3A). */
#define TMCS1133_B5A_S_MV_A (150.0f)
#endif // TMCS1133_B5A_S_MV_A

/**
 * @brief Macro to convert current sensor sensitivity from mV/A to V/A.
 * @param sensitivity_mv_a Sensitivity value in mV/A.
 */
#define QUAD_SENSOR_CALC_I_SENSITIVE(sensitivity_mv_a) ((sensitivity_mv_a) / 1000.0f)

/**
 * @brief Macro to calculate the maximum measurable current in Amperes.
 * @param sensitivity_mv_a Sensitivity value in mV/A.
 */
#define QUAD_SENSOR_CALC_I_MAX_A(sensitivity_mv_a) (QUAD_SENSOR_BASE_BIAS_V / QUAD_SENSOR_CALC_I_GAIN(sensitivity_mv_a))

/** @brief Configured maximum current limit based on the selected sensor variant (TMCS1133_B5A). */
#define QUAD_SENSOR_I_MAX_A     QUAD_SENSOR_CALC_I_MAX_A(TMCS1133_B5A_S_MV_A)

/** @brief Configured default current sensitivity based on the selected sensor variant (TMCS1133_B5A). */
#define QUAD_SENSOR_I_SENSITIVE QUAD_SENSOR_CALC_I_SENSITIVE(TMCS1133_B5A_S_MV_A)

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif // _QUAD_SENSOR_DOCKER_H_
