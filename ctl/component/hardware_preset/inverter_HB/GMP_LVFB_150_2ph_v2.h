/**
 * @file GMP_LVFB_150_2ph_v2.h
 * @brief Hardware Abstraction Layer (HAL) for the GMP LVFB 150V 2ph V2 Inverter Board.
 * @note  Based on schematic LVFB_100_2PH.pdf
 * Topology: 2-Phase Full-Bridge (H-Bridge)
 * Gate Driver: UCC21520DW
 * MOSFET: BSC093N15NS5 (150V, 55A @ 100°„C)
 */

#ifndef _GMP_LVFB_150_2PH_V2_H_
#define _GMP_LVFB_150_2PH_V2_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup hal_gmp_lvfb_150_2ph GMP LVFB 150V 2-Phase Full-Bridge Hardware Preset
 * @{
 * @brief Configuration constants, peripheral definitions, and scaling calculations for the GMP LVFB board.
 */

// ============================================================================
// 1. Nameplate & Identification
// ============================================================================

/** @brief Model identification name of the hardware board. */
#define GMP_LVFB_NAME        "GMP LVFB 2ph V6.1"

/** @brief Part number of the integrated gate driver IC. */
#define GMP_LVFB_GATE_DRIVER "UCC21520DWR"

/** @brief Part number of the power MOSFET switches. */
#define GMP_LVFB_MOSFET      "BSC093N15NS5"

/** @brief Part number of the on-board current sensing IC. */
#define GMP_LVFB_CURRENT_SENSOR_NAME "TMCS1133B5A"

/** @brief Part number of the on-board isolated voltage sensing amplifier. */
#define GMP_LVFB_VOLTAGE_SENSOR_NAME "AMC1311BDWVR"

// ============================================================================
// 2. Physical Operating Limits (Derated for Safety)
// ============================================================================

/** @brief Maximum allowable DC bus voltage, derated for safe operational margins. */
#define GMP_LVFB_VBUS_MAX_V (120.0f)

/** @brief Maximum allowable instantaneous peak phase current. */
#define GMP_LVFB_CURRENT_MAX_PEAK_A (55.0f)

/** @brief Maximum allowable continuous root-mean-square (RMS) phase current. */
#define GMP_LVFB_CURRENT_MAX_RMS_A  (35.0f)

// ============================================================================
// 3. Hardware Dead-time
// ============================================================================

/** @brief Hardware dead-time duration in nanoseconds, determined by DT resistor (DT = 10 * R_DT). */
#define GMP_LVFB_HW_DEADTIME_NS (523.0f)

// ============================================================================
// 4. On-board Current Sensing
// ============================================================================

/** @brief Bandwidth of the current sensing signal conditioning circuit. */
#define GMP_LVFB_CURRENT_BW_HZ  (300.0e3f)

/** @brief Reference bias voltage for the current sensor operational amplifier/ADC input. */
#define GMP_LVFB_CURRENT_BIAS_V (1.65f)


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
#define GMP_LVFB_SENSOR_CALC_I_SENSITIVE(sensitivity_mv_a) ((sensitivity_mv_a) / 1000.0f)

/**
 * @brief Macro to calculate the maximum measurable current based on system parameters.
 * @param sensitivity_mv_a Sensitivity value in mV/A.
 */
#define GMP_LVFB_SENSOR_CALC_I_MAX_A(sensitivity_mv_a) (QUAD_SENSOR_BASE_BIAS_V / QUAD_SENSOR_CALC_I_GAIN(sensitivity_mv_a))


/** @brief Configured maximum current limit based on the selected sensor variant (TMCS1133_B5A). */
#define GMP_LVFB_SENSOR_I_MAX_A     GMP_LVFB_SENSOR_CALC_I_MAX_A(TMCS1133_B5A_S_MV_A)

/** @brief Configured default current sensitivity based on the selected sensor variant (TMCS1133_B5A). */
#define GMP_LVFB_SENSOR_I_SENSITIVE GMP_LVFB_SENSOR_CALC_I_SENSITIVE(TMCS1133_B5A_S_MV_A)

// ============================================================================
// 5. On-board Voltage Sensing
// ============================================================================

/** @brief Base reference scale voltage for the analog-to-digital converter interface. */
#define GMP_LVFB_VOLTAGE_BASE_V (3.3f)
#define GMP_LVFB_VOLTAGE_BIAS_V (0.0f)

/** @brief Bandwidth of the voltage sensing attenuation and filtering network. */
#define GMP_LVFB_SENSOR_VOLTAGE_BW_HZ (114.47e3f)

/**
 * @brief Macro to calculate voltage sensing gain/sensitivity.
 * @param _r_low  Low-side resistor value.
 * @param _r_high High-side resistor value.
 */
#define GMP_LVFB_SENSOR_CALC_V_SENSITIVE(_r_low,_r_high) ((((_r_low)/((_r_low)+(_r_high*3))*1.65f)))

/**
 * @brief Macro to calculate the maximum safe measurable input voltage.
 * @param _r_low  Low-side resistor value.
 * @param _r_high High-side resistor value.
 */
#define GMP_LVFB_SENSOR_CALC_V_MAX(_r_low,_r_high) (GMP_LVFB_SENSOR_BASE_V/GMP_LVFB_SENSOR_CALC_V_SENSITIVE(_r_low,_r_high))

/** @brief Default resistance value for the low-side voltage divider resistor. */
#define GMP_LVFB_SENSOR_DEFAULT_R_LOW ((15e3))

/** @brief Default resistance value for the high-side voltage divider resistor network. */
#define GMP_LVFB_SENSOR_DEFAULT_R_HIGH ((300e3))

/** @brief Evaluated default voltage sensing gain based on the preset divider values. */
#define GMP_LVFB_SENSOR_V_SENSITIVE GMP_LVFB_SENSOR_CALC_V_SENSITIVE(GMP_LVFB_SENSOR_DEFAULT_R_LOW, GMP_LVFB_SENSOR_DEFAULT_R_HIGH)

/** @brief Evaluated maximum full-scale measurable voltage based on the preset divider values. */
#define GMP_LVFB_SENSOR_V_MAX GMP_LVFB_SENSOR_CALC_V_MAX(GMP_LVFB_SENSOR_DEFAULT_R_LOW, GMP_LVFB_SENSOR_DEFAULT_R_HIGH)

/** @} */

#ifdef __cplusplus
}
#endif

#endif // _GMP_LVFB_150_2PH_V2_H_
