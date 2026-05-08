/**
 * @file GMP_LVFB_150_2ph_v2.h
 * @brief Hardware Abstraction Layer (HAL) for the GMP LVFB 150V 2ph V2 Inverter Board.
 * @note  Based on schematic LVFB_100_2PH.pdf
 * Topology: 2-Phase Full-Bridge (H-Bridge)
 * Gate Driver: UCC21520DW
 * MOSFET: BSC093N15NS5 (150V, 55A @ 100°C)
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
 */

// ============================================================================
// 1. Nameplate & Identification
// ============================================================================
#define GMP_LVFB_150_2PH_NAME        "GMP_LVFB_150_2ph_v2"
#define GMP_LVFB_150_2PH_GATE_DRIVER "UCC21520DWR"
#define GMP_LVFB_150_2PH_MOSFET      "BSC093N15NS5"

// ============================================================================
// 2. Physical Operating Limits (Derated for Safety)
// ============================================================================
// MOSFET 标称 150V，保留 20% 裕度，最大允许直流母线为 120V
#define GMP_LVFB_150_2PH_VBUS_MAX_V (120.0f)

// MOSFET 在 100°C 结温下的额定电流为 55A
#define GMP_LVFB_150_2PH_CURRENT_MAX_PEAK_A (55.0f)
#define GMP_LVFB_150_2PH_CURRENT_MAX_RMS_A  (35.0f) // 约等于 55A / 1.414

// ============================================================================
// 3. Hardware Dead-time
// ============================================================================
// 根据原理图说明: DT(ns) = 10 * R_DT = 10 * 52.3k = 523 ns
#define GMP_LVFB_150_2PH_HW_DEADTIME_NS (523.0f)

// ============================================================================
// 4. On-board Sensing (Current Sensor Bandwidth & Gains)
// ============================================================================
// 电流传感器模拟链路带宽 (Spice 仿真极点)
#define GMP_LVFB_150_2PH_CURRENT_BW_HZ    (115.84e3f) // 115.84 kHz
#define GMP_LVFB_150_2PH_ONBOARD_I_BIAS_V (1.67f)

/**
 * @brief 动态计算板载 AMC1311 电流采样的灵敏度 (V/A)
 * @param r_shunt_ohm 分流器电阻值 (Ohm)
 * @note  原理图中 0.01Ω 对应理论增益为 84.4 mV/A (0.0844 V/A)
 */
#define GMP_LVFB_150_2PH_CALC_I_GAIN(r_shunt_ohm) (8.44f * (r_shunt_ohm))

/**
 * @brief 计算在给定的分流器下，ADC 端口不饱和所能测量的最大峰值电流 (A)
 * @note  假设 ADC 上限为 3.3V，中心偏置为 1.67V，最大正向摆幅为 1.63V
 */
#define GMP_LVFB_150_2PH_CALC_I_MAX_A(r_shunt_ohm) (1.63f / GMP_LVFB_150_2PH_CALC_I_GAIN(r_shunt_ohm))

/** @} */

#ifdef __cplusplus
}
#endif

#endif // _GMP_LVFB_150_2PH_V2_H_
