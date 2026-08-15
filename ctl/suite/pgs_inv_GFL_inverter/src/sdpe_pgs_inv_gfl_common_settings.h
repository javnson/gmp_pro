/**
 * @file sdpe_pgs_inv_gfl_common_settings.h
 * @brief SDPE project bindings for PGS Grid-Following Inverter Common Settings.
 * @note Platform-independent control contract shared by all pgs_inv_GFL_inverter projects. Hardware timing, sensing, board resources and BUILD_LEVEL selection remain in each platform requirement.
 */

#ifndef _PROJECT_SDPE_PGS_INV_GFL_COMMON_SETTINGS_H_
#define _PROJECT_SDPE_PGS_INV_GFL_COMMON_SETTINGS_H_

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Platform-independent GFL controller settings. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_INV_GFL_COMMON_SDPE_PROJECT_ID "pgs_inv_gfl_common"
#define PGS_INV_GFL_COMMON_SDPE_PROJECT_SUITE "pgs_inv_GFL_inverter"
#define PGS_INV_GFL_COMMON_SDPE_PROJECT_VERSION "1.2.0"
#define PGS_INV_GFL_COMMON_SDPE_PROJECT_UPDATED_AT "2026-08-15"

//=================================================================================================
/**
 * @brief Control Algorithm.
 */

/**
 * @brief Enable the existing discrete PID anti-saturation path.
 */
#define _USE_DEBUG_DISCRETE_PID

/**
 * @brief Use DSOGI PLL instead of the default SRF PLL.
 */
// #define USING_DSOGI_PLL

/**
 * @brief Use the three-level NPC modulator instead of the two-level SPWM modulator.
 */
// #define USING_NPC_MODULATOR

/**
 * @brief Use four-leg 3D-SVPWM. This enables an independently controlled neutral-leg duty and permits zero-sequence QPR control; the selected board must provide a fourth PWM bridge leg.
 */
// #define USING_3D_SVPWM

/**
 * @brief Enable omega*C cross-coupling feed-forward in the BUILD_LEVEL 6 capacitor-voltage loop. Disable this switch to commission the voltage PI loop without decoupling.
 */
#define GFL_ENABLE_VOLTAGE_DECOUPLE

/**
 * @brief Enable circular magnitude limiting of the complete BUILD_LEVEL 6 d-q current reference.
 */
#define GFL_ENABLE_VOLTAGE_CIRCLE_LIMIT

/**
 * @brief Enable independent symmetric d/q-axis limiting of the complete BUILD_LEVEL 6 current reference.
 */
// #define GFL_ENABLE_VOLTAGE_SQUARE_LIMIT

/**
 * @brief Enable frequency-P and voltage-Q droop reference generation ahead of the BUILD_LEVEL 5 P/Q controller.
 */
#define GFL_ENABLE_PQ_DROOP

//=================================================================================================
/**
 * @brief Runtime.
 */

/**
 * @brief Enable startup ADC offset calibration. Only enable while all calibrated power inputs are in a known zero state.
 */
// #define SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Enable CiA402/GMP framework debug information.
 */
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

//=================================================================================================
/**
 * @brief Sampling.
 */

/**
 * @brief Capacitor-current source: direct measurement, current difference, or capacitor-voltage derivative.
 *        Options: (1), (2), (3)
 */
#define GFL_CAPACITOR_CURRENT_CALCULATE_MODE (3)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Enable the seven ADC slots used by the three-phase inverter SIL input ABI.
 */
#ifndef GMP_PIL_RX_MASK
#define GMP_PIL_RX_MASK (127)
#endif // GMP_PIL_RX_MASK

/**
 * @brief Enable four PWM slots and all sixteen monitor slots used by the GFL SIL output ABI.
 */
#ifndef GMP_PIL_TX_MASK
#define GMP_PIL_TX_MASK (4294901775)
#endif // GMP_PIL_TX_MASK

/**
 * @brief ADC offset calibrator filter cutoff frequency.
 */
#define GFL_ADC_CALIBRATOR_FC_HZ real2param(20.0)

/**
 * @brief ADC offset calibrator second-order filter quality factor.
 */
#define GFL_ADC_CALIBRATOR_Q real2param(0.707)

/**
 * @brief Minimum CiA402 delay before Operation Enabled.
 */
#define GFL_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief Nominal grid phase-voltage magnitude in controller per unit.
 */
#define GFL_GRID_VOLTAGE_PU real2param(0.33)

/**
 * @brief P/Q outer-loop execution frequency in hertz.
 */
#define GFL_PQ_LOOP_FREQUENCY_HZ real2param(1000.0)

/**
 * @brief Current-ISR to P/Q-loop execution divider.
 */
#define GFL_PQ_LOOP_DIVIDER ((uint32_t)(CONTROLLER_FREQUENCY / GFL_PQ_LOOP_FREQUENCY_HZ))

/**
 * @brief Active-power loop proportional gain from P error PU to d-axis current PU.
 */
#define GFL_PQ_ACTIVE_KP real2param(0.75)

/**
 * @brief Active-power loop integral gain in inverse seconds.
 */
#define GFL_PQ_ACTIVE_KI real2param(0.001)

/**
 * @brief Reactive-power loop proportional gain from Q error PU to q-axis current PU.
 */
#define GFL_PQ_REACTIVE_KP real2param(0.75)

/**
 * @brief Reactive-power loop integral gain in inverse seconds.
 */
#define GFL_PQ_REACTIVE_KI real2param(0.001)

/**
 * @brief Circular magnitude limit applied to the d/q current reference produced by the P/Q loop.
 */
#define GFL_PQ_CURRENT_LIMIT_PU real2param(1.0)

/**
 * @brief PQ droop frequency and voltage measurement low-pass cutoff.
 */
#define GFL_PQ_DROOP_LPF_HZ real2param(10.0)

/**
 * @brief Additional active-power command per hertz of frequency deficit.
 */
#define GFL_PQ_DROOP_P_GAIN_PU_PER_HZ real2param(0.10)

/**
 * @brief Additional reactive-power command per PU of voltage deficit.
 */
#define GFL_PQ_DROOP_Q_GAIN_PU_PER_V_PU real2param(0.50)

/**
 * @brief Minimum active-power reference after PQ droop.
 */
#define GFL_PQ_DROOP_P_MIN_PU real2param(-0.80)

/**
 * @brief Maximum active-power reference after PQ droop.
 */
#define GFL_PQ_DROOP_P_MAX_PU real2param(0.80)

/**
 * @brief Minimum reactive-power reference after PQ droop.
 */
#define GFL_PQ_DROOP_Q_MIN_PU real2param(-0.80)

/**
 * @brief Maximum reactive-power reference after PQ droop.
 */
#define GFL_PQ_DROOP_Q_MAX_PU real2param(0.80)

/**
 * @brief BUILD_LEVEL 1 d-axis open-loop voltage command.
 */
#define GFL_OPEN_LOOP_VD_PU real2param(0.6)

/**
 * @brief BUILD_LEVEL 1 q-axis open-loop voltage command.
 */
#define GFL_OPEN_LOOP_VQ_PU real2param(0.6)

/**
 * @brief PLL lock-error threshold in controller per unit.
 */
#define CTRL_SPLL_EPSILON real2ctrl(0.005)

/**
 * @brief Default active-power reference. Positive power exports energy to the grid.
 */
#define GFL_ACTIVE_POWER_REF_PU real2param(0.1)

/**
 * @brief Default reactive-power reference using Q = vq*id - vd*iq.
 */
#define GFL_REACTIVE_POWER_REF_PU real2param(0.0)

/**
 * @brief BUILD_LEVEL 2 d-axis current command.
 */
#define GFL_CURRENT_LEVEL2_ID_PU real2param(0.1)

/**
 * @brief BUILD_LEVEL 2 q-axis current command.
 */
#define GFL_CURRENT_LEVEL2_IQ_PU real2param(0.1)

/**
 * @brief BUILD_LEVEL 3 grid-connected d-axis current command.
 */
#define GFL_CURRENT_LEVEL3_ID_PU real2param(0.1)

/**
 * @brief BUILD_LEVEL 3 grid-connected q-axis current command.
 */
#define GFL_CURRENT_LEVEL3_IQ_PU real2param(0.0)

/**
 * @brief BUILD_LEVEL 4 d-axis current command.
 */
#define GFL_CURRENT_LEVEL4_ID_PU real2param(0.6)

/**
 * @brief BUILD_LEVEL 4 q-axis current command.
 */
#define GFL_CURRENT_LEVEL4_IQ_PU real2param(0.6)

/**
 * @brief BUILD_LEVEL 6 positive-sequence d-axis capacitor phase-voltage reference in per unit.
 */
#define GFL_STANDALONE_VD_PU real2param(0.50)

/**
 * @brief BUILD_LEVEL 6 positive-sequence q-axis capacitor phase-voltage reference in per unit.
 */
#define GFL_STANDALONE_VQ_PU real2param(0.0)

/**
 * @brief BUILD_LEVEL 6 capacitor-voltage outer-loop bandwidth in hertz; keep it well below the inner current-loop bandwidth.
 */
#define GFL_VOLTAGE_LOOP_BW_HZ real2param(100.0)

/**
 * @brief BUILD_LEVEL 6 ordinary PI zero frequency in hertz.
 */
#define GFL_VOLTAGE_LOOP_ZERO_HZ real2param(20.0)

/**
 * @brief Circular magnitude limit for the complete BUILD_LEVEL 6 PI plus feed-forward current reference. The final limited output is returned to the ordinary PID integrators by clamping correction.
 */
#define GFL_VOLTAGE_CIRCLE_LIMIT_PU real2param(0.80)

/**
 * @brief Independent symmetric d/q-axis limit for the complete BUILD_LEVEL 6 PI plus feed-forward current reference.
 */
#define GFL_VOLTAGE_SQUARE_LIMIT_PU real2param(0.80)

/**
 * @brief Zero-sequence current QPR proportional gain used when 3D-SVPWM is enabled.
 */
#define GFL_ZERO_QPR_KP real2param(0.10)

/**
 * @brief Zero-sequence current QPR resonant gain used when 3D-SVPWM is enabled.
 */
#define GFL_ZERO_QPR_KR real2param(50.0)

/**
 * @brief Zero-sequence QPR resonant bandwidth/cutoff frequency in hertz.
 */
#define GFL_ZERO_QPR_CUTOFF_HZ real2param(5.0)

/**
 * @brief Symmetric zero-axis voltage-command limit for four-wire operation.
 */
#define GFL_ZERO_VOLTAGE_LIMIT_PU real2param(0.20)

/**
 * @brief Nominal grid frequency in hertz.
 */
#define GFL_GRID_FREQUENCY_HZ real2param(50.0)

// User project tail code
/* Accept the historical PIL spelling while new projects use the canonical switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif
/* Four-leg hardware must explicitly acknowledge that A/B/C/N PWM and protection are mapped. */
#if defined(USING_3D_SVPWM) && !defined(SPECIFY_PC_ENVIRONMENT) && !defined(GFL_3D_SVPWM_PLATFORM_MAPPED)
#error Define_GFL_3D_SVPWM_PLATFORM_MAPPED_only_after_mapping_all_four_PWM_legs
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_INV_GFL_COMMON_SETTINGS_H_
