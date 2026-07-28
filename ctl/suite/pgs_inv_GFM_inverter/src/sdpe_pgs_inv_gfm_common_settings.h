/**
 * @file sdpe_pgs_inv_gfm_common_settings.h
 * @brief SDPE project bindings for PGS Grid-Forming Inverter Common Settings.
 * @note Platform-independent control contract for the replaceable grid-forming outer loop, voltage loop, GFL-derived current core, and PLL-to-GFM transition.
 */

#ifndef _PROJECT_SDPE_PGS_INV_GFM_COMMON_SETTINGS_H_
#define _PROJECT_SDPE_PGS_INV_GFM_COMMON_SETTINGS_H_

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Platform-independent GFM controller settings. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_INV_GFM_COMMON_SDPE_PROJECT_ID "pgs_inv_gfm_common"
#define PGS_INV_GFM_COMMON_SDPE_PROJECT_SUITE "pgs_inv_GFM_inverter"
#define PGS_INV_GFM_COMMON_SDPE_PROJECT_VERSION "1.0.0"
#define PGS_INV_GFM_COMMON_SDPE_PROJECT_UPDATED_AT "2026-07-28"

//=================================================================================================
/**
 * @brief Control Algorithm.
 */

/**
 * @brief Enable the established discrete controller anti-saturation path.
 */
#define _USE_DEBUG_DISCRETE_PID

/**
 * @brief Use DSOGI PLL instead of the default SRF PLL during synchronization.
 */
// #define USING_DSOGI_PLL

/**
 * @brief Use four-leg 3D-SVPWM and permit zero-sequence QPR control.
 */
// #define USING_3D_SVPWM

/**
 * @brief Enable omega*C capacitor coupling feed-forward in BUILD_LEVEL 3 and 5.
 */
#define GFM_ENABLE_VOLTAGE_DECOUPLE

/**
 * @brief Enable circular limiting of the complete voltage-loop current reference.
 */
#define GFM_ENABLE_VOLTAGE_CIRCLE_LIMIT

/**
 * @brief Enable independent d/q-axis limiting of the complete voltage-loop current reference.
 */
// #define GFM_ENABLE_VOLTAGE_SQUARE_LIMIT

//=================================================================================================
/**
 * @brief Runtime.
 */

/**
 * @brief Enable startup ADC offset calibration only with known zero inputs.
 */
// #define SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Enable processor-in-the-loop input/output handling.
 */
// #define ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Nominal grid phase-voltage magnitude in controller per unit.
 */
#define GFM_GRID_VOLTAGE_PU (0.33f)

/**
 * @brief Nominal grid-forming and synchronization frequency.
 */
#define GFM_GRID_FREQUENCY_HZ (50.0f)

/**
 * @brief BUILD_LEVEL 1 d-axis open-loop voltage command.
 */
#define GFM_OPEN_LOOP_VD_PU (0.50f)

/**
 * @brief BUILD_LEVEL 1 q-axis open-loop voltage command.
 */
#define GFM_OPEN_LOOP_VQ_PU (0.0f)

/**
 * @brief BUILD_LEVEL 2 d-axis current command using the internal ramp angle.
 */
#define GFM_CURRENT_LEVEL2_ID_PU (0.10f)

/**
 * @brief BUILD_LEVEL 2 q-axis current command using the internal ramp angle.
 */
#define GFM_CURRENT_LEVEL2_IQ_PU (0.10f)

/**
 * @brief BUILD_LEVEL 4 PLL-oriented d-axis grid current command.
 */
#define GFM_CURRENT_LEVEL4_ID_PU (0.10f)

/**
 * @brief BUILD_LEVEL 4 PLL-oriented q-axis grid current command.
 */
#define GFM_CURRENT_LEVEL4_IQ_PU (0.0f)

/**
 * @brief BUILD_LEVEL 3 fixed voltage reference and BUILD_LEVEL 5 nominal droop voltage.
 */
#define GFM_VOLTAGE_VD_PU (0.50f)

/**
 * @brief BUILD_LEVEL 3 fixed q-axis voltage reference.
 */
#define GFM_VOLTAGE_VQ_PU (0.0f)

/**
 * @brief LC capacitor-voltage loop bandwidth.
 */
#define GFM_VOLTAGE_LOOP_BW_HZ (100.0f)

/**
 * @brief LC capacitor-voltage ordinary PI zero frequency.
 */
#define GFM_VOLTAGE_LOOP_ZERO_HZ (20.0f)

/**
 * @brief Circular magnitude limit of the complete voltage-loop current reference.
 */
#define GFM_VOLTAGE_CIRCLE_LIMIT_PU (0.80f)

/**
 * @brief Independent symmetric d/q-axis voltage-loop current-reference limit.
 */
#define GFM_VOLTAGE_SQUARE_LIMIT_PU (0.80f)

/**
 * @brief Active-power to frequency droop slope in Hz per power PU.
 */
#define GFM_DROOP_P_HZ_PER_PU (0.50f)

/**
 * @brief Reactive-power to voltage droop slope in voltage PU per reactive-power PU.
 */
#define GFM_DROOP_Q_V_PER_PU (0.05f)

/**
 * @brief Droop active/reactive-power measurement low-pass cutoff.
 */
#define GFM_DROOP_POWER_LPF_HZ (10.0f)

/**
 * @brief Maximum absolute droop frequency deviation from nominal.
 */
#define GFM_DROOP_FREQUENCY_DELTA_LIMIT_HZ (2.0f)

/**
 * @brief Minimum voltage magnitude requested by the droop module.
 */
#define GFM_DROOP_VOLTAGE_MIN_PU (0.40f)

/**
 * @brief Maximum voltage magnitude requested by the droop module.
 */
#define GFM_DROOP_VOLTAGE_MAX_PU (0.60f)

/**
 * @brief Default active-power reference for the droop algorithm.
 */
#define GFM_DROOP_ACTIVE_POWER_REF_PU (0.0f)

/**
 * @brief Default reactive-power reference for the droop algorithm.
 */
#define GFM_DROOP_REACTIVE_POWER_REF_PU (0.0f)

/**
 * @brief PLL-to-grid-forming phasor and current-command blend duration.
 */
#define GFM_TRANSITION_TIME_S (0.10f)

/**
 * @brief Continuous PLL-lock duration required before requesting grid-forming takeover.
 */
#define GFM_SYNC_HOLD_TIME_S (0.20f)

/**
 * @brief Maximum instantaneous PLL q-axis error during the continuous grid-forming synchronization hold.
 */
#define GFM_SYNC_PLL_ERROR_PU (0.08f)

/**
 * @brief Tracking-mode d-axis current command before grid-forming takeover.
 */
#define GFM_SYNC_ID_PU (0.0f)

/**
 * @brief Tracking-mode q-axis current command before grid-forming takeover.
 */
#define GFM_SYNC_IQ_PU (0.0f)

/**
 * @brief Zero-sequence current QPR proportional gain for four-wire operation.
 */
#define GFM_ZERO_QPR_KP (0.10f)

/**
 * @brief Zero-sequence current QPR resonant gain.
 */
#define GFM_ZERO_QPR_KR (50.0f)

/**
 * @brief Zero-sequence QPR resonant bandwidth.
 */
#define GFM_ZERO_QPR_CUTOFF_HZ (5.0f)

/**
 * @brief Symmetric zero-axis voltage-command limit.
 */
#define GFM_ZERO_VOLTAGE_LIMIT_PU (0.20f)

/**
 * @brief ADC offset calibrator cutoff.
 */
#define GFM_ADC_CALIBRATOR_FC_HZ (20.0f)

/**
 * @brief ADC offset calibrator quality factor.
 */
#define GFM_ADC_CALIBRATOR_Q (0.707f)

/**
 * @brief Minimum CiA402 delay before Operation Enabled.
 */
#define GFM_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief PLL lock-error threshold used before grid-forming takeover.
 */
#define CTRL_SPLL_EPSILON ((float2ctrl(0.005)))

// User project tail code
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif
#if defined(USING_3D_SVPWM) && !defined(SPECIFY_PC_ENVIRONMENT) && !defined(GFM_3D_SVPWM_PLATFORM_MAPPED)
#error Define_GFM_3D_SVPWM_PLATFORM_MAPPED_only_after_mapping_all_four_PWM_legs
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_INV_GFM_COMMON_SETTINGS_H_
