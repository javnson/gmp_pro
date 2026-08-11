/**
 * @file sdpe_pgs_inv_gfm_simulate_settings.h
 * @brief SDPE project bindings for PGS GFM Inverter Simulation.
 * @note Host simulation timing and sensing settings for the replaceable grid-forming controller.
 */

#ifndef _PROJECT_SDPE_PGS_INV_GFM_SIMULATE_SETTINGS_H_
#define _PROJECT_SDPE_PGS_INV_GFM_SIMULATE_SETTINGS_H_

#include <ctl/hardware_preset/grid_lc_filter/gmp_harmonia_3ph_lc_filter.h>
#include <ctl/hardware_preset/inverter_3ph/gmp_helios_3phganinv_lv.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
// SDPE extension point: add after_extern_open code in the Project Requirement Code page if needed.

// Common prefix code: PGS Grid-Forming Inverter Common Settings
/* Platform-independent GFM controller settings. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_INV_GFM_SIM_SDPE_PROJECT_ID "pgs_inv_gfm_simulate"
#define PGS_INV_GFM_SIM_SDPE_PROJECT_SUITE "pgs_inv_GFM_inverter"
#define PGS_INV_GFM_SIM_SDPE_PROJECT_VERSION "1.1.0"
#define PGS_INV_GFM_SIM_SDPE_PROJECT_UPDATED_AT "2026-08-08"

//=================================================================================================
/**
 * @brief Commissioning.
 */

/**
 * @brief BUILD_LEVEL descriptor: 1=open-loop voltage; 2=current loop with internal RG; 3=LC capacitor-voltage loop; 4=PLL-oriented grid current loop; 5=PLL synchronization followed by bumpless transfer to the SDPE-selected droop, VSM, or virtual-impedance grid-forming outer loop plus voltage loop.
 *        Options: (1), (2), (3), (4), (5)
 */
#define BUILD_LEVEL (5)

//=================================================================================================
/**
 * @brief Sampling.
 */

/**
 * @brief Number of directly sampled phase currents.
 *        Options: (2), (3)
 */
#define GFL_CURRENT_SAMPLE_PHASE_MODE (3)

/**
 * @brief Voltage sampling topology: 1=two line-to-line samples (Uab/Ubc), 2=two phase-to-neutral samples, 3=three phase-to-neutral samples. The supplied SIL models use mode 1.
 *        Options: (1), (2), (3)
 */
#define GFL_VOLTAGE_SAMPLE_PHASE_MODE (1)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Startup delay in milliseconds.
 */
#define CTRL_STARTUP_DELAY (50)

/**
 * @brief Simulation control step frequency.
 */
#define CONTROLLER_FREQUENCY (20e3)

/**
 * @brief Virtual PWM compare range.
 */
#define CTRL_PWM_CMP_MAX (3000)

/**
 * @brief Virtual PWM dead-band count.
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief Virtual ADC reference.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief DC-bus voltage base.
 */
#define CTRL_DCBUS_VOLTAGE (80.0f)

/**
 * @brief SVPWM phase-voltage base.
 */
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

/**
 * @brief Phase-current base.
 */
#define CTRL_CURRENT_BASE (10.0f)

/**
 * @brief Harmonia inductance.
 */
#define GFM_GRID_FILTER_INDUCTANCE_H (HARMONIA_3PH_LC_FILTER_INDUCTANCE_H)

/**
 * @brief Harmonia capacitance.
 */
#define GFM_GRID_FILTER_CAPACITANCE_F (HARMONIA_3PH_LC_FILTER_CAPACITANCE_F)

/**
 * @brief Grid-current sensitivity.
 */
#define CTRL_GRID_CURRENT_SENSITIVITY (HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSITIVITY_MV_A * 0.001f)

/**
 * @brief Grid-current bias.
 */
#define CTRL_GRID_CURRENT_BIAS (HARMONIA_3PH_LC_FILTER_PH_CURRENT_ZERO_BIAS_V)

/**
 * @brief Grid-voltage gain.
 */
#define CTRL_GRID_VOLTAGE_SENSITIVITY (HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Grid-voltage bias.
 */
#define CTRL_GRID_VOLTAGE_BIAS (HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Converter current sensitivity.
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY (0.05f)

/**
 * @brief Converter current bias.
 */
#define CTRL_INVERTER_CURRENT_BIAS (1.65f)

/**
 * @brief Converter voltage gain.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (0.02738589f)

/**
 * @brief Converter voltage bias.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (0.0f)

/**
 * @brief DC current sensitivity.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (0.02475f)

/**
 * @brief DC current bias.
 */
#define CTRL_DC_CURRENT_BIAS (1.65f)

/**
 * @brief DC voltage gain.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (0.02738589f)

/**
 * @brief DC voltage bias.
 */
#define CTRL_DC_VOLTAGE_BIAS (0.0f)

//=================================================================================================
/**
 * @brief Common fallbacks: PGS Grid-Forming Inverter Common Settings.
 */

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
 * @brief Control Algorithm.
 */

/**
 * @brief BUILD_LEVEL 5 grid-forming technique: 1=P-f/Q-V droop, 2=virtual synchronous machine, 3=droop angle source plus virtual impedance.
 *        Options: (1), (2), (3)
 */
#define GFM_CONTROL_TECHNOLOGY (1)

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
 * @brief VSM normalized swing-equation inertia in seconds.
 */
#define GFM_VSM_INERTIA_S (1.0f)

/**
 * @brief VSM damping power coefficient in power PU per hertz.
 */
#define GFM_VSM_DAMPING_PU_PER_HZ (2.0f)

/**
 * @brief VSM reactive-power voltage droop in voltage PU per reactive-power PU.
 */
#define GFM_VSM_Q_DROOP_V_PER_PU (0.05f)

/**
 * @brief VSM active/reactive-power measurement low-pass cutoff.
 */
#define GFM_VSM_POWER_LPF_HZ (10.0f)

/**
 * @brief Virtual resistance used to condition the voltage-loop reference.
 */
#define GFM_VIRTUAL_IMPEDANCE_R_PU (0.03f)

/**
 * @brief Virtual reactance used to condition the voltage-loop reference.
 */
#define GFM_VIRTUAL_IMPEDANCE_X_PU (0.03f)

/**
 * @brief Circular voltage-reference limit after virtual-impedance compensation.
 */
#define GFM_VIRTUAL_IMPEDANCE_VOLTAGE_LIMIT_PU (0.60f)

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

// Common tail code: PGS Grid-Forming Inverter Common Settings
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif
#if defined(USING_3D_SVPWM) && !defined(SPECIFY_PC_ENVIRONMENT) && !defined(GFM_3D_SVPWM_PLATFORM_MAPPED)
#error Define_GFM_3D_SVPWM_PLATFORM_MAPPED_only_after_mapping_all_four_PWM_legs
#endif

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 5)
#error BUILD_LEVEL_must_be_between_1_and_5
#endif
#if defined(USING_3D_SVPWM) && defined(USING_NPC_MODULATOR)
#error USING_3D_SVPWM_and_USING_NPC_MODULATOR_are_mutually_exclusive
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_INV_GFM_SIMULATE_SETTINGS_H_
