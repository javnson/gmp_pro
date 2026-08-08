/**
 * @file sdpe_pgs_inv_gfl_iris_settings.h
 * @brief SDPE project bindings for PGS GFL Inverter F280039C IRIS Node.
 * @note Validated IRIS platform timing, sensing and resource bindings for the common GFL controller.
 */

#ifndef _PROJECT_SDPE_PGS_INV_GFL_IRIS_SETTINGS_H_
#define _PROJECT_SDPE_PGS_INV_GFL_IRIS_SETTINGS_H_

#include <ctl/hardware_preset/grid_lc_filter/gmp_harmonia_3ph_lc_filter.h>
#include <ctl/hardware_preset/inverter_3ph/gmp_helios_3phganinv_lv.h>
#include <ctl/hardware_preset/mcu_board/iris_f280039c_node.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
// SDPE extension point: add after_extern_open code in the Project Requirement Code page if needed.

// Common prefix code: PGS Grid-Following Inverter Common Settings
/* Platform-independent GFL controller settings. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_INV_GFL_IRIS_SDPE_PROJECT_ID "pgs_inv_gfl_f280039c_iris_node"
#define PGS_INV_GFL_IRIS_SDPE_PROJECT_SUITE "pgs_inv_GFL_inverter"
#define PGS_INV_GFL_IRIS_SDPE_PROJECT_VERSION "1.0.0"
#define PGS_INV_GFL_IRIS_SDPE_PROJECT_UPDATED_AT "2026-08-08"

//=================================================================================================
/**
 * @brief PWM.
 */

/**
 * @brief IRIS Helios gate path uses the validated inverted PWM command polarity.
 */
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC (1)

//=================================================================================================
/**
 * @brief Commissioning.
 */

/**
 * @brief BUILD_LEVEL descriptor: 1=open-loop standalone voltage/PWM sensing check; 2=standalone d-q current loop with internal RG; 3=grid PLL plus positive/negative-sequence current control; 4=level 3 plus decoupling, active damping and lead compensation; 5=grid P/Q outer loop over level 4; 6=standalone LC capacitor-voltage outer loop using ordinary PI, final vector limiting and clamping correction over the current loop. USING_3D_SVPWM additionally requires a mapped fourth neutral-leg PWM.
 *        Options: (1), (2), (3), (4), (5), (6)
 */
#define BUILD_LEVEL (3)

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
 * @brief Number of directly sampled phase voltages.
 *        Options: (2), (3)
 */
#define GFL_VOLTAGE_SAMPLE_PHASE_MODE (3)

//=================================================================================================
/**
 * @brief PWM Channel Mapping.
 */

/**
 * @brief PWM base assigned to inverter phase U.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_U_BASE IRIS_EPWM1_BASE

/**
 * @brief PWM base assigned to inverter phase V.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_V_BASE IRIS_EPWM2_BASE

/**
 * @brief PWM base assigned to inverter phase W.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_W_BASE IRIS_EPWM3_BASE

//=================================================================================================
/**
 * @brief Gate Driver GPIO.
 */

/**
 * @brief Gate-driver enable GPIO assignment.
 *        Options: IRIS_GPIO1, IRIS_GPIO2, IRIS_GPIO3, IRIS_GPIO4, IRIS_GPIO5, IRIS_GPIO6
 */
#define PWM_ENABLE_PORT IRIS_GPIO1

/**
 * @brief Gate-driver reset GPIO assignment.
 *        Options: IRIS_GPIO1, IRIS_GPIO2, IRIS_GPIO3, IRIS_GPIO4, IRIS_GPIO5, IRIS_GPIO6
 */
#define PWM_RESET_PORT IRIS_GPIO3

//=================================================================================================
/**
 * @brief ADC Grid Voltage Sensing.
 */

/**
 * @brief INV_UA ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_UA_RESULT_BASE ADC_CH1_RESULT_BASE

/**
 * @brief INV_UA ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_UA ADC_CH1

/**
 * @brief INV_UB ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_UB_RESULT_BASE ADC_CH2_RESULT_BASE

/**
 * @brief INV_UB ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_UB ADC_CH2

/**
 * @brief INV_UC ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_UC_RESULT_BASE ADC_CH3_RESULT_BASE

/**
 * @brief INV_UC ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_UC ADC_CH3

//=================================================================================================
/**
 * @brief ADC Grid Current Sensing.
 */

/**
 * @brief INV_IA ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_IA_RESULT_BASE ADC_CH4_RESULT_BASE

/**
 * @brief INV_IA ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_IA ADC_CH4

/**
 * @brief INV_IB ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_IB_RESULT_BASE ADC_CH5_RESULT_BASE

/**
 * @brief INV_IB ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_IB ADC_CH5

/**
 * @brief INV_IC ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_IC_RESULT_BASE ADC_CH6_RESULT_BASE

/**
 * @brief INV_IC ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_IC ADC_CH6

//=================================================================================================
/**
 * @brief ADC Inverter Phase Voltage Sensing.
 */

/**
 * @brief INV_UU ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_UU_RESULT_BASE ADC_CH9_RESULT_BASE

/**
 * @brief INV_UU ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_UU ADC_CH9

/**
 * @brief INV_UV ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_UV_RESULT_BASE ADC_CH10_RESULT_BASE

/**
 * @brief INV_UV ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_UV ADC_CH10

/**
 * @brief INV_UW ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_UW_RESULT_BASE ADC_CH11_RESULT_BASE

/**
 * @brief INV_UW ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_UW ADC_CH11

//=================================================================================================
/**
 * @brief ADC Inverter Phase Current Sensing.
 */

/**
 * @brief INV_IU ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_IU_RESULT_BASE ADC_CH4_RESULT_BASE

/**
 * @brief INV_IU ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_IU ADC_CH4

/**
 * @brief INV_IV ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_IV_RESULT_BASE ADC_CH5_RESULT_BASE

/**
 * @brief INV_IV ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_IV ADC_CH5

/**
 * @brief INV_IW ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_IW_RESULT_BASE ADC_CH6_RESULT_BASE

/**
 * @brief INV_IW ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_IW ADC_CH6

//=================================================================================================
/**
 * @brief ADC DC Bus Sensing.
 */

/**
 * @brief INV_VBUS ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_VBUS_RESULT_BASE ADC_CH7_RESULT_BASE

/**
 * @brief INV_VBUS ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_VBUS ADC_CH7

/**
 * @brief INV_IBUS ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_IBUS_RESULT_BASE ADC_CH8_RESULT_BASE

/**
 * @brief INV_IBUS ADC channel assignment.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_IBUS ADC_CH8

//=================================================================================================
/**
 * @brief Status GPIO.
 */

/**
 * @brief System status LED assignment.
 *        Options: IRIS_LED1, IRIS_LED2, LED_R, LED_G
 */
#define SYSTEM_LED IRIS_LED1

/**
 * @brief Controller status LED assignment.
 *        Options: IRIS_LED1, IRIS_LED2, LED_R, LED_G
 */
#define CONTROLLER_LED IRIS_LED2

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Number of samples stored per channel by the four-channel hardware Data Link Scope.
 */
#define GMP_DL_SCOPE_DEPTH (100)

/**
 * @brief DC-link voltage sensing gain.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (0.02738589f)

/**
 * @brief DC-link voltage sensing bias.
 */
#define CTRL_DC_VOLTAGE_BIAS (0.0f)

/**
 * @brief DC-link current sensitivity in volts per ampere.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (0.02475f)

/**
 * @brief DC-link current zero bias.
 */
#define CTRL_DC_CURRENT_BIAS (1.65f)

/**
 * @brief Helios phase-voltage sensing gain.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (0.02738589f)

/**
 * @brief Helios phase-voltage sensing bias.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (0.0f)

/**
 * @brief Grid-voltage sensing gain.
 */
#define CTRL_GRID_VOLTAGE_SENSITIVITY (HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Grid-voltage sensing bias.
 */
#define CTRL_GRID_VOLTAGE_BIAS (HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Grid-current sensitivity in volts per ampere.
 */
#define CTRL_GRID_CURRENT_SENSITIVITY (HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSITIVITY_MV_A * 0.001f)

/**
 * @brief Grid-current zero bias.
 */
#define CTRL_GRID_CURRENT_BIAS (HARMONIA_3PH_LC_FILTER_PH_CURRENT_ZERO_BIAS_V)

/**
 * @brief Validated Helios phase-current sensitivity in volts per ampere.
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY (0.05f)

/**
 * @brief Helios phase-current zero bias.
 */
#define CTRL_INVERTER_CURRENT_BIAS (1.65f)

/**
 * @brief Current-loop and PWM update frequency in hertz.
 */
#define CONTROLLER_FREQUENCY (20e3)

/**
 * @brief IRIS ePWM compare range at 20 kHz.
 */
#define CTRL_PWM_CMP_MAX (3000 - 1)

/**
 * @brief IRIS ePWM dead-band count.
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief F280039C system clock in hertz.
 */
#define CTRL_SYS_FREQUENCY (120e6)

/**
 * @brief C2000 millisecond tick divider.
 */
#define DSP_C2000_DSP_TIME_DIV (CTRL_SYS_FREQUENCY / 1000 / CTRL_PWM_CMP_MAX / 2)

/**
 * @brief ADC reference voltage.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief DC-bus per-unit voltage base.
 */
#define CTRL_DCBUS_VOLTAGE (80.0f)

/**
 * @brief SVPWM phase-voltage base.
 */
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

/**
 * @brief Phase-current per-unit base in amperes.
 */
#define CTRL_CURRENT_BASE (10.0f)

/**
 * @brief Grid-filter inductance from the selected Harmonia entity.
 */
#define GFL_GRID_FILTER_INDUCTANCE_H (HARMONIA_3PH_LC_FILTER_INDUCTANCE_H)

/**
 * @brief Grid-filter capacitance from the selected Harmonia entity.
 */
#define GFL_GRID_FILTER_CAPACITANCE_F (HARMONIA_3PH_LC_FILTER_CAPACITANCE_F)

/**
 * @brief Startup delay in milliseconds.
 */
#define CTRL_STARTUP_DELAY (100)

//=================================================================================================
/**
 * @brief Common fallbacks: PGS Grid-Following Inverter Common Settings.
 */

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
 * @brief Enable processor-in-the-loop input/output handling.
 */
// #define ENABLE_GMP_DL_PIL_SIM

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
 * @brief ADC offset calibrator filter cutoff frequency.
 */
#define GFL_ADC_CALIBRATOR_FC_HZ (20.0f)

/**
 * @brief ADC offset calibrator second-order filter quality factor.
 */
#define GFL_ADC_CALIBRATOR_Q (0.707f)

/**
 * @brief Minimum CiA402 delay before Operation Enabled.
 */
#define GFL_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief Nominal grid phase-voltage magnitude in controller per unit.
 */
#define GFL_GRID_VOLTAGE_PU (0.33f)

/**
 * @brief P/Q outer-loop execution frequency in hertz.
 */
#define GFL_PQ_LOOP_FREQUENCY_HZ (1000.0f)

/**
 * @brief Current-ISR to P/Q-loop execution divider.
 */
#define GFL_PQ_LOOP_DIVIDER ((uint32_t)(CONTROLLER_FREQUENCY / GFL_PQ_LOOP_FREQUENCY_HZ))

/**
 * @brief Active-power loop proportional gain from P error PU to d-axis current PU.
 */
#define GFL_PQ_ACTIVE_KP (0.75f)

/**
 * @brief Active-power loop integral gain in inverse seconds.
 */
#define GFL_PQ_ACTIVE_KI (0.001f)

/**
 * @brief Reactive-power loop proportional gain from Q error PU to q-axis current PU.
 */
#define GFL_PQ_REACTIVE_KP (0.75f)

/**
 * @brief Reactive-power loop integral gain in inverse seconds.
 */
#define GFL_PQ_REACTIVE_KI (0.001f)

/**
 * @brief Circular magnitude limit applied to the d/q current reference produced by the P/Q loop.
 */
#define GFL_PQ_CURRENT_LIMIT_PU (1.0f)

/**
 * @brief PQ droop frequency and voltage measurement low-pass cutoff.
 */
#define GFL_PQ_DROOP_LPF_HZ (10.0f)

/**
 * @brief Additional active-power command per hertz of frequency deficit.
 */
#define GFL_PQ_DROOP_P_GAIN_PU_PER_HZ (0.10f)

/**
 * @brief Additional reactive-power command per PU of voltage deficit.
 */
#define GFL_PQ_DROOP_Q_GAIN_PU_PER_V_PU (0.50f)

/**
 * @brief Minimum active-power reference after PQ droop.
 */
#define GFL_PQ_DROOP_P_MIN_PU (-0.80f)

/**
 * @brief Maximum active-power reference after PQ droop.
 */
#define GFL_PQ_DROOP_P_MAX_PU (0.80f)

/**
 * @brief Minimum reactive-power reference after PQ droop.
 */
#define GFL_PQ_DROOP_Q_MIN_PU (-0.80f)

/**
 * @brief Maximum reactive-power reference after PQ droop.
 */
#define GFL_PQ_DROOP_Q_MAX_PU (0.80f)

/**
 * @brief BUILD_LEVEL 1 d-axis open-loop voltage command.
 */
#define GFL_OPEN_LOOP_VD_PU (0.6f)

/**
 * @brief BUILD_LEVEL 1 q-axis open-loop voltage command.
 */
#define GFL_OPEN_LOOP_VQ_PU (0.6f)

/**
 * @brief PLL lock-error threshold in controller per unit.
 */
#define CTRL_SPLL_EPSILON ((float2ctrl(0.005)))

/**
 * @brief Default active-power reference. Positive power exports energy to the grid.
 */
#define GFL_ACTIVE_POWER_REF_PU (0.1f)

/**
 * @brief Default reactive-power reference using Q = vq*id - vd*iq.
 */
#define GFL_REACTIVE_POWER_REF_PU (0.0f)

/**
 * @brief BUILD_LEVEL 2 d-axis current command.
 */
#define GFL_CURRENT_LEVEL2_ID_PU (0.1f)

/**
 * @brief BUILD_LEVEL 2 q-axis current command.
 */
#define GFL_CURRENT_LEVEL2_IQ_PU (0.1f)

/**
 * @brief BUILD_LEVEL 3 grid-connected d-axis current command.
 */
#define GFL_CURRENT_LEVEL3_ID_PU (0.1f)

/**
 * @brief BUILD_LEVEL 3 grid-connected q-axis current command.
 */
#define GFL_CURRENT_LEVEL3_IQ_PU (0.0f)

/**
 * @brief BUILD_LEVEL 4 d-axis current command.
 */
#define GFL_CURRENT_LEVEL4_ID_PU (0.6f)

/**
 * @brief BUILD_LEVEL 4 q-axis current command.
 */
#define GFL_CURRENT_LEVEL4_IQ_PU (0.6f)

/**
 * @brief BUILD_LEVEL 6 positive-sequence d-axis capacitor phase-voltage reference in per unit.
 */
#define GFL_STANDALONE_VD_PU (0.50f)

/**
 * @brief BUILD_LEVEL 6 positive-sequence q-axis capacitor phase-voltage reference in per unit.
 */
#define GFL_STANDALONE_VQ_PU (0.0f)

/**
 * @brief BUILD_LEVEL 6 capacitor-voltage outer-loop bandwidth in hertz; keep it well below the inner current-loop bandwidth.
 */
#define GFL_VOLTAGE_LOOP_BW_HZ (100.0f)

/**
 * @brief BUILD_LEVEL 6 ordinary PI zero frequency in hertz.
 */
#define GFL_VOLTAGE_LOOP_ZERO_HZ (20.0f)

/**
 * @brief Circular magnitude limit for the complete BUILD_LEVEL 6 PI plus feed-forward current reference. The final limited output is returned to the ordinary PID integrators by clamping correction.
 */
#define GFL_VOLTAGE_CIRCLE_LIMIT_PU (0.80f)

/**
 * @brief Independent symmetric d/q-axis limit for the complete BUILD_LEVEL 6 PI plus feed-forward current reference.
 */
#define GFL_VOLTAGE_SQUARE_LIMIT_PU (0.80f)

/**
 * @brief Zero-sequence current QPR proportional gain used when 3D-SVPWM is enabled.
 */
#define GFL_ZERO_QPR_KP (0.10f)

/**
 * @brief Zero-sequence current QPR resonant gain used when 3D-SVPWM is enabled.
 */
#define GFL_ZERO_QPR_KR (50.0f)

/**
 * @brief Zero-sequence QPR resonant bandwidth/cutoff frequency in hertz.
 */
#define GFL_ZERO_QPR_CUTOFF_HZ (5.0f)

/**
 * @brief Symmetric zero-axis voltage-command limit for four-wire operation.
 */
#define GFL_ZERO_VOLTAGE_LIMIT_PU (0.20f)

/**
 * @brief Nominal grid frequency in hertz.
 */
#define GFL_GRID_FREQUENCY_HZ (50.0f)

// Common tail code: PGS Grid-Following Inverter Common Settings
/* Accept the historical PIL spelling while new projects use the canonical switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif
/* Four-leg hardware must explicitly acknowledge that A/B/C/N PWM and protection are mapped. */
#if defined(USING_3D_SVPWM) && !defined(SPECIFY_PC_ENVIRONMENT) && !defined(GFL_3D_SVPWM_PLATFORM_MAPPED)
#error Define_GFL_3D_SVPWM_PLATFORM_MAPPED_only_after_mapping_all_four_PWM_legs
#endif

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 6)
#error BUILD_LEVEL_must_be_between_1_and_6
#endif
#if defined(USING_3D_SVPWM) && defined(USING_NPC_MODULATOR)
#error USING_3D_SVPWM_and_USING_NPC_MODULATOR_are_mutually_exclusive
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_INV_GFL_IRIS_SETTINGS_H_
