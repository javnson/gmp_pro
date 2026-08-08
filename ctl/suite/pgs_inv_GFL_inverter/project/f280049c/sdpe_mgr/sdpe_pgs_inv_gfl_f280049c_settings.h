/**
 * @file sdpe_pgs_inv_gfl_f280049c_settings.h
 * @brief SDPE project bindings for PGS GFL Inverter LaunchXL-F280049C.
 * @note Validated LaunchXL-F280049C timing, sensing and BOOSTXL resource bindings for the common GFL controller.
 */

#ifndef _PROJECT_SDPE_PGS_INV_GFL_F280049C_SETTINGS_H_
#define _PROJECT_SDPE_PGS_INV_GFL_F280049C_SETTINGS_H_

#include <ctl/hardware_preset/grid_lc_filter/gmp_harmonia_3ph_lc_filter.h>
#include <ctl/hardware_preset/inverter_3ph/gmp_helios_3phganinv_lv.h>
#include <ctl/hardware_preset/mcu_board/launchxl_f280049c.h>

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

#define PGS_INV_GFL_F280049C_SDPE_PROJECT_ID "pgs_inv_gfl_f280049c"
#define PGS_INV_GFL_F280049C_SDPE_PROJECT_SUITE "pgs_inv_GFL_inverter"
#define PGS_INV_GFL_F280049C_SDPE_PROJECT_VERSION "1.0.0"
#define PGS_INV_GFL_F280049C_SDPE_PROJECT_UPDATED_AT "2026-08-08"

//=================================================================================================
/**
 * @brief Commissioning.
 */

/**
 * @brief BUILD_LEVEL descriptor: 1=open-loop standalone voltage/PWM sensing check; 2=standalone d-q current loop with internal RG; 3=grid PLL plus positive/negative-sequence current control; 4=level 3 plus decoupling, active damping and lead compensation; 5=grid P/Q outer loop over level 4; 6=standalone LC capacitor-voltage outer loop using ordinary PI, final vector limiting and clamping correction over the current loop. USING_3D_SVPWM additionally requires a mapped fourth neutral-leg PWM.
 *        Options: (1), (2), (3), (4), (5), (6)
 */
#define BUILD_LEVEL (1)

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
 *        Options: EPWM_J4_PHASE_U_BASE, EPWM_J4_PHASE_V_BASE, EPWM_J4_PHASE_W_BASE
 */
#define PHASE_U_BASE EPWM_J4_PHASE_U_BASE

/**
 * @brief PWM base assigned to inverter phase V.
 *        Options: EPWM_J4_PHASE_U_BASE, EPWM_J4_PHASE_V_BASE, EPWM_J4_PHASE_W_BASE
 */
#define PHASE_V_BASE EPWM_J4_PHASE_V_BASE

/**
 * @brief PWM base assigned to inverter phase W.
 *        Options: EPWM_J4_PHASE_U_BASE, EPWM_J4_PHASE_V_BASE, EPWM_J4_PHASE_W_BASE
 */
#define PHASE_W_BASE EPWM_J4_PHASE_W_BASE

//=================================================================================================
/**
 * @brief Gate Driver GPIO.
 */

/**
 * @brief Gate-driver enable GPIO assignment.
 *        Options: ENABLE_GATE, RESET_GATE
 */
#define PWM_ENABLE_PORT ENABLE_GATE

/**
 * @brief Gate-driver reset GPIO assignment.
 *        Options: ENABLE_GATE, RESET_GATE
 */
#define PWM_RESET_PORT RESET_GATE

//=================================================================================================
/**
 * @brief ADC DC Bus Sensing.
 */

/**
 * @brief INV_VBUS ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_VBUS_RESULT_BASE J3_VDC_RESULT_BASE

/**
 * @brief INV_VBUS ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_VBUS J3_VDC

/**
 * @brief INV_IBUS ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_IBUS_RESULT_BASE J7_VDC_RESULT_BASE

/**
 * @brief INV_IBUS ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_IBUS J7_VDC

//=================================================================================================
/**
 * @brief ADC Grid Voltage Sensing.
 */

/**
 * @brief INV_UA ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_UA_RESULT_BASE J7_VU_RESULT_BASE

/**
 * @brief INV_UA ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_UA J7_VU

/**
 * @brief INV_UB ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_UB_RESULT_BASE J7_VV_RESULT_BASE

/**
 * @brief INV_UB ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_UB J7_VV

/**
 * @brief INV_UC ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_UC_RESULT_BASE J7_VW_RESULT_BASE

/**
 * @brief INV_UC ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_UC J7_VW

//=================================================================================================
/**
 * @brief ADC Grid Current Sensing.
 */

/**
 * @brief INV_IA ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_IA_RESULT_BASE J7_IU_RESULT_BASE

/**
 * @brief INV_IA ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_IA J7_IU

/**
 * @brief INV_IB ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_IB_RESULT_BASE J7_IV_RESULT_BASE

/**
 * @brief INV_IB ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_IB J7_IV

/**
 * @brief INV_IC ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_IC_RESULT_BASE J7_IW_RESULT_BASE

/**
 * @brief INV_IC ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_IC J7_IW

//=================================================================================================
/**
 * @brief ADC Inverter Phase Voltage Sensing.
 */

/**
 * @brief INV_UU ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_UU_RESULT_BASE J3_VU_RESULT_BASE

/**
 * @brief INV_UU ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_UU J3_VU

/**
 * @brief INV_UV ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_UV_RESULT_BASE J3_VV_RESULT_BASE

/**
 * @brief INV_UV ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_UV J3_VV

/**
 * @brief INV_UW ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_UW_RESULT_BASE J3_VW_RESULT_BASE

/**
 * @brief INV_UW ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_UW J3_VW

//=================================================================================================
/**
 * @brief ADC Inverter Phase Current Sensing.
 */

/**
 * @brief INV_IU ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_IU_RESULT_BASE J3_IU_RESULT_BASE

/**
 * @brief INV_IU ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_IU J3_IU

/**
 * @brief INV_IV ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_IV_RESULT_BASE J3_IV_RESULT_BASE

/**
 * @brief INV_IV ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_IV J3_IV

/**
 * @brief INV_IW ADC result register base.
 *        Options: J3_VDC_RESULT_BASE, J7_VDC_RESULT_BASE, J7_VU_RESULT_BASE, J7_VV_RESULT_BASE, J7_VW_RESULT_BASE, J7_IU_RESULT_BASE, J7_IV_RESULT_BASE, J7_IW_RESULT_BASE, J3_VU_RESULT_BASE, J3_VV_RESULT_BASE, J3_VW_RESULT_BASE, J3_IU_RESULT_BASE, J3_IV_RESULT_BASE, J3_IW_RESULT_BASE
 */
#define INV_IW_RESULT_BASE J3_IW_RESULT_BASE

/**
 * @brief INV_IW ADC channel assignment.
 *        Options: J3_VDC, J7_VDC, J7_VU, J7_VV, J7_VW, J7_IU, J7_IV, J7_IW, J3_VU, J3_VV, J3_VW, J3_IU, J3_IV, J3_IW
 */
#define INV_IW J3_IW

//=================================================================================================
/**
 * @brief Status GPIO.
 */

/**
 * @brief System status LED assignment.
 *        Options: LED_R, LED_G
 */
#define SYSTEM_LED LED_R

/**
 * @brief Controller status LED assignment.
 *        Options: LED_R, LED_G
 */
#define CONTROLLER_LED LED_G

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Number of samples stored per channel by the four-channel hardware Data Link Scope.
 */
#define GMP_DL_SCOPE_DEPTH (100)

/**
 * @brief Current-loop and PWM update frequency in hertz.
 */
#define CONTROLLER_FREQUENCY (10e3)

/**
 * @brief Validated F280049C ePWM compare range.
 */
#define CTRL_PWM_CMP_MAX (6000)

/**
 * @brief ePWM dead-band count.
 */
#define CTRL_PWM_DEADBAND_CMP (100)

/**
 * @brief F280049C system clock in hertz.
 */
#define CTRL_SYS_FREQUENCY (100e6)

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
 * @brief Harmonia grid-filter inductance.
 */
#define GFL_GRID_FILTER_INDUCTANCE_H (HARMONIA_3PH_LC_FILTER_INDUCTANCE_H)

/**
 * @brief Harmonia grid-filter capacitance.
 */
#define GFL_GRID_FILTER_CAPACITANCE_F (HARMONIA_3PH_LC_FILTER_CAPACITANCE_F)

/**
 * @brief DC-link voltage sensing gain.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (0.02738589f)

/**
 * @brief DC-link voltage sensing bias.
 */
#define CTRL_DC_VOLTAGE_BIAS (0.0f)

/**
 * @brief Grid-voltage sensing gain.
 */
#define CTRL_GRID_VOLTAGE_SENSITIVITY (HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Grid-voltage sensing bias.
 */
#define CTRL_GRID_VOLTAGE_BIAS (HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Helios voltage sensing gain.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (0.02738589f)

/**
 * @brief Helios voltage sensing bias.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (0.0f)

/**
 * @brief DC-link current sensitivity.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (0.02475f)

/**
 * @brief DC-link current bias.
 */
#define CTRL_DC_CURRENT_BIAS (1.65f)

/**
 * @brief Grid-current sensitivity.
 */
#define CTRL_GRID_CURRENT_SENSITIVITY (HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSITIVITY_MV_A * 0.001f)

/**
 * @brief Grid-current bias.
 */
#define CTRL_GRID_CURRENT_BIAS (HARMONIA_3PH_LC_FILTER_PH_CURRENT_ZERO_BIAS_V)

/**
 * @brief Validated Helios current sensitivity.
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY (0.05f)

/**
 * @brief Helios current bias.
 */
#define CTRL_INVERTER_CURRENT_BIAS (1.65f)

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

#endif // _PROJECT_SDPE_PGS_INV_GFL_F280049C_SETTINGS_H_
