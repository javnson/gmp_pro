/**
 * @file sdpe_pgs_inv_gfm_iris_settings.h
 * @brief SDPE project bindings for PGS GFM Inverter F280039C IRIS Node.
 * @note IRIS platform timing, sensing and resource bindings for the common GFM controller.
 */

#ifndef _PROJECT_SDPE_PGS_INV_GFM_IRIS_SETTINGS_H_
#define _PROJECT_SDPE_PGS_INV_GFM_IRIS_SETTINGS_H_

#include <ctl/hardware_preset/grid_lc_filter/gmp_harmonia_3ph_lc_filter.h>
#include <ctl/hardware_preset/inverter_3ph/gmp_helios_3phganinv_lv.h>
#include <ctl/hardware_preset/mcu_board/iris_f280039c_node.h>

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

#define PGS_INV_GFM_IRIS_SDPE_PROJECT_ID "pgs_inv_gfm_f280039c_iris_node"
#define PGS_INV_GFM_IRIS_SDPE_PROJECT_SUITE "pgs_inv_GFM_inverter"
#define PGS_INV_GFM_IRIS_SDPE_PROJECT_VERSION "1.0.0"
#define PGS_INV_GFM_IRIS_SDPE_PROJECT_UPDATED_AT "2026-08-08"

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
 * @brief BUILD_LEVEL descriptor: 1=open-loop voltage; 2=current loop with internal RG; 3=stand-alone LC capacitor-voltage loop; 4=PLL-oriented grid current loop; 5=PLL synchronization followed by bumpless transfer to the SDPE-selected droop, VSM, or virtual-impedance GFM outer loop plus voltage loop. USING_3D_SVPWM additionally requires a mapped fourth neutral-leg PWM.
 *        Options: (1), (2), (3), (4), (5)
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
 * @brief Startup delay in milliseconds.
 */
#define CTRL_STARTUP_DELAY (100)

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
#define CTRL_ADC_VOLTAGE_REF real2param(3.3)

/**
 * @brief DC-bus per-unit voltage base.
 */
#define CTRL_DCBUS_VOLTAGE real2param(80.0)

/**
 * @brief SVPWM phase-voltage base.
 */
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

/**
 * @brief Phase-current per-unit base in amperes.
 */
#define CTRL_CURRENT_BASE real2param(10.0)

/**
 * @brief Grid-filter inductance from the selected Harmonia entity.
 */
#define GFM_GRID_FILTER_INDUCTANCE_H (HARMONIA_3PH_LC_FILTER_INDUCTANCE_H)

/**
 * @brief Grid-filter capacitance from the selected Harmonia entity.
 */
#define GFM_GRID_FILTER_CAPACITANCE_F (HARMONIA_3PH_LC_FILTER_CAPACITANCE_F)

/**
 * @brief Grid-current sensitivity in volts per ampere.
 */
#define CTRL_GRID_CURRENT_SENSITIVITY (HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSITIVITY_MV_A * 0.001f)

/**
 * @brief Grid-current zero bias.
 */
#define CTRL_GRID_CURRENT_BIAS (HARMONIA_3PH_LC_FILTER_PH_CURRENT_ZERO_BIAS_V)

/**
 * @brief Grid-voltage sensing gain.
 */
#define CTRL_GRID_VOLTAGE_SENSITIVITY (HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Grid-voltage sensing bias.
 */
#define CTRL_GRID_VOLTAGE_BIAS (HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Validated Helios phase-current sensitivity in volts per ampere.
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY real2param(0.05)

/**
 * @brief Helios phase-current zero bias.
 */
#define CTRL_INVERTER_CURRENT_BIAS real2param(1.65)

/**
 * @brief Helios phase-voltage sensing gain.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY real2param(0.02738589)

/**
 * @brief Helios phase-voltage sensing bias.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS real2param(0.0)

/**
 * @brief DC-link current sensitivity in volts per ampere.
 */
#define CTRL_DC_CURRENT_SENSITIVITY real2param(0.02475)

/**
 * @brief DC-link current zero bias.
 */
#define CTRL_DC_CURRENT_BIAS real2param(1.65)

/**
 * @brief DC-link voltage sensing gain.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY real2param(0.02738589)

/**
 * @brief DC-link voltage sensing bias.
 */
#define CTRL_DC_VOLTAGE_BIAS real2param(0.0)

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
 * @brief Enable the seven ADC slots used by the three-phase inverter SIL input ABI.
 */
#ifndef GMP_PIL_RX_MASK
#define GMP_PIL_RX_MASK (127)
#endif // GMP_PIL_RX_MASK

/**
 * @brief Enable four PWM slots and all sixteen monitor slots used by the GFM SIL output ABI.
 */
#ifndef GMP_PIL_TX_MASK
#define GMP_PIL_TX_MASK (4294901775)
#endif // GMP_PIL_TX_MASK

/**
 * @brief Nominal grid phase-voltage magnitude in controller per unit.
 */
#define GFM_GRID_VOLTAGE_PU real2param(0.33)

/**
 * @brief Nominal grid-forming and synchronization frequency.
 */
#define GFM_GRID_FREQUENCY_HZ real2param(50.0)

/**
 * @brief BUILD_LEVEL 1 d-axis open-loop voltage command.
 */
#define GFM_OPEN_LOOP_VD_PU real2param(0.50)

/**
 * @brief BUILD_LEVEL 1 q-axis open-loop voltage command.
 */
#define GFM_OPEN_LOOP_VQ_PU real2param(0.0)

/**
 * @brief BUILD_LEVEL 2 d-axis current command using the internal ramp angle.
 */
#define GFM_CURRENT_LEVEL2_ID_PU real2param(0.10)

/**
 * @brief BUILD_LEVEL 2 q-axis current command using the internal ramp angle.
 */
#define GFM_CURRENT_LEVEL2_IQ_PU real2param(0.10)

/**
 * @brief BUILD_LEVEL 4 PLL-oriented d-axis grid current command.
 */
#define GFM_CURRENT_LEVEL4_ID_PU real2param(0.10)

/**
 * @brief BUILD_LEVEL 4 PLL-oriented q-axis grid current command.
 */
#define GFM_CURRENT_LEVEL4_IQ_PU real2param(0.0)

/**
 * @brief BUILD_LEVEL 3 fixed voltage reference and BUILD_LEVEL 5 nominal droop voltage.
 */
#define GFM_VOLTAGE_VD_PU real2param(0.50)

/**
 * @brief BUILD_LEVEL 3 fixed q-axis voltage reference.
 */
#define GFM_VOLTAGE_VQ_PU real2param(0.0)

/**
 * @brief LC capacitor-voltage loop bandwidth.
 */
#define GFM_VOLTAGE_LOOP_BW_HZ real2param(100.0)

/**
 * @brief LC capacitor-voltage ordinary PI zero frequency.
 */
#define GFM_VOLTAGE_LOOP_ZERO_HZ real2param(20.0)

/**
 * @brief Circular magnitude limit of the complete voltage-loop current reference.
 */
#define GFM_VOLTAGE_CIRCLE_LIMIT_PU real2param(0.80)

/**
 * @brief Independent symmetric d/q-axis voltage-loop current-reference limit.
 */
#define GFM_VOLTAGE_SQUARE_LIMIT_PU real2param(0.80)

/**
 * @brief Active-power to frequency droop slope in Hz per power PU.
 */
#define GFM_DROOP_P_HZ_PER_PU real2param(0.50)

/**
 * @brief Reactive-power to voltage droop slope in voltage PU per reactive-power PU.
 */
#define GFM_DROOP_Q_V_PER_PU real2param(0.05)

/**
 * @brief Droop active/reactive-power measurement low-pass cutoff.
 */
#define GFM_DROOP_POWER_LPF_HZ real2param(10.0)

/**
 * @brief Maximum absolute droop frequency deviation from nominal.
 */
#define GFM_DROOP_FREQUENCY_DELTA_LIMIT_HZ real2param(2.0)

/**
 * @brief Minimum voltage magnitude requested by the droop module.
 */
#define GFM_DROOP_VOLTAGE_MIN_PU real2param(0.40)

/**
 * @brief Maximum voltage magnitude requested by the droop module.
 */
#define GFM_DROOP_VOLTAGE_MAX_PU real2param(0.60)

/**
 * @brief Default active-power reference for the droop algorithm.
 */
#define GFM_DROOP_ACTIVE_POWER_REF_PU real2param(0.0)

/**
 * @brief Default reactive-power reference for the droop algorithm.
 */
#define GFM_DROOP_REACTIVE_POWER_REF_PU real2param(0.0)

/**
 * @brief VSM normalized swing-equation inertia in seconds.
 */
#define GFM_VSM_INERTIA_S real2param(1.0)

/**
 * @brief VSM damping power coefficient in power PU per hertz.
 */
#define GFM_VSM_DAMPING_PU_PER_HZ real2param(2.0)

/**
 * @brief VSM reactive-power voltage droop in voltage PU per reactive-power PU.
 */
#define GFM_VSM_Q_DROOP_V_PER_PU real2param(0.05)

/**
 * @brief VSM active/reactive-power measurement low-pass cutoff.
 */
#define GFM_VSM_POWER_LPF_HZ real2param(10.0)

/**
 * @brief Virtual resistance used to condition the voltage-loop reference.
 */
#define GFM_VIRTUAL_IMPEDANCE_R_PU real2param(0.03)

/**
 * @brief Virtual reactance used to condition the voltage-loop reference.
 */
#define GFM_VIRTUAL_IMPEDANCE_X_PU real2param(0.03)

/**
 * @brief Circular voltage-reference limit after virtual-impedance compensation.
 */
#define GFM_VIRTUAL_IMPEDANCE_VOLTAGE_LIMIT_PU real2param(0.60)

/**
 * @brief PLL-to-grid-forming phasor and current-command blend duration.
 */
#define GFM_TRANSITION_TIME_S real2param(0.10)

/**
 * @brief Continuous PLL-lock duration required before requesting grid-forming takeover.
 */
#define GFM_SYNC_HOLD_TIME_S real2param(0.20)

/**
 * @brief Maximum instantaneous PLL q-axis error during the continuous grid-forming synchronization hold.
 */
#define GFM_SYNC_PLL_ERROR_PU real2param(0.08)

/**
 * @brief Tracking-mode d-axis current command before grid-forming takeover.
 */
#define GFM_SYNC_ID_PU real2param(0.0)

/**
 * @brief Tracking-mode q-axis current command before grid-forming takeover.
 */
#define GFM_SYNC_IQ_PU real2param(0.0)

/**
 * @brief Zero-sequence current QPR proportional gain for four-wire operation.
 */
#define GFM_ZERO_QPR_KP real2param(0.10)

/**
 * @brief Zero-sequence current QPR resonant gain.
 */
#define GFM_ZERO_QPR_KR real2param(50.0)

/**
 * @brief Zero-sequence QPR resonant bandwidth.
 */
#define GFM_ZERO_QPR_CUTOFF_HZ real2param(5.0)

/**
 * @brief Symmetric zero-axis voltage-command limit.
 */
#define GFM_ZERO_VOLTAGE_LIMIT_PU real2param(0.20)

/**
 * @brief ADC offset calibrator cutoff.
 */
#define GFM_ADC_CALIBRATOR_FC_HZ real2param(20.0)

/**
 * @brief ADC offset calibrator quality factor.
 */
#define GFM_ADC_CALIBRATOR_Q real2param(0.707)

/**
 * @brief Minimum CiA402 delay before Operation Enabled.
 */
#define GFM_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief PLL lock-error threshold used before grid-forming takeover.
 */
#define CTRL_SPLL_EPSILON real2ctrl(0.005)

//=================================================================================================
/**
 * @brief Common fallbacks: GMP Suite PIL Common Transport.
 */

//=================================================================================================
/**
 * @brief PIL Runtime.
 */

/**
 * @brief Run control steps only from Data Link PIL transactions while physical control dispatch and power-stage enable remain isolated.
 */
// #ifndef ENABLE_GMP_DL_PIL_SIM
// #define ENABLE_GMP_DL_PIL_SIM
// #endif // ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Base command followed by mask, step, status, and abort subcommands.
 */
#ifndef GMP_PIL_DL_BASE_COMMAND
#define GMP_PIL_DL_BASE_COMMAND (16)
#endif // GMP_PIL_DL_BASE_COMMAND

/**
 * @brief Portable commissioning rate used unless a hardware project overrides it.
 */
#ifndef GMP_DL_UART_BAUDRATE
#define GMP_DL_UART_BAUDRATE (115200)
#endif // GMP_DL_UART_BAUDRATE

/**
 * @brief Host running Simulink and the GMP PIL bridge.
 */
#ifndef GMP_PIL_UDP_HOST
#define GMP_PIL_UDP_HOST "127.0.0.1"
#endif // GMP_PIL_UDP_HOST

/**
 * @brief Bridge port receiving plant samples from Simulink.
 */
#ifndef GMP_PIL_BRIDGE_UDP_LISTEN_PORT
#define GMP_PIL_BRIDGE_UDP_LISTEN_PORT (12501)
#endif // GMP_PIL_BRIDGE_UDP_LISTEN_PORT

/**
 * @brief Simulink port receiving controller results from the bridge.
 */
#ifndef GMP_PIL_MATLAB_UDP_LISTEN_PORT
#define GMP_PIL_MATLAB_UDP_LISTEN_PORT (12500)
#endif // GMP_PIL_MATLAB_UDP_LISTEN_PORT

/**
 * @brief Bridge port receiving out-of-band Simulink commands.
 */
#ifndef GMP_PIL_MATLAB_COMMAND_TX_PORT
#define GMP_PIL_MATLAB_COMMAND_TX_PORT (12502)
#endif // GMP_PIL_MATLAB_COMMAND_TX_PORT

/**
 * @brief Simulink port receiving command acknowledgements.
 */
#ifndef GMP_PIL_MATLAB_COMMAND_RX_PORT
#define GMP_PIL_MATLAB_COMMAND_RX_PORT (12503)
#endif // GMP_PIL_MATLAB_COMMAND_RX_PORT

/**
 * @brief Maximum wait for one target Data Link response.
 */
#ifndef GMP_PIL_MCU_TIMEOUT_MS
#define GMP_PIL_MCU_TIMEOUT_MS (200)
#endif // GMP_PIL_MCU_TIMEOUT_MS

/**
 * @brief Maximum wait for the next Simulink plant sample.
 */
#ifndef GMP_PIL_MATLAB_TIMEOUT_MS
#define GMP_PIL_MATLAB_TIMEOUT_MS (5000)
#endif // GMP_PIL_MATLAB_TIMEOUT_MS

/**
 * @brief Digital input slot packed into the standard Data Link PIL request.
 */
#ifndef GMP_PIL_UDP_DIGITAL_INDEX
#define GMP_PIL_UDP_DIGITAL_INDEX (0)
#endif // GMP_PIL_UDP_DIGITAL_INDEX

// Common tail code: PGS Grid-Forming Inverter Common Settings
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif
#if defined(USING_3D_SVPWM) && !defined(SPECIFY_PC_ENVIRONMENT) && !defined(GFM_3D_SVPWM_PLATFORM_MAPPED)
#error Define_GFM_3D_SVPWM_PLATFORM_MAPPED_only_after_mapping_all_four_PWM_legs
#endif

// Common tail code: GMP Suite PIL Common Transport
/** Validate the shared PIL command allocation. */
#if (GMP_PIL_DL_BASE_COMMAND > 251U)
#error "GMP_PIL_DL_BASE_COMMAND must leave room for four PIL subcommands."
#endif
#if (GMP_PIL_UDP_DIGITAL_INDEX >= 8U)
#error "GMP_PIL_UDP_DIGITAL_INDEX must be in the range [0, 7]."
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

#endif // _PROJECT_SDPE_PGS_INV_GFM_IRIS_SETTINGS_H_
