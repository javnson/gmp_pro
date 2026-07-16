/**
 * @file sdpe_dps_fsbb_iris_settings.h
 * @brief SDPE project bindings for DPS FSBB F280039C IRIS Node.
 * @note Four-switch buck-boost converter SDPE project requirement prepared from ctl/suite/dps_fsbb/project/f280039c_Iris_node/xplt/ctrl_settings.h.
 *       The requirement introduces the GMP LVFB 150V 2-phase board as the switching stage and sensor source, and IRIS F280039C Node as the peripheral option provider.
 */

#ifndef _PROJECT_SDPE_DPS_FSBB_IRIS_SETTINGS_H_
#define _PROJECT_SDPE_DPS_FSBB_IRIS_SETTINGS_H_

#include <ctl/hardware_preset/mcu_board/iris_f280039c_node.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
#include <sdpe_dps_fsbb_common_settings.h>
/* Project-specific includes can be added here if the migrated project needs extra headers. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define DPS_FSBB_IRIS_SDPE_PROJECT_ID "dps_fsbb_f280039c_Iris_node"
#define DPS_FSBB_IRIS_SDPE_PROJECT_SUITE "dps_fsbb"
#define DPS_FSBB_IRIS_SDPE_PROJECT_VERSION "0.1.0"
#define DPS_FSBB_IRIS_SDPE_PROJECT_UPDATED_AT "2026-07-16"

//=================================================================================================
/**
 * @brief Controller Settings.
 */

/**
 * @brief Enable ADC calibration before normal control starts.
 */
#define SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Enable PIL simulation mode. This mode disables direct controller output for safe communication-based simulation.
 */
// #define ENABLE_GMP_DL_PIL_SIM

/**
 * @brief Enable GMP framework debug information.
 */
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

/**
 * @brief Invert duty-to-CMP mapping because the IRIS ePWM action qualifier sets high on up-count CMPA and low on down-count CMPA.
 */
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC 1

/**
 * @brief This macro will disable the fault request of the controller. That's dangerous.
 */
#define DISABLE_FSBB_PROTECTION_FAULT_LOGIC

//=================================================================================================
/**
 * @brief Optional Sampling Channels.
 */

/**
 * @brief Enable output/load current sampling path.
 */
// #define FSBB_ENABLE_IOUT_SAMPLE

/**
 * @brief Enable input voltage sampling path.
 */
// #define FSBB_ENABLE_VIN_SAMPLE

//=================================================================================================
/**
 * @brief Controller Options.
 */

/**
 * @brief Incremental debug build level. 1: modulation and hardware check; 2: current loop; 3: voltage loop.
 *        Options: (1), (2), (3)
 */
#define BUILD_LEVEL (1)

//=================================================================================================
/**
 * @brief Board GPIO.
 */

/**
 * @brief Gate-driver enable GPIO.
 *        Options: IRIS_GPIO1, IRIS_GPIO2, IRIS_GPIO3, IRIS_GPIO4, IRIS_GPIO5, IRIS_GPIO6
 */
#define PWM_ENABLE_PORT IRIS_GPIO1

/**
 * @brief Gate-driver reset GPIO.
 *        Options: IRIS_GPIO1, IRIS_GPIO2, IRIS_GPIO3, IRIS_GPIO4, IRIS_GPIO5, IRIS_GPIO6
 */
#define PWM_RESET_PORT IRIS_GPIO3

/**
 * @brief System status LED.
 *        Options: IRIS_LED1, IRIS_LED2, LED_R, LED_G
 */
#define SYSTEM_LED IRIS_LED1

//=================================================================================================
/**
 * @brief PWM Channel.
 */

/**
 * @brief Buck phase ePWM base.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_BUCK_BASE IRIS_EPWM1_BASE

/**
 * @brief Boost phase ePWM base.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_BOOST_BASE IRIS_EPWM2_BASE

//=================================================================================================
/**
 * @brief ADC Channel.
 */

/**
 * @brief Input voltage ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define FSBB_VIN_ADC_BASE ADC_CH1_RESULT_BASE

/**
 * @brief Input voltage ADC channel index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_VIN ADC_CH1

/**
 * @brief Output voltage ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define FSBB_VOUT_ADC_BASE ADC_CH2_RESULT_BASE

/**
 * @brief Output voltage ADC channel index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_VOUT ADC_CH2

/**
 * @brief Inductor current ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define FSBB_IL_ADC_BASE ADC_CH3_RESULT_BASE

/**
 * @brief Inductor current ADC channel index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_IL ADC_CH3

/**
 * @brief Input current ADC result register base, available when input-current sampling is enabled.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define FSBB_IIN_ADC_BASE ADC_CH4_RESULT_BASE

/**
 * @brief Input current ADC channel index, available when input-current sampling is enabled.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_IIN ADC_CH4

/**
 * @brief Output/load current ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define FSBB_IOUT_ADC_BASE ADC_CH5_RESULT_BASE

/**
 * @brief Output/load current ADC channel index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_IOUT ADC_CH5

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief PWM compare maximum value for the configured ePWM time base.
 */
#define CTRL_PWM_CMP_MAX (3000 - 1)

/**
 * @brief PWM deadband compare count.
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief CPU main clock frequency.
 */
#define CTRL_SYS_FREQUENCY (120e6)

/**
 * @brief C2000 system tick divider derived from PWM period.
 */
#define DSP_C2000_DSP_TIME_DIV (CTRL_SYS_FREQUENCY / 1000 / CTRL_PWM_CMP_MAX / 2)

// User project tail code
/* Backward compatibility for the historical misspelled PIL switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif

/* Project-specific compile-time validation. */
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 3)
#error "BUILD_LEVEL must be 1 (open loop), 2 (current loop), or 3 (voltage/current cascade)."
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_DPS_FSBB_IRIS_SETTINGS_H_
