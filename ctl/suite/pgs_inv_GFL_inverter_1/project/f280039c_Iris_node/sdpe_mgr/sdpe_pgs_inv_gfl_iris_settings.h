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
#include <sdpe_pgs_inv_gfl_common_settings.h>

/* Existing IRIS SysConfig resource names assigned to GFL application roles. */
#define PHASE_U_BASE IRIS_EPWM1_BASE
#define PHASE_V_BASE IRIS_EPWM2_BASE
#define PHASE_W_BASE IRIS_EPWM3_BASE
#define PWM_ENABLE_PORT IRIS_GPIO1
#define PWM_RESET_PORT IRIS_GPIO3
#define INV_UA_RESULT_BASE ADC_CH1_RESULT_BASE
#define INV_UA ADC_CH1
#define INV_UB_RESULT_BASE ADC_CH2_RESULT_BASE
#define INV_UB ADC_CH2
#define INV_UC_RESULT_BASE ADC_CH3_RESULT_BASE
#define INV_UC ADC_CH3
#define INV_IA_RESULT_BASE ADC_CH4_RESULT_BASE
#define INV_IA ADC_CH4
#define INV_IB_RESULT_BASE ADC_CH5_RESULT_BASE
#define INV_IB ADC_CH5
#define INV_IC_RESULT_BASE ADC_CH6_RESULT_BASE
#define INV_IC ADC_CH6
#define INV_UU_RESULT_BASE ADC_CH9_RESULT_BASE
#define INV_UU ADC_CH9
#define INV_UV_RESULT_BASE ADC_CH10_RESULT_BASE
#define INV_UV ADC_CH10
#define INV_UW_RESULT_BASE ADC_CH11_RESULT_BASE
#define INV_UW ADC_CH11
#define INV_IU_RESULT_BASE ADC_CH4_RESULT_BASE
#define INV_IU ADC_CH4
#define INV_IV_RESULT_BASE ADC_CH5_RESULT_BASE
#define INV_IV ADC_CH5
#define INV_IW_RESULT_BASE ADC_CH6_RESULT_BASE
#define INV_IW ADC_CH6
#define INV_VBUS_RESULT_BASE ADC_CH7_RESULT_BASE
#define INV_VBUS ADC_CH7
#define INV_IBUS_RESULT_BASE ADC_CH8_RESULT_BASE
#define INV_IBUS ADC_CH8
#define SYSTEM_LED IRIS_LED1
#define CONTROLLER_LED IRIS_LED2

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_INV_GFL_IRIS_SDPE_PROJECT_ID "pgs_inv_gfl_f280039c_iris_node"
#define PGS_INV_GFL_IRIS_SDPE_PROJECT_SUITE "pgs_inv_GFL_inverter"
#define PGS_INV_GFL_IRIS_SDPE_PROJECT_VERSION "1.0.0"
#define PGS_INV_GFL_IRIS_SDPE_PROJECT_UPDATED_AT "2026-07-15"

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
 * @brief Incremental control level; level 5 enables the cascaded P/Q power loop.
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
 * @brief Requirement bindings.
 */

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
#define CTRL_INVERTER_CURRENT_SENSITIVITY (0.05f)

/**
 * @brief Helios phase-current zero bias.
 */
#define CTRL_INVERTER_CURRENT_BIAS (1.65f)

/**
 * @brief Helios phase-voltage sensing gain.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (0.02738589f)

/**
 * @brief Helios phase-voltage sensing bias.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (0.0f)

/**
 * @brief DC-link current sensitivity in volts per ampere.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (0.02475f)

/**
 * @brief DC-link current zero bias.
 */
#define CTRL_DC_CURRENT_BIAS (1.65f)

/**
 * @brief DC-link voltage sensing gain.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (0.02738589f)

/**
 * @brief DC-link voltage sensing bias.
 */
#define CTRL_DC_VOLTAGE_BIAS (0.0f)

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 5)
#error BUILD_LEVEL_must_be_between_1_and_5
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_INV_GFL_IRIS_SETTINGS_H_
