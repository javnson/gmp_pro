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
#include <sdpe_pgs_inv_gfl_common_settings.h>

/* Existing LaunchPad SysConfig resource names assigned to GFL roles. */
#define PHASE_U_BASE EPWM_J4_PHASE_U_BASE
#define PHASE_V_BASE EPWM_J4_PHASE_V_BASE
#define PHASE_W_BASE EPWM_J4_PHASE_W_BASE
#define PWM_ENABLE_PORT ENABLE_GATE
#define PWM_RESET_PORT RESET_GATE
#define INV_VBUS_RESULT_BASE J3_VDC_RESULT_BASE
#define INV_VBUS J3_VDC
#define INV_IBUS_RESULT_BASE J7_VDC_RESULT_BASE
#define INV_IBUS J7_VDC
#define INV_UA_RESULT_BASE J7_VU_RESULT_BASE
#define INV_UA J7_VU
#define INV_UB_RESULT_BASE J7_VV_RESULT_BASE
#define INV_UB J7_VV
#define INV_UC_RESULT_BASE J7_VW_RESULT_BASE
#define INV_UC J7_VW
#define INV_IA_RESULT_BASE J7_IU_RESULT_BASE
#define INV_IA J7_IU
#define INV_IB_RESULT_BASE J7_IV_RESULT_BASE
#define INV_IB J7_IV
#define INV_IC_RESULT_BASE J7_IW_RESULT_BASE
#define INV_IC J7_IW
#define INV_UU_RESULT_BASE J3_VU_RESULT_BASE
#define INV_UU J3_VU
#define INV_UV_RESULT_BASE J3_VV_RESULT_BASE
#define INV_UV J3_VV
#define INV_UW_RESULT_BASE J3_VW_RESULT_BASE
#define INV_UW J3_VW
#define INV_IU_RESULT_BASE J3_IU_RESULT_BASE
#define INV_IU J3_IU
#define INV_IV_RESULT_BASE J3_IV_RESULT_BASE
#define INV_IV J3_IV
#define INV_IW_RESULT_BASE J3_IW_RESULT_BASE
#define INV_IW J3_IW
#define SYSTEM_LED LED_R
#define CONTROLLER_LED LED_G

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_INV_GFL_F280049C_SDPE_PROJECT_ID "pgs_inv_gfl_f280049c"
#define PGS_INV_GFL_F280049C_SDPE_PROJECT_SUITE "pgs_inv_GFL_inverter"
#define PGS_INV_GFL_F280049C_SDPE_PROJECT_VERSION "1.0.0"
#define PGS_INV_GFL_F280049C_SDPE_PROJECT_UPDATED_AT "2026-07-16"

//=================================================================================================
/**
 * @brief Commissioning.
 */

/**
 * @brief Incremental control level; level 5 enables the cascaded P/Q power loop.
 *        Options: (1), (2), (3), (4), (5)
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
 * @brief Requirement bindings.
 */

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

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 5)
#error BUILD_LEVEL_must_be_between_1_and_5
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_INV_GFL_F280049C_SETTINGS_H_
