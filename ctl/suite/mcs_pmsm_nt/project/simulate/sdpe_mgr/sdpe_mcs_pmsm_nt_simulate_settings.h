/**
 * @file sdpe_mcs_pmsm_nt_simulate_settings.h
 * @brief SDPE project bindings for MCS PMSM NT Windows/Simulink SIL.
 * @note Windows/Simulink SIL implementation of the common mcs_pmsm_nt controller requirement contract.
 */

#ifndef _PROJECT_SDPE_MCS_PMSM_NT_SIMULATE_SETTINGS_H_
#define _PROJECT_SDPE_MCS_PMSM_NT_SIMULATE_SETTINGS_H_

#include <ctl/hardware_preset/inverter_3ph/ti_boostxl_3phganinv.h>
#include <ctl/hardware_preset/pmsm_motor/tyi_5008_kv335.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
#include <sdpe_mcs_pmsm_nt_common_settings.h>

/* Preserve the validated SIL motor and inverter models while using SDPE as the settings source. */
/* Motor and inverter headers are emitted from the SDPE hardware list. */
#define MOTOR_TYPE TYI_5008_KV335_MOTOR_TYPE
#define MOTOR_PARAM_RS TYI_5008_KV335_RS
#define MOTOR_PARAM_LS TYI_5008_KV335_LD
#define MOTOR_PARAM_LD TYI_5008_KV335_LD
#define MOTOR_PARAM_LQ TYI_5008_KV335_LQ
#define MOTOR_PARAM_FLUX TYI_5008_KV335_FLUX
#define MOTOR_PARAM_POLE_PAIRS TYI_5008_KV335_POLE_PAIRS
#define MOTOR_PARAM_KV TYI_5008_KV335_KV
#define MOTOR_PARAM_EMF TYI_5008_KV335_EMF
#define MOTOR_PARAM_RATED_VOLTAGE TYI_5008_KV335_RATED_VOLTAGE
#define MOTOR_PARAM_NO_LOAD_CURRENT TYI_5008_KV335_NO_LOAD_CURRENT
#define MOTOR_PARAM_RATED_FREQUENCY TYI_5008_KV335_RATED_FREQUENCY
#define MOTOR_PARAM_MAX_SPEED TYI_5008_KV335_MAX_SPEED
#define MOTOR_PARAM_MAX_TORQUE TYI_5008_KV335_MAX_TORQUE
#define MOTOR_PARAM_MAX_DC_VOLTAGE TYI_5008_KV335_MAX_DC_VOLTAGE
#define MOTOR_PARAM_MAX_PH_CURRENT TYI_5008_KV335_MAX_PH_CURRENT

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define MCS_PMSM_NT_SIMULATE_SDPE_PROJECT_ID "mcs_pmsm_nt_simulate"
#define MCS_PMSM_NT_SIMULATE_SDPE_PROJECT_SUITE "mcs_pmsm_nt"
#define MCS_PMSM_NT_SIMULATE_SDPE_PROJECT_VERSION "1.2.0"
#define MCS_PMSM_NT_SIMULATE_SDPE_PROJECT_UPDATED_AT "2026-07-15"

//=================================================================================================
/**
 * @brief ADC.
 */

/**
 * @brief Directly sampled phase-current count.
 *        Options: (2), (3)
 */
#define MC_CURRENT_SAMPLE_PHASE_MODE (3)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Maximum compare count of the platform PWM peripheral at the configured controller switching frequency.
 */
#define CTRL_PWM_CMP_MAX (2500 - 1)

/**
 * @brief Dead-time count interpreted in the selected PWM peripheral clock domain.
 */
#define CTRL_PWM_DEADBAND_CMP (100)

/**
 * @brief Platform CPU or system clock frequency in hertz.
 */
#define CTRL_SYS_FREQUENCY (CONTROLLER_FREQUENCY)

/**
 * @brief ADC reference voltage used by all sensor conversions.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief Configured DC-bus voltage base.
 */
#define CTRL_DCBUS_VOLTAGE (80.0f)

/**
 * @brief Phase-voltage per-unit base derived from the DC-bus base.
 */
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

/**
 * @brief Phase-current per-unit base in amperes.
 */
#define CTRL_CURRENT_BASE (10.0f)

/**
 * @brief Encoder counts per mechanical revolution.
 */
#define CTRL_POS_ENC_FS (16384)

/**
 * @brief Mechanical encoder position bias in per unit.
 */
#define CTRL_POS_ENC_BIAS (0.0999145508f)

/**
 * @brief Mechanical speed and position division factor.
 */
#define CTRL_MECH_DIV (5)

/**
 * @brief Phase-current sensor sensitivity in volts per ampere.
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY (TI_BOOSTXL_3PHGANINV_PH_SHUNT_RESISTANCE_OHM * TI_BOOSTXL_3PHGANINV_PH_CSA_GAIN_V_V)

/**
 * @brief Phase-current sensor zero-current bias in volts.
 */
#define CTRL_INVERTER_CURRENT_BIAS (TI_BOOSTXL_3PHGANINV_PH_CSA_BIAS_V)

/**
 * @brief Phase-voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (TI_BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Phase-voltage sensor bias in volts.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (TI_BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief DC-bus current sensing gain. The selected inverter reports SENSOR_NONE for this path.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (TI_BOOSTXL_3PHGANINV_DCBUS_CURRENT_SENSE_GAIN)

/**
 * @brief DC-bus current sensor bias.
 */
#define CTRL_DC_CURRENT_BIAS (TI_BOOSTXL_3PHGANINV_DCBUS_CURRENT_SENSE_BIAS_V)

/**
 * @brief DC-bus voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (TI_BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_GAIN)

/**
 * @brief DC-bus voltage sensor bias in volts.
 */
#define CTRL_DC_VOLTAGE_BIAS (TI_BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_BIAS_V)

// User project tail code
/* No additional platform-specific tail definitions. */

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_MCS_PMSM_NT_SIMULATE_SETTINGS_H_
