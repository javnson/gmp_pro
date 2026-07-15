/**
 * @file sdpe_mcs_pmsm_nt_iris_settings.h
 * @brief SDPE project bindings for MCS PMSM NT F280039C IRIS Node.
 * @note F280039C IRIS hardware timing, sensing and peripheral bindings. Platform-independent controller settings are supplied by src/sdpe_mcs_pmsm_nt_common_settings.h.
 */

#ifndef _PROJECT_SDPE_MCS_PMSM_NT_IRIS_SETTINGS_H_
#define _PROJECT_SDPE_MCS_PMSM_NT_IRIS_SETTINGS_H_

#include <ctl/hardware_preset/inverter_3ph/gmp_3ph_2136sinv_dual.h>
#include <ctl/hardware_preset/mcu_board/iris_f280039c_node.h>
#include <ctl/hardware_preset/pmsm_motor/sm060r20b30mnad.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
#include <sdpe_mcs_pmsm_nt_common_settings.h>

/* The inverter is selected as an SDPE hardware entity. */

/* Compatibility names consumed by the existing PMSM controller. */
#define MOTOR_TYPE                     SM060R20B30MNAD_MOTOR_TYPE
#define MOTOR_PARAM_RS                 SM060R20B30MNAD_RS
#define MOTOR_PARAM_LS                 SM060R20B30MNAD_LD
#define MOTOR_PARAM_LD                 SM060R20B30MNAD_LD
#define MOTOR_PARAM_LQ                 SM060R20B30MNAD_LQ
#define MOTOR_PARAM_FLUX               SM060R20B30MNAD_FLUX
#define MOTOR_PARAM_POLE_PAIRS         SM060R20B30MNAD_POLE_PAIRS
#define MOTOR_PARAM_INERTIA            SM060R20B30MNAD_INERTIA
#define MOTOR_PARAM_FRICTION           SM060R20B30MNAD_FRICTION
#define MOTOR_PARAM_KV                 SM060R20B30MNAD_KV
#define MOTOR_PARAM_EMF                SM060R20B30MNAD_EMF
#define MOTOR_PARAM_RATED_VOLTAGE      SM060R20B30MNAD_RATED_VOLTAGE
#define MOTOR_PARAM_RATED_CURRENT      SM060R20B30MNAD_RATED_CURRENT
#define MOTOR_PARAM_NO_LOAD_CURRENT    SM060R20B30MNAD_NO_LOAD_CURRENT
#define MOTOR_PARAM_RATED_FREQUENCY    SM060R20B30MNAD_RATED_FREQUENCY
#define MOTOR_PARAM_MAX_SPEED          SM060R20B30MNAD_MAX_SPEED
#define MOTOR_PARAM_MAX_TORQUE         SM060R20B30MNAD_MAX_TORQUE
#define MOTOR_PARAM_MAX_DC_VOLTAGE     SM060R20B30MNAD_MAX_DC_VOLTAGE
#define MOTOR_PARAM_MAX_PH_CURRENT     SM060R20B30MNAD_MAX_PH_CURRENT

/* Compatibility identifiers for code that still checks the fixed board target. */
#define LAUNCHPAD 0
#define GMP_IRIS  1
#define BOARD_SELECTION GMP_IRIS

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define MCS_PMSM_NT_IRIS_SDPE_PROJECT_ID "mcs_pmsm_nt_f280039c_iris_node"
#define MCS_PMSM_NT_IRIS_SDPE_PROJECT_SUITE "mcs_pmsm_nt"
#define MCS_PMSM_NT_IRIS_SDPE_PROJECT_VERSION "1.2.0"
#define MCS_PMSM_NT_IRIS_SDPE_PROJECT_UPDATED_AT "2026-07-15"

//=================================================================================================
/**
 * @brief ADC.
 */

/**
 * @brief Directly sampled phase-current count; the selected 2136SINV preset provides three low-side channels.
 *        Options: (2), (3)
 */
#define MC_CURRENT_SAMPLE_PHASE_MODE (3)

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
 *        Options: IRIS_LED1, IRIS_LED2
 */
#define SYSTEM_LED IRIS_LED1

/**
 * @brief Controller-running status LED.
 *        Options: IRIS_LED1, IRIS_LED2
 */
#define CONTROLLER_LED IRIS_LED2

//=================================================================================================
/**
 * @brief PWM Channel.
 */

/**
 * @brief U-phase ePWM base.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_U_BASE IRIS_EPWM1_BASE

/**
 * @brief V-phase ePWM base.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_V_BASE IRIS_EPWM2_BASE

/**
 * @brief W-phase ePWM base.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_W_BASE IRIS_EPWM3_BASE

//=================================================================================================
/**
 * @brief Encoder.
 */

/**
 * @brief Rotor encoder eQEP base.
 *        Options: IRIS_EQEP1_BASE
 */
#define EQEP_Encoder_BASE IRIS_EQEP1_BASE

//=================================================================================================
/**
 * @brief ADC Voltage Channels.
 */

/**
 * @brief U-phase voltage ADC result base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_UU_RESULT_BASE ADC_CH1_RESULT_BASE

/**
 * @brief U-phase voltage ADC SOC index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_UU ADC_CH1

/**
 * @brief V-phase voltage ADC result base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_UV_RESULT_BASE ADC_CH2_RESULT_BASE

/**
 * @brief V-phase voltage ADC SOC index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_UV ADC_CH2

/**
 * @brief W-phase voltage ADC result base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_UW_RESULT_BASE ADC_CH3_RESULT_BASE

/**
 * @brief W-phase voltage ADC SOC index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_UW ADC_CH3

//=================================================================================================
/**
 * @brief ADC Current Channels.
 */

/**
 * @brief U-phase current ADC result base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_IU_RESULT_BASE ADC_CH4_RESULT_BASE

/**
 * @brief U-phase current ADC SOC index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_IU ADC_CH4

/**
 * @brief V-phase current ADC result base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_IV_RESULT_BASE ADC_CH5_RESULT_BASE

/**
 * @brief V-phase current ADC SOC index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_IV ADC_CH5

/**
 * @brief W-phase current ADC result base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_IW_RESULT_BASE ADC_CH6_RESULT_BASE

/**
 * @brief W-phase current ADC SOC index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_IW ADC_CH6

//=================================================================================================
/**
 * @brief ADC DC Bus Channel.
 */

/**
 * @brief DC-bus voltage ADC result base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_VBUS_RESULT_BASE ADC_CH7_RESULT_BASE

/**
 * @brief DC-bus voltage ADC SOC index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_VBUS ADC_CH7

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Maximum compare count of the platform PWM peripheral at the configured controller switching frequency.
 */
#define CTRL_PWM_CMP_MAX (3000 - 1)

/**
 * @brief Dead-time count interpreted in the selected PWM peripheral clock domain.
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief Platform CPU or system clock frequency in hertz.
 */
#define CTRL_SYS_FREQUENCY (120e6)

/**
 * @brief C2000 millisecond system-tick divider derived from the CPU clock and ePWM period.
 */
#define DSP_C2000_DSP_TIME_DIV (CTRL_SYS_FREQUENCY / 1000 / CTRL_PWM_CMP_MAX / 2)

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
#define CTRL_POS_ENC_FS (10000)

/**
 * @brief Mechanical encoder position bias in per unit.
 */
#define CTRL_POS_ENC_BIAS (0.0207000002f)

/**
 * @brief Mechanical speed and position division factor.
 */
#define CTRL_MECH_DIV (5)

/**
 * @brief Phase-current sensor sensitivity in volts per ampere.
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY (GMP_3PH_2136SINV_DUAL_PH_SHUNT_RESISTANCE_OHM * GMP_3PH_2136SINV_DUAL_PH_CSA_GAIN_V_V)

/**
 * @brief Phase-current sensor zero-current bias in volts.
 */
#define CTRL_INVERTER_CURRENT_BIAS (GMP_3PH_2136SINV_DUAL_PH_CSA_BIAS_V)

/**
 * @brief Phase-voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (GMP_3PH_2136SINV_DUAL_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Phase-voltage sensor bias in volts.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (GMP_3PH_2136SINV_DUAL_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief DC-bus current sensing gain. The selected inverter reports SENSOR_NONE for this path.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (GMP_3PH_2136SINV_DUAL_DCBUS_CURRENT_SENSE_GAIN)

/**
 * @brief DC-bus current sensor bias.
 */
#define CTRL_DC_CURRENT_BIAS (GMP_3PH_2136SINV_DUAL_DCBUS_CURRENT_SENSE_BIAS_V)

/**
 * @brief DC-bus voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (GMP_3PH_2136SINV_DUAL_DCBUS_VOLTAGE_SENSE_GAIN)

/**
 * @brief DC-bus voltage sensor bias in volts.
 */
#define CTRL_DC_VOLTAGE_BIAS (GMP_3PH_2136SINV_DUAL_DCBUS_VOLTAGE_SENSE_BIAS_V)

// User project tail code
/* No additional platform-specific tail definitions. */

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_MCS_PMSM_NT_IRIS_SETTINGS_H_
