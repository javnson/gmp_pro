/**
 * @file sdpe_mcs_pmsm_nt_stm32f405_settings.h
 * @brief SDPE project bindings for MCS PMSM NT STM32F405.
 * @note Validated STM32F405 PMSM project settings, timer resources and USART2 DMA Datalink bindings.
 */

#ifndef _PROJECT_SDPE_MCS_PMSM_NT_STM32F405_SETTINGS_H_
#define _PROJECT_SDPE_MCS_PMSM_NT_STM32F405_SETTINGS_H_

#include <ctl/hardware_preset/inverter_3ph/gmp_3ph_2136sinv_dual.h>
#include <ctl/hardware_preset/mcu_board/stm32f405_motor_board.h>
#include <ctl/hardware_preset/pmsm_motor/sm060r20b30mnad.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
#include <sdpe_mcs_pmsm_nt_common_settings.h>

/* SDPE inverter selection. */
#define MOTOR_TYPE SM060R20B30MNAD_MOTOR_TYPE
#define MOTOR_PARAM_RS SM060R20B30MNAD_RS
#define MOTOR_PARAM_LS SM060R20B30MNAD_LD
#define MOTOR_PARAM_LD SM060R20B30MNAD_LD
#define MOTOR_PARAM_LQ SM060R20B30MNAD_LQ
#define MOTOR_PARAM_FLUX SM060R20B30MNAD_FLUX
#define MOTOR_PARAM_POLE_PAIRS SM060R20B30MNAD_POLE_PAIRS
#define MOTOR_PARAM_INERTIA SM060R20B30MNAD_INERTIA
#define MOTOR_PARAM_FRICTION SM060R20B30MNAD_FRICTION
#define MOTOR_PARAM_KV SM060R20B30MNAD_KV
#define MOTOR_PARAM_EMF SM060R20B30MNAD_EMF
#define MOTOR_PARAM_RATED_VOLTAGE SM060R20B30MNAD_RATED_VOLTAGE
#define MOTOR_PARAM_RATED_CURRENT SM060R20B30MNAD_RATED_CURRENT
#define MOTOR_PARAM_NO_LOAD_CURRENT SM060R20B30MNAD_NO_LOAD_CURRENT
#define MOTOR_PARAM_RATED_FREQUENCY SM060R20B30MNAD_RATED_FREQUENCY
#define MOTOR_PARAM_MAX_SPEED SM060R20B30MNAD_MAX_SPEED
#define MOTOR_PARAM_MAX_TORQUE SM060R20B30MNAD_MAX_TORQUE
#define MOTOR_PARAM_MAX_DC_VOLTAGE SM060R20B30MNAD_MAX_DC_VOLTAGE
#define MOTOR_PARAM_MAX_PH_CURRENT SM060R20B30MNAD_MAX_PH_CURRENT

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define MCS_PMSM_NT_STM32F405_SDPE_PROJECT_ID "mcs_pmsm_nt_stm32f405"
#define MCS_PMSM_NT_STM32F405_SDPE_PROJECT_SUITE "mcs_pmsm_nt"
#define MCS_PMSM_NT_STM32F405_SDPE_PROJECT_VERSION "1.2.0"
#define MCS_PMSM_NT_STM32F405_SDPE_PROJECT_UPDATED_AT "2026-07-15"

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
 * @brief Board.
 */

/**
 * @brief Advanced motor PWM timer selection.
 *        Options: 1
 */
#define MCS_PWM_TIMER_SELECTION 1

/**
 * @brief Datalink UART handle.
 *        Options: (&huart2)
 */
#define MCS_UART_HANDLE (&huart2)

/**
 * @brief Datalink UART instance.
 *        Options: USART2
 */
#define MCS_UART_INSTANCE USART2

/**
 * @brief Encoder timer handle.
 *        Options: (&htim3)
 */
#define MCS_ENCODER_TIMER_HANDLE (&htim3)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Maximum compare count of the platform PWM peripheral at the configured controller switching frequency.
 */
#define CTRL_PWM_CMP_MAX (4199)

/**
 * @brief Dead-time count interpreted in the selected PWM peripheral clock domain.
 */
#define CTRL_PWM_DEADBAND_CMP (100)

/**
 * @brief Platform CPU or system clock frequency in hertz.
 */
#define CTRL_SYS_FREQUENCY (168e6)

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

/**
 * @brief Circular UART RX DMA buffer size.
 */
#define MCS_UART_RX_BUFFER_SIZE (64)

// User project tail code
/* Platform-specific peripheral selection and validation. */
#define MCS_PWM_TIMER_HANDLE (&htim1)
#define MCS_PWM_ADC_TRIGGER ADC_EXTERNALTRIGINJEC_T1_TRGO

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_MCS_PMSM_NT_STM32F405_SETTINGS_H_
