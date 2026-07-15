/**
 * @file sdpe_mcs_pmsm_nt_stm32g474_hrtim_settings.h
 * @brief SDPE project bindings for MCS PMSM NT NUCLEO-G474RE HRTIM.
 * @note NUCLEO-G474RE PMSM settings with independently selectable HRTIM A-E resources for phases U/V/W and USART2 circular-DMA Datalink.
 */

#ifndef _PROJECT_SDPE_MCS_PMSM_NT_STM32G474_HRTIM_SETTINGS_H_
#define _PROJECT_SDPE_MCS_PMSM_NT_STM32G474_HRTIM_SETTINGS_H_

#include <ctl/hardware_preset/inverter_3ph/st_x_nucleo_ihm08m1.h>
#include <ctl/hardware_preset/mcu_board/nucleo_g474re_hrtim_motor_board.h>
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

#define MCS_PMSM_NT_STM32G474_HRTIM_SDPE_PROJECT_ID "mcs_pmsm_nt_stm32g474_hrtim"
#define MCS_PMSM_NT_STM32G474_HRTIM_SDPE_PROJECT_SUITE "mcs_pmsm_nt"
#define MCS_PMSM_NT_STM32G474_HRTIM_SDPE_PROJECT_VERSION "1.2.0"
#define MCS_PMSM_NT_STM32G474_HRTIM_SDPE_PROJECT_UPDATED_AT "2026-07-15"

//=================================================================================================
/**
 * @brief ADC.
 */

/**
 * @brief Directly sampled phase-current count.
 *        Options: (2), (3)
 */
#define MC_CURRENT_SAMPLE_PHASE_MODE (2)

//=================================================================================================
/**
 * @brief HRTIM Phase Mapping.
 */

/**
 * @brief U-phase HRTIM channel: 0=A, 1=B, 2=C, 3=D, 4=E.
 *        Options: 0, 1, 2, 3, 4
 */
#define MCS_HRTIM_PHASE_U_CHANNEL 0

/**
 * @brief V-phase HRTIM channel: 0=A, 1=B, 2=C, 3=D, 4=E.
 *        Options: 0, 1, 2, 3, 4
 */
#define MCS_HRTIM_PHASE_V_CHANNEL 1

/**
 * @brief W-phase HRTIM channel: 0=A, 1=B, 2=C, 3=D, 4=E.
 *        Options: 0, 1, 2, 3, 4
 */
#define MCS_HRTIM_PHASE_W_CHANNEL 2

//=================================================================================================
/**
 * @brief Board.
 */

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
#define CTRL_PWM_CMP_MAX (34000)

/**
 * @brief Dead-time count interpreted in the selected PWM peripheral clock domain.
 */
#define CTRL_PWM_DEADBAND_CMP (100)

/**
 * @brief Platform CPU or system clock frequency in hertz.
 */
#define CTRL_SYS_FREQUENCY (170e6)

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
#define CTRL_INVERTER_CURRENT_SENSITIVITY (ST_X_NUCLEO_IHM08M1_PH_SHUNT_RESISTANCE_OHM * ST_X_NUCLEO_IHM08M1_PH_CSA_GAIN_V_V)

/**
 * @brief Phase-current sensor zero-current bias in volts.
 */
#define CTRL_INVERTER_CURRENT_BIAS (ST_X_NUCLEO_IHM08M1_PH_CSA_BIAS_V)

/**
 * @brief Phase-voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (ST_X_NUCLEO_IHM08M1_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Phase-voltage sensor bias in volts.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (ST_X_NUCLEO_IHM08M1_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief DC-link current scaling is not consumed in the selected phase-current sampling mode.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (ST_X_NUCLEO_IHM08M1_DCBUS_CURRENT_SENSE_GAIN)

/**
 * @brief DC-bus current sensor bias.
 */
#define CTRL_DC_CURRENT_BIAS (ST_X_NUCLEO_IHM08M1_DCBUS_CURRENT_SENSE_BIAS_V)

/**
 * @brief DC-bus voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (ST_X_NUCLEO_IHM08M1_DCBUS_VOLTAGE_SENSE_GAIN)

/**
 * @brief DC-bus voltage sensor bias in volts.
 */
#define CTRL_DC_VOLTAGE_BIAS (ST_X_NUCLEO_IHM08M1_DCBUS_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Circular UART RX DMA buffer size.
 */
#define MCS_UART_RX_BUFFER_SIZE (64)

// User project tail code
/* Platform-specific peripheral selection and validation. */
#if (MCS_HRTIM_PHASE_U_CHANNEL == MCS_HRTIM_PHASE_V_CHANNEL) || (MCS_HRTIM_PHASE_U_CHANNEL == MCS_HRTIM_PHASE_W_CHANNEL) || (MCS_HRTIM_PHASE_V_CHANNEL == MCS_HRTIM_PHASE_W_CHANNEL)
#error "HRTIM phase U/V/W channels must be unique."
#endif
#if (MCS_HRTIM_PHASE_U_CHANNEL > 4) || (MCS_HRTIM_PHASE_V_CHANNEL > 4) || (MCS_HRTIM_PHASE_W_CHANNEL > 4)
#error "HRTIM phase channel must be in range 0 (A) through 4 (E)."
#endif

#define MCS_HRTIM_CAT_RAW(a, b) a##b
#define MCS_HRTIM_CAT(a, b) MCS_HRTIM_CAT_RAW(a, b)
#define MCS_HRTIM_SELECT(prefix, channel) MCS_HRTIM_CAT(prefix, channel)
#define MCS_HRTIM_TIMER_INDEX_0 HRTIM_TIMERINDEX_TIMER_A
#define MCS_HRTIM_TIMER_INDEX_1 HRTIM_TIMERINDEX_TIMER_B
#define MCS_HRTIM_TIMER_INDEX_2 HRTIM_TIMERINDEX_TIMER_C
#define MCS_HRTIM_TIMER_INDEX_3 HRTIM_TIMERINDEX_TIMER_D
#define MCS_HRTIM_TIMER_INDEX_4 HRTIM_TIMERINDEX_TIMER_E
#define MCS_HRTIM_TIMER_ID_0 HRTIM_TIMERID_TIMER_A
#define MCS_HRTIM_TIMER_ID_1 HRTIM_TIMERID_TIMER_B
#define MCS_HRTIM_TIMER_ID_2 HRTIM_TIMERID_TIMER_C
#define MCS_HRTIM_TIMER_ID_3 HRTIM_TIMERID_TIMER_D
#define MCS_HRTIM_TIMER_ID_4 HRTIM_TIMERID_TIMER_E
#define MCS_HRTIM_OUTPUT1_0 HRTIM_OUTPUT_TA1
#define MCS_HRTIM_OUTPUT1_1 HRTIM_OUTPUT_TB1
#define MCS_HRTIM_OUTPUT1_2 HRTIM_OUTPUT_TC1
#define MCS_HRTIM_OUTPUT1_3 HRTIM_OUTPUT_TD1
#define MCS_HRTIM_OUTPUT1_4 HRTIM_OUTPUT_TE1
#define MCS_HRTIM_OUTPUT2_0 HRTIM_OUTPUT_TA2
#define MCS_HRTIM_OUTPUT2_1 HRTIM_OUTPUT_TB2
#define MCS_HRTIM_OUTPUT2_2 HRTIM_OUTPUT_TC2
#define MCS_HRTIM_OUTPUT2_3 HRTIM_OUTPUT_TD2
#define MCS_HRTIM_OUTPUT2_4 HRTIM_OUTPUT_TE2
#define MCS_HRTIM_ADC_UPDATE_0 HRTIM_ADCTRIGGERUPDATE_TIMER_A
#define MCS_HRTIM_ADC_UPDATE_1 HRTIM_ADCTRIGGERUPDATE_TIMER_B
#define MCS_HRTIM_ADC_UPDATE_2 HRTIM_ADCTRIGGERUPDATE_TIMER_C
#define MCS_HRTIM_ADC_UPDATE_3 HRTIM_ADCTRIGGERUPDATE_TIMER_D
#define MCS_HRTIM_ADC_UPDATE_4 HRTIM_ADCTRIGGERUPDATE_TIMER_E
#define MCS_HRTIM_ADC_EVENT_0 HRTIM_ADCTRIGGEREVENT24_TIMERA_PERIOD
#define MCS_HRTIM_ADC_EVENT_1 HRTIM_ADCTRIGGEREVENT24_TIMERB_PERIOD
#define MCS_HRTIM_ADC_EVENT_2 HRTIM_ADCTRIGGEREVENT24_TIMERC_PERIOD
#define MCS_HRTIM_ADC_EVENT_3 HRTIM_ADCTRIGGEREVENT24_TIMERD_PERIOD
#define MCS_HRTIM_ADC_EVENT_4 HRTIM_ADCTRIGGEREVENT24_TIMERE_RESET
#define MCS_HRTIM_PHASE_U_TIMER_INDEX MCS_HRTIM_SELECT(MCS_HRTIM_TIMER_INDEX_, MCS_HRTIM_PHASE_U_CHANNEL)
#define MCS_HRTIM_PHASE_V_TIMER_INDEX MCS_HRTIM_SELECT(MCS_HRTIM_TIMER_INDEX_, MCS_HRTIM_PHASE_V_CHANNEL)
#define MCS_HRTIM_PHASE_W_TIMER_INDEX MCS_HRTIM_SELECT(MCS_HRTIM_TIMER_INDEX_, MCS_HRTIM_PHASE_W_CHANNEL)
#define MCS_HRTIM_PHASE_U_TIMER_ID MCS_HRTIM_SELECT(MCS_HRTIM_TIMER_ID_, MCS_HRTIM_PHASE_U_CHANNEL)
#define MCS_HRTIM_PHASE_V_TIMER_ID MCS_HRTIM_SELECT(MCS_HRTIM_TIMER_ID_, MCS_HRTIM_PHASE_V_CHANNEL)
#define MCS_HRTIM_PHASE_W_TIMER_ID MCS_HRTIM_SELECT(MCS_HRTIM_TIMER_ID_, MCS_HRTIM_PHASE_W_CHANNEL)
#define MCS_HRTIM_PHASE_U_OUTPUT1 MCS_HRTIM_SELECT(MCS_HRTIM_OUTPUT1_, MCS_HRTIM_PHASE_U_CHANNEL)
#define MCS_HRTIM_PHASE_V_OUTPUT1 MCS_HRTIM_SELECT(MCS_HRTIM_OUTPUT1_, MCS_HRTIM_PHASE_V_CHANNEL)
#define MCS_HRTIM_PHASE_W_OUTPUT1 MCS_HRTIM_SELECT(MCS_HRTIM_OUTPUT1_, MCS_HRTIM_PHASE_W_CHANNEL)
#define MCS_HRTIM_PHASE_U_OUTPUT2 MCS_HRTIM_SELECT(MCS_HRTIM_OUTPUT2_, MCS_HRTIM_PHASE_U_CHANNEL)
#define MCS_HRTIM_PHASE_V_OUTPUT2 MCS_HRTIM_SELECT(MCS_HRTIM_OUTPUT2_, MCS_HRTIM_PHASE_V_CHANNEL)
#define MCS_HRTIM_PHASE_W_OUTPUT2 MCS_HRTIM_SELECT(MCS_HRTIM_OUTPUT2_, MCS_HRTIM_PHASE_W_CHANNEL)
#define MCS_HRTIM_PHASE_U_OUTPUTS (MCS_HRTIM_PHASE_U_OUTPUT1 | MCS_HRTIM_PHASE_U_OUTPUT2)
#define MCS_HRTIM_PHASE_V_OUTPUTS (MCS_HRTIM_PHASE_V_OUTPUT1 | MCS_HRTIM_PHASE_V_OUTPUT2)
#define MCS_HRTIM_PHASE_W_OUTPUTS (MCS_HRTIM_PHASE_W_OUTPUT1 | MCS_HRTIM_PHASE_W_OUTPUT2)
#define MCS_HRTIM_ADC_UPDATE_SOURCE MCS_HRTIM_SELECT(MCS_HRTIM_ADC_UPDATE_, MCS_HRTIM_PHASE_U_CHANNEL)
#define MCS_HRTIM_ADC_TRIGGER_EVENT MCS_HRTIM_SELECT(MCS_HRTIM_ADC_EVENT_, MCS_HRTIM_PHASE_U_CHANNEL)

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_MCS_PMSM_NT_STM32G474_HRTIM_SETTINGS_H_
