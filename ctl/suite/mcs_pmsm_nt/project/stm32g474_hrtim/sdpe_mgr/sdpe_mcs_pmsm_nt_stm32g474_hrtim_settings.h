/**
 * @file sdpe_mcs_pmsm_nt_stm32g474_hrtim_settings.h
 * @brief SDPE project bindings for MCS PMSM NT NUCLEO-G474RE HRTIM.
 * @note NUCLEO-G474RE PMSM settings with independently selectable HRTIM A-E resources for phases U/V/W and USART2 circular-DMA Datalink.
 */

#ifndef _PROJECT_SDPE_MCS_PMSM_NT_STM32G474_HRTIM_SETTINGS_H_
#define _PROJECT_SDPE_MCS_PMSM_NT_STM32G474_HRTIM_SETTINGS_H_

#include <ctl/hardware_preset/mcu_board/nucleo_g474re_hrtim_motor_board.h>
#include <ctl/hardware_preset/pmsm_motor/sm060r20b30mnad.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* BOOSTXL inverter compatibility. */
#define BOOSTXL_3PHGANINV_IS_DEFAULT_PARAM
#include <ctl/component/hardware_preset/inverter_3ph/GMP_3PH_2136SINV_DUAL_TMPL.h>
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

#define SDPE_PROJECT_ID "mcs_pmsm_nt_stm32g474_hrtim"
#define SDPE_PROJECT_SUITE "mcs_pmsm_nt"
#define SDPE_PROJECT_VERSION "1.0.0"
#define SDPE_PROJECT_UPDATED_AT "2026-07-14"

//=================================================================================================
/**
 * @brief Control Algorithm.
 */

/**
 * @brief Use discrete controller implementation.
 */
// #define PMSM_CTRL_USING_DISCRETE_CTRL

/**
 * @brief Enable discrete PID anti-saturation.
 */
#define _USE_DEBUG_DISCRETE_PID

/**
 * @brief Enable sliding-mode observer.
 */
#define ENABLE_SMO

//=================================================================================================
/**
 * @brief Protection.
 */

/**
 * @brief Enable motor fault protection.
 */
#define ENABLE_MOTOR_FAULT_PROTECTION

//=================================================================================================
/**
 * @brief ADC.
 */

/**
 * @brief Enable ADC offset calibration.
 */
#define SPECIFY_ENABLE_ADC_CALIBRATE

//=================================================================================================
/**
 * @brief PWM.
 */

/**
 * @brief Use active-low inverter modulation.
 */
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC (1)

/**
 * @brief Enable NPC modulation.
 */
// #define USING_NPC_MODULATOR

//=================================================================================================
/**
 * @brief Controller.
 */

/**
 * @brief Commissioning build level.
 *        Options: (1), (2), (3), (4)
 */
#define BUILD_LEVEL (4)

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
 * @brief Controller startup delay in milliseconds.
 */
#define CTRL_STARTUP_DELAY (100)

/**
 * @brief Motor-control update frequency in hertz.
 */
#define CONTROLLER_FREQUENCY (20e3f)

/**
 * @brief HRTIM period for 20 kHz center-aligned PWM at 170 MHz with x8 multiplier.
 */
#define CTRL_PWM_CMP_MAX (34000)

/**
 * @brief Advanced-timer dead-time count.
 */
#define CTRL_PWM_DEADBAND_CMP (100)

/**
 * @brief STM32G474 system clock frequency.
 */
#define CTRL_SYS_FREQUENCY (170e6)

/**
 * @brief ADC reference voltage.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief ADC calibration timeout in milliseconds.
 */
#define TIMEOUT_ADC_CALIB_MS (10000)

/**
 * @brief DC-bus voltage base.
 */
#define CTRL_DCBUS_VOLTAGE (80.0f)

/**
 * @brief SVPWM phase-voltage base.
 */
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

/**
 * @brief Phase-current base in amperes.
 */
#define CTRL_CURRENT_BASE (10.0f)

/**
 * @brief Encoder counts per revolution.
 */
#define CTRL_POS_ENC_FS (10000)

/**
 * @brief Encoder position bias in per unit.
 */
#define CTRL_POS_ENC_BIAS (0.0207000002f)

/**
 * @brief Mechanical speed and position division.
 */
#define CTRL_MECH_DIV (5)

/**
 * @brief Phase-current sensor sensitivity.
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY (MY_BOARD_PH_SHUNT_RESISTANCE_OHM * MY_BOARD_PH_CSA_GAIN_V_V)

/**
 * @brief Phase-current sensor bias.
 */
#define CTRL_INVERTER_CURRENT_BIAS (MY_BOARD_PH_CSA_BIAS_V)

/**
 * @brief Phase-voltage sensor gain.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (MY_BOARD_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Phase-voltage sensor bias.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief DC-bus current sensor gain.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (MY_BOARD_DCBUS_CURRENT_SENSE_GAIN)

/**
 * @brief DC-bus current sensor bias.
 */
#define CTRL_DC_CURRENT_BIAS (MY_BOARD_DCBUS_CURRENT_SENSE_BIAS_V)

/**
 * @brief DC-bus voltage sensor gain.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN)

/**
 * @brief DC-bus voltage sensor bias.
 */
#define CTRL_DC_VOLTAGE_BIAS (MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Dead-time compensation current deadband.
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A (0.2f)

/**
 * @brief Dead-time compensation hysteresis.
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A (0.05f)

/**
 * @brief Open-loop electrical frequency.
 */
#define MCS_OPEN_LOOP_FREQ_HZ (20.0f)

/**
 * @brief Open-loop frequency slew rate.
 */
#define MCS_OPEN_LOOP_FREQ_SLOPE_HZ_S (20.0f)

/**
 * @brief Position-loop proportional gain.
 */
#define MCS_MECH_POSITION_KP_PU (5.0f)

/**
 * @brief Position-loop integral gain.
 */
#define MCS_MECH_POSITION_KI_PU_S (1.0f)

/**
 * @brief Velocity-loop proportional gain.
 */
#define MCS_MECH_VELOCITY_KP_PU (5.0f)

/**
 * @brief Velocity-loop integral gain.
 */
#define MCS_MECH_VELOCITY_KI_PU_S (1.0f)

/**
 * @brief Mechanical speed limit.
 */
#define MCS_MECH_SPEED_LIMIT_RPM (MOTOR_PARAM_MAX_SPEED)

/**
 * @brief Mechanical speed slew rate.
 */
#define MCS_MECH_SPEED_SLOPE_RPM_S (MOTOR_PARAM_MAX_SPEED)

/**
 * @brief Mechanical current limit.
 */
#define MCS_MECH_CURRENT_LIMIT_A (0.3f * CTRL_CURRENT_BASE)

/**
 * @brief Encoder speed filter cutoff.
 */
#define MCS_ENCODER_SPEED_FILTER_FC_HZ (20.0f)

/**
 * @brief Commissioning d-axis current.
 */
#define MCS_COMMISSIONING_ID_REF_A (0.1f * CTRL_CURRENT_BASE)

/**
 * @brief Commissioning q-axis current.
 */
#define MCS_COMMISSIONING_IQ_REF_A (0.1f * CTRL_CURRENT_BASE)

/**
 * @brief Commissioning speed.
 */
#define MCS_COMMISSIONING_SPEED_REF_RPM (0.1f * MOTOR_PARAM_MAX_SPEED)

/**
 * @brief CiA402 operation-enable delay.
 */
#define MCS_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief ADC calibrator cutoff.
 */
#define MCS_ADC_CALIBRATOR_FC_HZ (20.0f)

/**
 * @brief ADC calibrator quality factor.
 */
#define MCS_ADC_CALIBRATOR_Q (0.707f)

/**
 * @brief Circular UART RX DMA buffer size.
 */
#define MCS_UART_RX_BUFFER_SIZE (64)

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be in range 1..4."
#endif
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
