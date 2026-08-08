/**
 * @file sdpe_mcs_pmsm_nt_stm32g431_settings.h
 * @brief SDPE project bindings for MCS PMSM NT NUCLEO-G431RB.
 * @note NUCLEO-G431RB PMSM settings with selectable TIM1/TIM8 motor PWM and USART2 circular-DMA Datalink.
 */

#ifndef _PROJECT_SDPE_MCS_PMSM_NT_STM32G431_SETTINGS_H_
#define _PROJECT_SDPE_MCS_PMSM_NT_STM32G431_SETTINGS_H_

#include <ctl/hardware_preset/inverter_3ph/st_x_nucleo_ihm08m1.h>
#include <ctl/hardware_preset/mcu_board/nucleo_g431rb_motor_board.h>
#include <ctl/hardware_preset/pmsm_motor/sm060r20b30mnad.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
// SDPE extension point: add after_extern_open code in the Project Requirement Code page if needed.

// Common prefix code: MCS PMSM NT Common Controller Settings
/* Platform-independent settings only. Project hardware is supplied by the including project header. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define MCS_PMSM_NT_STM32G431_SDPE_PROJECT_ID "mcs_pmsm_nt_stm32g431"
#define MCS_PMSM_NT_STM32G431_SDPE_PROJECT_SUITE "mcs_pmsm_nt"
#define MCS_PMSM_NT_STM32G431_SDPE_PROJECT_VERSION "1.2.0"
#define MCS_PMSM_NT_STM32G431_SDPE_PROJECT_UPDATED_AT "2026-08-08"

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
 * @brief Select TIM1 or TIM8 as the complete three-phase PWM and ADC trigger source.
 *        Options: 1, 8
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
 * @brief Number of samples stored per channel by the four-channel hardware Data Link Scope.
 */
#define GMP_DL_SCOPE_DEPTH (100)

/**
 * @brief Maximum compare count of the platform PWM peripheral at the configured controller switching frequency.
 */
#define CTRL_PWM_CMP_MAX (4249)

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

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_TYPE SM060R20B30MNAD_MOTOR_TYPE

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_RS SM060R20B30MNAD_RS

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_LS SM060R20B30MNAD_LD

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_LD SM060R20B30MNAD_LD

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_LQ SM060R20B30MNAD_LQ

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_FLUX SM060R20B30MNAD_FLUX

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_POLE_PAIRS SM060R20B30MNAD_POLE_PAIRS

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_INERTIA SM060R20B30MNAD_INERTIA

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_FRICTION SM060R20B30MNAD_FRICTION

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_KV SM060R20B30MNAD_KV

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_EMF SM060R20B30MNAD_EMF

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_RATED_VOLTAGE SM060R20B30MNAD_RATED_VOLTAGE

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_RATED_CURRENT SM060R20B30MNAD_RATED_CURRENT

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_NO_LOAD_CURRENT SM060R20B30MNAD_NO_LOAD_CURRENT

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_RATED_FREQUENCY SM060R20B30MNAD_RATED_FREQUENCY

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_MAX_SPEED SM060R20B30MNAD_MAX_SPEED

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_MAX_TORQUE SM060R20B30MNAD_MAX_TORQUE

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_MAX_DC_VOLTAGE SM060R20B30MNAD_MAX_DC_VOLTAGE

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_MAX_PH_CURRENT SM060R20B30MNAD_MAX_PH_CURRENT

//=================================================================================================
/**
 * @brief Common fallbacks: MCS PMSM NT Common Controller Settings.
 */

//=================================================================================================
/**
 * @brief Control Algorithm.
 */

/**
 * @brief Use the discrete controller implementation instead of the default continuous controller path.
 */
// #define PMSM_CTRL_USING_DISCRETE_CTRL

/**
 * @brief Enable the existing discrete-PID anti-saturation debug path.
 */
#define _USE_DEBUG_DISCRETE_PID

/**
 * @brief Enable the sliding-mode observer path. Disabled to preserve the current sensored-control build.
 */
// #define ENABLE_SMO

/**
 * @brief Enable motor fault protection processing.
 */
// #define ENABLE_MOTOR_FAULT_PROTECTION

//=================================================================================================
/**
 * @brief Controller Runtime.
 */

/**
 * @brief Calibrate ADC offsets before enabling normal control.
 */
#define SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Enable processor-in-the-loop mode and suppress direct PWM controller output.
 */
// #define ENABLE_GMP_DL_PIL_SIM

/**
 * @brief Enable CiA402/GMP framework debug information.
 */
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

//=================================================================================================
/**
 * @brief PWM Modulator.
 */

/**
 * @brief Use active-low PWM modulation for the selected inverter gate path.
 */
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC (1)

/**
 * @brief Use the three-level NPC modulator. Disabled for the two-level three-phase inverter.
 */
// #define USING_NPC_MODULATOR

//=================================================================================================
/**
 * @brief Controller Options.
 */

/**
 * @brief Incremental commissioning level. 1: V/f voltage open loop; 2: current loop with synthetic electrical angle; 3: current loop with encoder angle; 4: speed loop with encoder feedback.
 *        Options: (1), (2), (3), (4)
 */
#define BUILD_LEVEL (2)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Main motor-control ISR frequency in hertz.
 */
#define CONTROLLER_FREQUENCY (20e3f)

/**
 * @brief Minimum absolute phase current in amperes before PWM dead-time compensation selects a current direction. Converted to PU with CTRL_CURRENT_BASE at initialization.
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A (0.2f)

/**
 * @brief Phase-current hysteresis band in amperes used to prevent dead-time compensation direction chatter around zero current.
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A (0.05f)

/**
 * @brief
 */
#ifndef CTRL_SPEED_RPM_BASE
#define CTRL_SPEED_RPM_BASE MOTOR_PARAM_MAX_SPEED
#endif // CTRL_SPEED_RPM_BASE

/**
 * @brief
 */
#ifndef MOTOR_PARAM_RS
#define MOTOR_PARAM_RS SM060R20B30MNAD_RS
#endif // MOTOR_PARAM_RS

/**
 * @brief
 */
#ifndef MOTOR_PARAM_LS
#define MOTOR_PARAM_LS SM060R20B30MNAD_LD
#endif // MOTOR_PARAM_LS

/**
 * @brief
 */
#ifndef MOTOR_PARAM_LD
#define MOTOR_PARAM_LD SM060R20B30MNAD_LD
#endif // MOTOR_PARAM_LD

/**
 * @brief
 */
#ifndef MOTOR_PARAM_LQ
#define MOTOR_PARAM_LQ SM060R20B30MNAD_LQ
#endif // MOTOR_PARAM_LQ

/**
 * @brief
 */
#ifndef MOTOR_PARAM_FLUX
#define MOTOR_PARAM_FLUX SM060R20B30MNAD_FLUX
#endif // MOTOR_PARAM_FLUX

/**
 * @brief
 */
#ifndef MOTOR_PARAM_POLE_PAIRS
#define MOTOR_PARAM_POLE_PAIRS SM060R20B30MNAD_POLE_PAIRS
#endif // MOTOR_PARAM_POLE_PAIRS

/**
 * @brief
 */
#ifndef MOTOR_PARAM_INERTIA
#define MOTOR_PARAM_INERTIA SM060R20B30MNAD_INERTIA
#endif // MOTOR_PARAM_INERTIA

/**
 * @brief
 */
#ifndef MOTOR_PARAM_FRICTION
#define MOTOR_PARAM_FRICTION SM060R20B30MNAD_FRICTION
#endif // MOTOR_PARAM_FRICTION

/**
 * @brief Cutoff frequency in hertz of the low-pass filter applied to encoder-derived mechanical speed.
 */
#define MCS_ENCODER_SPEED_FILTER_FC_HZ (20.0f)

/**
 * @brief Cutoff frequency in hertz of the second-order low-pass filter used while estimating ADC zero offsets.
 */
#define MCS_ADC_CALIBRATOR_FC_HZ (20.0f)

/**
 * @brief Quality factor of the ADC calibration low-pass filter; 0.707 gives an approximately Butterworth second-order response.
 */
#define MCS_ADC_CALIBRATOR_Q (0.707f)

/**
 * @brief Absolute mechanical speed-command limit in rpm. It is divided by MOTOR_PARAM_MAX_SPEED to obtain the controller PU limit.
 */
#define MCS_MECH_SPEED_LIMIT_RPM (3000.0f)

/**
 * @brief Maximum mechanical speed-command slew rate in rpm/s. The configured value corresponds to 1 PU/s for the selected 3000 rpm motor.
 */
#define MCS_MECH_SPEED_SLOPE_RPM_S (3000.0f)

/**
 * @brief Absolute q-axis current/torque command limit in amperes. It is divided by CTRL_CURRENT_BASE to obtain the controller PU limit; 3 A corresponds to the previous 0.3 PU setting.
 */
#define MCS_MECH_CURRENT_LIMIT_A (3.0f)

/**
 * @brief Rectangular saturation limit for d/q axes in V.
 */
#define MCS_MAX_RECT_SATURATION_VOLTAGE_V (10.0f)

/**
 * @brief
 */
#define MCS_MAX_DC_BUS_VOLTAGE_V (CTRL_DCBUS_VOLTAGE*1.2f)

/**
 * @brief The current limit value at which the machine must be shut down.
 */
#ifndef MCS_MAX_SHUTDOWN_CURRENT_A
#define MCS_MAX_SHUTDOWN_CURRENT_A (10.0f)
#endif // MCS_MAX_SHUTDOWN_CURRENT_A

/**
 * @brief Circular saturation limit for voltage vector magnitude in V.
 */
#define MCS_MAX_CIR_SATURATION_VOLTAGE_V (10.0f)

/**
 * @brief D-axis current reference in amperes used by BUILD_LEVEL 2 and 3 commissioning. Converted to PU using CTRL_CURRENT_BASE.
 */
#define MCS_COMMISSIONING_ID_REF_A (1.0f)

/**
 * @brief Q-axis current reference in amperes used by BUILD_LEVEL 2 and 3 commissioning. Converted to PU using CTRL_CURRENT_BASE.
 */
#define MCS_COMMISSIONING_IQ_REF_A (1.0f)

/**
 * @brief Mechanical speed reference in rpm used by BUILD_LEVEL 4 commissioning. Converted to PU using MOTOR_PARAM_MAX_SPEED.
 */
#define MCS_COMMISSIONING_SPEED_REF_RPM (300.0f)

/**
 * @brief Electrical frequency command in hertz used by the BUILD_LEVEL 1 V/f path and the BUILD_LEVEL 2 synthetic-angle current-loop path.
 */
#define MCS_OPEN_LOOP_FREQ_HZ (20.0f)

/**
 * @brief Maximum electrical-frequency slew rate in hertz per second for the synthetic angle generator.
 */
#define MCS_OPEN_LOOP_FREQ_SLOPE_HZ_S (20.0f)

/**
 * @brief Position-loop proportional gain. Input is mechanical position error in PU revolutions and output is speed reference in PU, so the gain is speed_pu/position_pu.
 */
#define MCS_MECH_POSITION_KP_PU (5.0f)

/**
 * @brief Position-loop integral gain in speed_pu/(position_pu*s). The continuous gain is divided by the mechanical-loop sampling frequency internally.
 */
#define MCS_MECH_POSITION_KI_PU_S (1.0f)

/**
 * @brief Velocity-loop proportional gain. Input is speed error in PU and output is q-axis current/torque reference in PU, so the gain is current_pu/speed_pu.
 */
#define MCS_MECH_VELOCITY_KP_PU (5.0f)

/**
 * @brief Velocity-loop integral gain in current_pu/(speed_pu*s). The continuous gain is divided by the mechanical-loop sampling frequency internally.
 */
#define MCS_MECH_VELOCITY_KI_PU_S (1.0f)

/**
 * @brief Controller startup delay in milliseconds.
 */
#define CTRL_STARTUP_DELAY (100)

/**
 * @brief Minimum delay in milliseconds before the CiA402 state machine enters Operation Enabled. The project control tick is expressed in milliseconds.
 */
#define MCS_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief
 */
#define MCS_MIN_DC_BUS_VOLTAGE_V (CTRL_DCBUS_VOLTAGE*0.2f)

// Common tail code: MCS PMSM NT Common Controller Settings
/* Accept the historical misspelling while all source code uses the canonical switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif

/* Reject unsupported incremental build levels at preprocessing time. */
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be 1 (V/f), 2 (current loop/synthetic angle), 3 (current loop/encoder), or 4 (speed loop)."
#endif

// User project tail code
/* Platform-specific peripheral selection and validation. */
#if MCS_PWM_TIMER_SELECTION == 1
#define MCS_PWM_TIMER_HANDLE (&htim1)
#define MCS_PWM_ADC_TRIGGER ADC_EXTERNALTRIGINJEC_T1_TRGO
#elif MCS_PWM_TIMER_SELECTION == 8
#define MCS_PWM_TIMER_HANDLE (&htim8)
#define MCS_PWM_ADC_TRIGGER ADC_EXTERNALTRIGINJEC_T8_TRGO
#else
#error "MCS_PWM_TIMER_SELECTION must be 1 or 8."
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_MCS_PMSM_NT_STM32G431_SETTINGS_H_
