/**
 * @file sdpe_mcs_pmsm_nt_iris_settings.h
 * @brief SDPE project bindings for MCS PMSM NT F280039C IRIS Node.
 * @note PMSM sensored-control project settings for the F280039C IRIS node. This SDPE requirement is the single source of truth for controller timing, per-unit bases, encoder and sensor scaling, optional control features, and project peripheral bindings formerly held in xplt/ctrl_settings.h.
 */

#ifndef _PROJECT_SDPE_MCS_PMSM_NT_IRIS_SETTINGS_H_
#define _PROJECT_SDPE_MCS_PMSM_NT_IRIS_SETTINGS_H_

#include <ctl/hardware_preset/mcu_board/iris_f280039c_node.h>
#include <ctl/hardware_preset/pmsm_motor/sm060r20b30mnad.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* The inverter preset remains a legacy component until it has an SDPE entity. */
#define BOOSTXL_3PHGANINV_IS_DEFAULT_PARAM
#include <ctl/component/hardware_preset/inverter_3ph/GMP_3PH_2136SINV_DUAL_TMPL.h>

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

#define SDPE_PROJECT_ID "mcs_pmsm_nt_f280039c_iris_node"
#define SDPE_PROJECT_SUITE "mcs_pmsm_nt"
#define SDPE_PROJECT_VERSION "1.0.0"
#define SDPE_PROJECT_UPDATED_AT "2026-07-14"

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
 * @brief Controller startup delay in milliseconds.
 */
#define CTRL_STARTUP_DELAY (100)

/**
 * @brief Main motor-control ISR frequency in hertz.
 */
#define CONTROLLER_FREQUENCY (20e3f)

/**
 * @brief Maximum PWM compare count for 20 kHz center-aligned PWM at 120 MHz.
 */
#define CTRL_PWM_CMP_MAX (3000 - 1)

/**
 * @brief PWM dead-band count.
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief F280039C system clock frequency in hertz.
 */
#define CTRL_SYS_FREQUENCY (120e6)

/**
 * @brief C2000 system tick divider derived from the system clock and PWM period.
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
#define CTRL_INVERTER_CURRENT_SENSITIVITY (MY_BOARD_PH_SHUNT_RESISTANCE_OHM * MY_BOARD_PH_CSA_GAIN_V_V)

/**
 * @brief Phase-current sensor zero-current bias in volts.
 */
#define CTRL_INVERTER_CURRENT_BIAS (MY_BOARD_PH_CSA_BIAS_V)

/**
 * @brief Phase-voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (MY_BOARD_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Phase-voltage sensor bias in volts.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief DC-bus current sensing gain. The selected inverter reports SENSOR_NONE for this path.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (MY_BOARD_DCBUS_CURRENT_SENSE_GAIN)

/**
 * @brief DC-bus current sensor bias.
 */
#define CTRL_DC_CURRENT_BIAS (MY_BOARD_DCBUS_CURRENT_SENSE_BIAS_V)

/**
 * @brief DC-bus voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN)

/**
 * @brief DC-bus voltage sensor bias in volts.
 */
#define CTRL_DC_VOLTAGE_BIAS (MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Minimum absolute phase current in amperes before PWM dead-time compensation selects a current direction. Converted to PU with CTRL_CURRENT_BASE at initialization.
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A (0.2f)

/**
 * @brief Phase-current hysteresis band in amperes used to prevent dead-time compensation direction chatter around zero current.
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A (0.05f)

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
 * @brief Cutoff frequency in hertz of the low-pass filter applied to encoder-derived mechanical speed.
 */
#define MCS_ENCODER_SPEED_FILTER_FC_HZ (20.0f)

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
 * @brief Minimum delay in milliseconds before the CiA402 state machine enters Operation Enabled. The project control tick is expressed in milliseconds.
 */
#define MCS_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief Cutoff frequency in hertz of the second-order low-pass filter used while estimating ADC zero offsets.
 */
#define MCS_ADC_CALIBRATOR_FC_HZ (20.0f)

/**
 * @brief Quality factor of the ADC calibration low-pass filter; 0.707 gives an approximately Butterworth second-order response.
 */
#define MCS_ADC_CALIBRATOR_Q (0.707f)

/**
 * @brief Legacy loop-convergence epsilon retained for controller compatibility.
 */
#define CTRL_SPLL_EPSILON ((float2ctrl(0.005)))

// User project tail code
/* Accept the historical misspelling while all source code uses the canonical switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif

/* Reject unsupported incremental build levels at preprocessing time. */
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be 1 (V/f), 2 (current loop/synthetic angle), 3 (current loop/encoder), or 4 (speed loop)."
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_MCS_PMSM_NT_IRIS_SETTINGS_H_
