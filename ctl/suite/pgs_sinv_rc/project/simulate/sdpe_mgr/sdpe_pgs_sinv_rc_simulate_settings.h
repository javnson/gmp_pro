/**
 * @file sdpe_pgs_sinv_rc_simulate_settings.h
 * @brief SDPE project bindings for PGS SINV RC MATLAB/Simulink SIL.
 * @note Windows SIL platform, plant and sensing settings for all SINV commissioning models.
 */

#ifndef _PROJECT_SDPE_PGS_SINV_RC_SIMULATE_SETTINGS_H_
#define _PROJECT_SDPE_PGS_SINV_RC_SIMULATE_SETTINGS_H_

#include <ctl/hardware_preset/grid_lc_filter/gmp_harmonia_3ph_lc_filter.h>
#include <ctl/hardware_preset/half_bridge/gmp_lvfb_150_2ph_v2.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
#define SPECIFY_PC_ENVIRONMENT

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_SINV_RC_SIM_SDPE_PROJECT_ID "pgs_sinv_rc_simulate"
#define PGS_SINV_RC_SIM_SDPE_PROJECT_SUITE "pgs_sinv_rc"
#define PGS_SINV_RC_SIM_SDPE_PROJECT_VERSION "1.0.0"
#define PGS_SINV_RC_SIM_SDPE_PROJECT_UPDATED_AT "2026-08-08"

//=================================================================================================
/**
 * @brief SIL Runtime.
 */

/**
 * @brief Automatically request CiA402 ENABLE_OPERATION in the simulation executable.
 */
#define SINV_SIM_AUTO_ENABLE

/**
 * @brief Simulated sensor ADC offsets are deterministic and do not require startup calibration.
 */
// #define SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Use the native ASIO UDP SIL path instead of datalink PIL.
 */
// #define ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
/**
 * @brief Commissioning.
 */

/**
 * @brief Single-phase converter commissioning level.
 *        BUILD_LEVEL 1: open-loop sinusoidal H-bridge voltage on an isolated resistive load; validates ADC polarity, PWM mapping and the power stage without a current loop.
 *        BUILD_LEVEL 2: closed AC-current loop on an isolated resistive load; validates QPR tracking, grid-voltage feedforward and optional FDRC.
 *        BUILD_LEVEL 3: grid-connected signed P/Q command through the current loop; positive P exports power and negative P rectifies.
 *        BUILD_LEVEL 4: grid-connected measured-active-power outer loop feeding the current loop.
 *        BUILD_LEVEL 5: active-front-end rectifier with a DC-bus voltage outer loop; takeover is initialized from the measured passive-rectifier power.
 *        Options: (1), (2), (3), (4), (5)
 */
#define BUILD_LEVEL (5)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief SIL controller and PWM update frequency.
 */
#define CONTROLLER_FREQUENCY (20e3f)

/**
 * @brief Plant switching frequency.
 */
#define SINV_PWM_FREQUENCY_HZ (20e3f)

/**
 * @brief Virtual PWM compare maximum.
 */
#define CTRL_PWM_CMP_MAX (2999)

/**
 * @brief Virtual PWM deadband counts.
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief ADC resolution used by all sensor blocks.
 */
#define CTRL_ADC_RESOLUTION (12)

/**
 * @brief ADC reference voltage.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief Rated and nominal DC-bus voltage.
 */
#define CTRL_DCBUS_VOLTAGE (60.0f)

/**
 * @brief Nominal grid/load RMS voltage.
 */
#define CTRL_GRID_VOLTAGE_RMS (24.0f)

/**
 * @brief Rated RMS AC current.
 */
#define CTRL_RATED_CURRENT_RMS (10.0f)

/**
 * @brief Peak voltage PU base.
 */
#define CTRL_VOLTAGE_BASE (34.0f)

/**
 * @brief Peak current PU base.
 */
#define CTRL_CURRENT_BASE (14.14f)

/**
 * @brief AC filter series inductance.
 */
#define CTRL_AC_INDUCTANCE (480e-6f)

/**
 * @brief AC filter series resistance.
 */
#define CTRL_AC_RESISTANCE (0.10f)

/**
 * @brief AC filter shunt capacitance.
 */
#define SINV_FILTER_CAPACITANCE_F (22e-6f)

/**
 * @brief Filter capacitor ESR.
 */
#define SINV_FILTER_CAP_ESR_OHM (0.10f)

/**
 * @brief DC-link capacitance.
 */
#define SINV_DC_CAPACITANCE_F (2200e-6f)

/**
 * @brief Resistive load for levels 1 and 2.
 */
#define SINV_RLOAD_OHM (12.0f)

/**
 * @brief DC-side load for level 5. At 60 V this draws 120 W, within the configured converter current rating.
 */
#define SINV_RECTIFIER_RLOAD_OHM (30.0f)

/**
 * @brief AC voltage sensor sensitivity in V/V.
 */
#define CTRL_AC_VOLTAGE_SENSITIVITY (0.020f)

/**
 * @brief AC voltage ADC bias.
 */
#define CTRL_AC_VOLTAGE_BIAS (1.65f)

/**
 * @brief AC current sensor sensitivity in V/A.
 */
#define CTRL_AC_CURRENT_SENSITIVITY (0.150f)

/**
 * @brief AC current ADC bias.
 */
#define CTRL_AC_CURRENT_BIAS (1.65f)

/**
 * @brief DC bus voltage sensor sensitivity in V/V.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (0.040f)

/**
 * @brief DC bus voltage ADC bias.
 */
#define CTRL_DC_VOLTAGE_BIAS (0.0f)

/**
 * @brief DC bus overvoltage threshold.
 */
#define CTRL_PROT_VBUS_MAX (90.0f)

/**
 * @brief Fast AC peak-current threshold.
 */
#define CTRL_PROT_IAC_PEAK_MAX (18.0f)

/**
 * @brief Controller divergence threshold; BUILD_LEVEL 5 masks it only during passive-rectifier takeover.
 */
#define CTRL_PROT_VCTRL_MAX_PU (1.5f)

/**
 * @brief Minimum precharged DC bus accepted by startup.
 */
#define CTRL_DCBUS_READY_MIN (25.0f)

/**
 * @brief Maximum DC bus accepted by startup.
 */
#define CTRL_DCBUS_READY_MAX (90.0f)

/**
 * @brief ADC calibration timeout.
 */
#define TIMEOUT_ADC_CALIB_MS (3000)

/**
 * @brief Plant MOSFET on resistance.
 */
#define SINV_MODEL_MOSFET_RON (4.6e-3f)

/**
 * @brief Body-diode on resistance.
 */
#define SINV_MODEL_DIODE_RON (0.01f)

/**
 * @brief Body-diode forward voltage.
 */
#define SINV_MODEL_DIODE_VF (0.5f)

//=================================================================================================
/**
 * @brief Common fallbacks: PGS Single-Phase Inverter Common Control.
 */

//=================================================================================================
/**
 * @brief Control Features.
 */

/**
 * @brief FDRC master switch. Select this switch to enable delayed insertion of the frequency-adaptive repetitive controller; clear it to keep FDRC disabled at every BUILD_LEVEL.
 */
#define SINV_ENABLE_REPETITIVE_CONTROL

/**
 * @brief Enable grid-voltage feedforward for closed-current-loop build levels.
 */
#define SINV_ENABLE_GRID_VOLTAGE_FEEDFORWARD

//=================================================================================================
/**
 * @brief Runtime.
 */

/**
 * @brief Allow ENABLE_OPERATION to advance through the complete CiA402 startup sequence.
 */
#define CIA402_CONFIG_ENABLE_SEQUENCE_SWITCH

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief DC-bus outer-loop proportional gain.
 */
#ifndef SINV_DC_BUS_LOOP_KP
#define SINV_DC_BUS_LOOP_KP (0.8f)
#endif // SINV_DC_BUS_LOOP_KP

/**
 * @brief DC-bus outer-loop integral gain per second.
 */
#ifndef SINV_DC_BUS_LOOP_KI
#define SINV_DC_BUS_LOOP_KI (12.0f)
#endif // SINV_DC_BUS_LOOP_KI

/**
 * @brief Symmetric outer-loop active-power command limit.
 */
#ifndef SINV_OUTER_LOOP_POWER_LIMIT_PU
#define SINV_OUTER_LOOP_POWER_LIMIT_PU (0.65f)
#endif // SINV_OUTER_LOOP_POWER_LIMIT_PU

/**
 * @brief Power and DC-bus outer-loop execution frequency.
 */
#ifndef SINV_OUTER_LOOP_FREQUENCY_HZ
#define SINV_OUTER_LOOP_FREQUENCY_HZ (1000.0f)
#endif // SINV_OUTER_LOOP_FREQUENCY_HZ

/**
 * @brief Active-power outer-loop proportional gain.
 */
#ifndef SINV_POWER_LOOP_KP
#define SINV_POWER_LOOP_KP (0.6f)
#endif // SINV_POWER_LOOP_KP

/**
 * @brief Active-power outer-loop integral gain per second.
 */
#ifndef SINV_POWER_LOOP_KI
#define SINV_POWER_LOOP_KI (8.0f)
#endif // SINV_POWER_LOOP_KI

/**
 * @brief Repetitive-control learning gain.
 */
#ifndef SINV_FDRC_LEARNING_GAIN
#define SINV_FDRC_LEARNING_GAIN (0.10f)
#endif // SINV_FDRC_LEARNING_GAIN

/**
 * @brief FDRC robustness-filter cutoff frequency.
 */
#ifndef SINV_FDRC_Q_FILTER_HZ
#define SINV_FDRC_Q_FILTER_HZ (1000.0f)
#endif // SINV_FDRC_Q_FILTER_HZ

/**
 * @brief Plant-delay compensation in controller samples.
 */
#ifndef SINV_FDRC_LEAD_STEPS
#define SINV_FDRC_LEAD_STEPS (3.0f)
#endif // SINV_FDRC_LEAD_STEPS

/**
 * @brief Current-error threshold above which RC learning is frozen.
 */
#ifndef SINV_FDRC_FREEZE_ERROR_PU
#define SINV_FDRC_FREEZE_ERROR_PU (0.05f)
#endif // SINV_FDRC_FREEZE_ERROR_PU

/**
 * @brief Settling time before repetitive control starts learning.
 */
#ifndef SINV_FDRC_ENABLE_DELAY_MS
#define SINV_FDRC_ENABLE_DELAY_MS (300)
#endif // SINV_FDRC_ENABLE_DELAY_MS

/**
 * @brief Enable the three sparse ADC slots used by the single-phase inverter SIL input ABI.
 */
#ifndef GMP_PIL_RX_MASK
#define GMP_PIL_RX_MASK (21)
#endif // GMP_PIL_RX_MASK

/**
 * @brief Enable two PWM slots and all sixteen monitor slots used by the single-phase inverter SIL output ABI.
 */
#ifndef GMP_PIL_TX_MASK
#define GMP_PIL_TX_MASK (4294901763)
#endif // GMP_PIL_TX_MASK

/**
 * @brief Nominal grid frequency in Hz.
 */
#ifndef CTRL_GRID_FREQUENCY
#define CTRL_GRID_FREQUENCY (50.0f)
#endif // CTRL_GRID_FREQUENCY

/**
 * @brief Minimum voltage magnitude used by the P/Q reference generator.
 */
#ifndef CTRL_GRID_VMIN_PU
#define CTRL_GRID_VMIN_PU (0.1f)
#endif // CTRL_GRID_VMIN_PU

/**
 * @brief SOGI PLL proportional gain.
 */
#ifndef CTRL_PLL_KP
#define CTRL_PLL_KP (10.0f)
#endif // CTRL_PLL_KP

/**
 * @brief SOGI PLL integral time constant in seconds.
 */
#ifndef CTRL_PLL_TI
#define CTRL_PLL_TI (0.02f)
#endif // CTRL_PLL_TI

/**
 * @brief PLL error-filter cutoff frequency in Hz.
 */
#ifndef CTRL_PLL_LPF_FC
#define CTRL_PLL_LPF_FC (20.0f)
#endif // CTRL_PLL_LPF_FC

/**
 * @brief PLL frequency-error lock threshold in PU.
 */
#ifndef CTRL_SPLL_EPSILON
#define CTRL_SPLL_EPSILON (0.005f)
#endif // CTRL_SPLL_EPSILON

/**
 * @brief Current deadband used by PWM dead-time compensation.
 */
#ifndef CTRL_CURRENT_DB_PU
#define CTRL_CURRENT_DB_PU (0.01f)
#endif // CTRL_CURRENT_DB_PU

/**
 * @brief Active-power command slew limit in PU/s.
 */
#ifndef CTRL_P_SLEW_PU_S
#define CTRL_P_SLEW_PU_S (5.0f)
#endif // CTRL_P_SLEW_PU_S

/**
 * @brief Reactive-power command slew limit in PU/s.
 */
#ifndef CTRL_Q_SLEW_PU_S
#define CTRL_Q_SLEW_PU_S (5.0f)
#endif // CTRL_Q_SLEW_PU_S

/**
 * @brief Peak current command limit in PU.
 */
#ifndef CTRL_CURRENT_LIMIT_PU
#define CTRL_CURRENT_LIMIT_PU (0.9f)
#endif // CTRL_CURRENT_LIMIT_PU

/**
 * @brief Power measurement low-pass cutoff frequency in Hz.
 */
#ifndef CTRL_PQ_LPF_FC
#define CTRL_PQ_LPF_FC (200.0f)
#endif // CTRL_PQ_LPF_FC

/**
 * @brief Minimum fundamental tracked by FDRC in Hz.
 */
#ifndef CTRL_FDRC_MIN_FREQ
#define CTRL_FDRC_MIN_FREQ (45.0f)
#endif // CTRL_FDRC_MIN_FREQ

/**
 * @brief QPR current-loop crossover target in Hz.
 */
#define SINV_CURRENT_LOOP_BANDWIDTH_HZ (600.0f)

/**
 * @brief Minimum operation-enabled transition delay.
 */
#define SINV_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief BUILD_LEVEL 1 sinusoidal H-bridge voltage amplitude.
 */
#define SINV_LEVEL1_VOLTAGE_REF_PU (0.35f)

/**
 * @brief BUILD_LEVEL 2 peak current command with a resistive load.
 */
#define SINV_LEVEL2_CURRENT_REF_PEAK_PU (0.20f)

/**
 * @brief BUILD_LEVEL 3 signed grid active-power command; positive exports power.
 */
#define SINV_LEVEL3_ACTIVE_POWER_REF_PU (0.10f)

/**
 * @brief BUILD_LEVEL 3 grid reactive-power command.
 */
#define SINV_LEVEL3_REACTIVE_POWER_REF_PU (0.0f)

/**
 * @brief BUILD_LEVEL 4 measured active-power closed-loop target.
 */
#define SINV_LEVEL4_ACTIVE_POWER_REF_PU (0.15f)

/**
 * @brief BUILD_LEVEL 5 physical DC bus voltage target.
 */
#define SINV_DC_BUS_REF_V (60.0f)

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 5)
#error BUILD_LEVEL_must_be_between_1_and_5
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_SINV_RC_SIMULATE_SETTINGS_H_
