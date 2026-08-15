/**
 * @file ctrl_settings.h
 * @brief SDPE project bindings for PGS SINV RC MATLAB/Simulink SIL.
 * @note Windows SIL platform, plant and sensing settings for all SINV commissioning models.
 */

#ifndef _PROJECT_CTRL_SETTINGS_H_
#define _PROJECT_CTRL_SETTINGS_H_

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
#define PGS_SINV_RC_SIM_SDPE_PROJECT_UPDATED_AT "2026-08-15"

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
#define BUILD_LEVEL (2)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief SIL controller and PWM update frequency.
 */
#define CONTROLLER_FREQUENCY real2param(20e3)

/**
 * @brief Plant switching frequency.
 */
#define SINV_PWM_FREQUENCY_HZ real2param(20e3)

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
#define CTRL_ADC_VOLTAGE_REF real2param(3.3)

/**
 * @brief Rated and nominal DC-bus voltage.
 */
#define CTRL_DCBUS_VOLTAGE real2param(60.0)

/**
 * @brief Nominal grid/load RMS voltage.
 */
#define CTRL_GRID_VOLTAGE_RMS real2param(24.0)

/**
 * @brief Rated RMS AC current.
 */
#define CTRL_RATED_CURRENT_RMS real2param(10.0)

/**
 * @brief Peak voltage PU base.
 */
#define CTRL_VOLTAGE_BASE real2param(34.0)

/**
 * @brief Peak current PU base.
 */
#define CTRL_CURRENT_BASE real2param(14.14)

/**
 * @brief AC filter series inductance.
 */
#define CTRL_AC_INDUCTANCE real2param(480e-6)

/**
 * @brief AC filter series resistance.
 */
#define CTRL_AC_RESISTANCE real2param(0.10)

/**
 * @brief AC filter shunt capacitance.
 */
#define SINV_FILTER_CAPACITANCE_F real2param(22e-6)

/**
 * @brief Filter capacitor ESR.
 */
#define SINV_FILTER_CAP_ESR_OHM real2param(0.10)

/**
 * @brief DC-link capacitance.
 */
#define SINV_DC_CAPACITANCE_F real2param(2200e-6)

/**
 * @brief Resistive load for levels 1 and 2.
 */
#define SINV_RLOAD_OHM real2param(12.0)

/**
 * @brief DC-side load for level 5. At 60 V this draws 120 W, within the configured converter current rating.
 */
#define SINV_RECTIFIER_RLOAD_OHM real2param(30.0)

/**
 * @brief AC voltage sensor sensitivity in V/V.
 */
#define CTRL_AC_VOLTAGE_SENSITIVITY real2param(0.020)

/**
 * @brief AC voltage ADC bias.
 */
#define CTRL_AC_VOLTAGE_BIAS real2param(1.65)

/**
 * @brief AC current sensor sensitivity in V/A.
 */
#define CTRL_AC_CURRENT_SENSITIVITY real2param(0.150)

/**
 * @brief AC current ADC bias.
 */
#define CTRL_AC_CURRENT_BIAS real2param(1.65)

/**
 * @brief DC bus voltage sensor sensitivity in V/V.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY real2param(0.040)

/**
 * @brief DC bus voltage ADC bias.
 */
#define CTRL_DC_VOLTAGE_BIAS real2param(0.0)

/**
 * @brief DC bus overvoltage threshold.
 */
#define CTRL_PROT_VBUS_MAX real2param(90.0)

/**
 * @brief Fast AC peak-current threshold.
 */
#define CTRL_PROT_IAC_PEAK_MAX real2param(18.0)

/**
 * @brief Controller divergence threshold; BUILD_LEVEL 5 masks it only during passive-rectifier takeover.
 */
#define CTRL_PROT_VCTRL_MAX_PU real2param(1.5)

/**
 * @brief Minimum precharged DC bus accepted by startup.
 */
#define CTRL_DCBUS_READY_MIN real2param(25.0)

/**
 * @brief Maximum DC bus accepted by startup.
 */
#define CTRL_DCBUS_READY_MAX real2param(90.0)

/**
 * @brief ADC calibration timeout.
 */
#define TIMEOUT_ADC_CALIB_MS (3000)

/**
 * @brief Plant MOSFET on resistance.
 */
#define SINV_MODEL_MOSFET_RON real2param(4.6e-3)

/**
 * @brief Body-diode on resistance.
 */
#define SINV_MODEL_DIODE_RON real2param(0.01)

/**
 * @brief Body-diode forward voltage.
 */
#define SINV_MODEL_DIODE_VF real2param(0.5)

/**
 * @brief Filter-capacitor equivalent parallel resistance used by the plant model.
 */
#define SINV_FILTER_CAP_EPR_OHM real2param(100e3)

/**
 * @brief DC-link capacitor equivalent series resistance.
 */
#define SINV_DC_CAP_ESR_OHM real2param(0.05)

/**
 * @brief DC-link capacitor equivalent parallel resistance.
 */
#define SINV_DC_CAP_EPR_OHM real2param(10e3)

/**
 * @brief Analog sensor low-pass cutoff used by all simulated voltage and current channels.
 */
#define SINV_SENSOR_FILTER_HZ real2param(1500.0)

/**
 * @brief Simulated shunt resistance for DC-link, capacitor and grid-current channels.
 */
#define SINV_CURRENT_SHUNT_OHM real2param(0.02)

/**
 * @brief Simulated current-sense amplifier gain.
 */
#define SINV_CURRENT_AMPLIFIER_GAIN real2param(20.0)

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
#define SINV_DC_BUS_LOOP_KP real2param(0.8)
#endif // SINV_DC_BUS_LOOP_KP

/**
 * @brief DC-bus outer-loop integral gain per second.
 */
#ifndef SINV_DC_BUS_LOOP_KI
#define SINV_DC_BUS_LOOP_KI real2param(12.0)
#endif // SINV_DC_BUS_LOOP_KI

/**
 * @brief Symmetric outer-loop active-power command limit.
 */
#ifndef SINV_OUTER_LOOP_POWER_LIMIT_PU
#define SINV_OUTER_LOOP_POWER_LIMIT_PU real2param(0.65)
#endif // SINV_OUTER_LOOP_POWER_LIMIT_PU

/**
 * @brief Power and DC-bus outer-loop execution frequency.
 */
#ifndef SINV_OUTER_LOOP_FREQUENCY_HZ
#define SINV_OUTER_LOOP_FREQUENCY_HZ real2param(1000.0)
#endif // SINV_OUTER_LOOP_FREQUENCY_HZ

/**
 * @brief Active-power outer-loop proportional gain.
 */
#ifndef SINV_POWER_LOOP_KP
#define SINV_POWER_LOOP_KP real2param(0.6)
#endif // SINV_POWER_LOOP_KP

/**
 * @brief Active-power outer-loop integral gain per second.
 */
#ifndef SINV_POWER_LOOP_KI
#define SINV_POWER_LOOP_KI real2param(8.0)
#endif // SINV_POWER_LOOP_KI

/**
 * @brief Repetitive-control learning gain.
 */
#ifndef SINV_FDRC_LEARNING_GAIN
#define SINV_FDRC_LEARNING_GAIN real2param(0.10)
#endif // SINV_FDRC_LEARNING_GAIN

/**
 * @brief FDRC robustness-filter cutoff frequency.
 */
#ifndef SINV_FDRC_Q_FILTER_HZ
#define SINV_FDRC_Q_FILTER_HZ real2param(1000.0)
#endif // SINV_FDRC_Q_FILTER_HZ

/**
 * @brief Plant-delay compensation in controller samples.
 */
#ifndef SINV_FDRC_LEAD_STEPS
#define SINV_FDRC_LEAD_STEPS real2param(3.0)
#endif // SINV_FDRC_LEAD_STEPS

/**
 * @brief Current-error threshold above which RC learning is frozen.
 */
#ifndef SINV_FDRC_FREEZE_ERROR_PU
#define SINV_FDRC_FREEZE_ERROR_PU real2param(0.05)
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
#define CTRL_GRID_FREQUENCY real2param(50.0)
#endif // CTRL_GRID_FREQUENCY

/**
 * @brief Minimum voltage magnitude used by the P/Q reference generator.
 */
#ifndef CTRL_GRID_VMIN_PU
#define CTRL_GRID_VMIN_PU real2param(0.1)
#endif // CTRL_GRID_VMIN_PU

/**
 * @brief SOGI PLL proportional gain.
 */
#ifndef CTRL_PLL_KP
#define CTRL_PLL_KP real2param(10.0)
#endif // CTRL_PLL_KP

/**
 * @brief SOGI PLL integral time constant in seconds.
 */
#ifndef CTRL_PLL_TI
#define CTRL_PLL_TI real2param(0.02)
#endif // CTRL_PLL_TI

/**
 * @brief PLL error-filter cutoff frequency in Hz.
 */
#ifndef CTRL_PLL_LPF_FC
#define CTRL_PLL_LPF_FC real2param(20.0)
#endif // CTRL_PLL_LPF_FC

/**
 * @brief PLL frequency-error lock threshold in PU.
 */
#ifndef CTRL_SPLL_EPSILON
#define CTRL_SPLL_EPSILON real2param(0.005)
#endif // CTRL_SPLL_EPSILON

/**
 * @brief Current deadband used by PWM dead-time compensation.
 */
#ifndef CTRL_CURRENT_DB_PU
#define CTRL_CURRENT_DB_PU real2param(0.01)
#endif // CTRL_CURRENT_DB_PU

/**
 * @brief Active-power command slew limit in PU/s.
 */
#ifndef CTRL_P_SLEW_PU_S
#define CTRL_P_SLEW_PU_S real2param(5.0)
#endif // CTRL_P_SLEW_PU_S

/**
 * @brief Reactive-power command slew limit in PU/s.
 */
#ifndef CTRL_Q_SLEW_PU_S
#define CTRL_Q_SLEW_PU_S real2param(5.0)
#endif // CTRL_Q_SLEW_PU_S

/**
 * @brief Peak current command limit in PU.
 */
#ifndef CTRL_CURRENT_LIMIT_PU
#define CTRL_CURRENT_LIMIT_PU real2param(0.9)
#endif // CTRL_CURRENT_LIMIT_PU

/**
 * @brief Power measurement low-pass cutoff frequency in Hz.
 */
#ifndef CTRL_PQ_LPF_FC
#define CTRL_PQ_LPF_FC real2param(200.0)
#endif // CTRL_PQ_LPF_FC

/**
 * @brief Minimum fundamental tracked by FDRC in Hz.
 */
#ifndef CTRL_FDRC_MIN_FREQ
#define CTRL_FDRC_MIN_FREQ real2param(45.0)
#endif // CTRL_FDRC_MIN_FREQ

/**
 * @brief QPR current-loop crossover target in Hz.
 */
#define SINV_CURRENT_LOOP_BANDWIDTH_HZ real2param(600.0)

/**
 * @brief Minimum operation-enabled transition delay.
 */
#define SINV_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief BUILD_LEVEL 1 sinusoidal H-bridge voltage amplitude.
 */
#define SINV_LEVEL1_VOLTAGE_REF_PU real2param(0.35)

/**
 * @brief BUILD_LEVEL 2 peak current command with a resistive load.
 */
#define SINV_LEVEL2_CURRENT_REF_PEAK_PU real2param(0.20)

/**
 * @brief BUILD_LEVEL 3 signed grid active-power command; positive exports power.
 */
#define SINV_LEVEL3_ACTIVE_POWER_REF_PU real2param(0.10)

/**
 * @brief BUILD_LEVEL 3 grid reactive-power command.
 */
#define SINV_LEVEL3_REACTIVE_POWER_REF_PU real2param(0.0)

/**
 * @brief BUILD_LEVEL 4 measured active-power closed-loop target.
 */
#define SINV_LEVEL4_ACTIVE_POWER_REF_PU real2param(0.15)

/**
 * @brief BUILD_LEVEL 5 physical DC bus voltage target.
 */
#define SINV_DC_BUS_REF_V real2param(60.0)

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 5)
#error BUILD_LEVEL_must_be_between_1_and_5
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_CTRL_SETTINGS_H_
