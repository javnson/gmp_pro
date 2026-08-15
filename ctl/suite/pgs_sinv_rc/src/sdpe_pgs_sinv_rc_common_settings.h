/**
 * @file sdpe_pgs_sinv_rc_common_settings.h
 * @brief SDPE project bindings for PGS Single-Phase Inverter Common Control.
 * @note Platform-independent control contract shared by all pgs_sinv_rc projects.
 */

#ifndef _PROJECT_SDPE_PGS_SINV_RC_COMMON_SETTINGS_H_
#define _PROJECT_SDPE_PGS_SINV_RC_COMMON_SETTINGS_H_

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
// SDPE extension point: add after_extern_open code in the Project Requirement Code page if needed.

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_SINV_RC_COMMON_SDPE_PROJECT_ID "pgs_sinv_rc_common"
#define PGS_SINV_RC_COMMON_SDPE_PROJECT_SUITE "pgs_sinv_rc"
#define PGS_SINV_RC_COMMON_SDPE_PROJECT_VERSION "1.0.0"
#define PGS_SINV_RC_COMMON_SDPE_PROJECT_UPDATED_AT "2026-08-15"

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
// SDPE extension point: add before_footer code in the Project Requirement Code page if needed.

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_SINV_RC_COMMON_SETTINGS_H_
