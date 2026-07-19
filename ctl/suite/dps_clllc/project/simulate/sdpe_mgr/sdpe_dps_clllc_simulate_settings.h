/**
 * @file sdpe_dps_clllc_simulate_settings.h
 * @brief SDPE project bindings for DPS CLLLC / DAB MATLAB Simulink SIL.
 * @note PC SIL runtime, BUILD_LEVEL and virtual advanced-PWM settings layered over the common CLLLC contract.
 */

#ifndef _PROJECT_SDPE_DPS_CLLLC_SIMULATE_SETTINGS_H_
#define _PROJECT_SDPE_DPS_CLLLC_SIMULATE_SETTINGS_H_

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
#include <sdpe_dps_clllc_common_settings.h>
/* Simulink/SIL platform bindings follow the shared CLLLC/DAB contract. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define DPS_CLLLC_SIM_SDPE_PROJECT_ID "dps_clllc_simulate"
#define DPS_CLLLC_SIM_SDPE_PROJECT_SUITE "dps_clllc"
#define DPS_CLLLC_SIM_SDPE_PROJECT_VERSION "1.0.0"
#define DPS_CLLLC_SIM_SDPE_PROJECT_UPDATED_AT "2026-07-19"

//=================================================================================================
/**
 * @brief Simulation.
 */

/**
 * @brief Enable PC/SIL-specific commissioning behavior.
 */
#define SPECIFY_PC_ENVIRONMENT

//=================================================================================================
/**
 * @brief Controller Mode.
 */

/**
 * @brief 1: open loop; 2: current loop; 3: voltage loop; 4: voltage/current parallel competition loop.
 *        Options: (1), (2), (3), (4)
 */
#define BUILD_LEVEL (1)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Virtual ePWM clock.
 */
#define CLLLC_TIMER_CLOCK_HZ (100000000.0f)

/**
 * @brief PWM periods per ADC/control update.
 */
#define CLLLC_PWM_CYCLES_PER_CONTROL (2)

/**
 * @brief Nominal controller frequency.
 */
#define CONTROLLER_FREQUENCY (CLLLC_F_RESONANT_HZ / CLLLC_PWM_CYCLES_PER_CONTROL)

/**
 * @brief Virtual advanced-PWM base period.
 */
#define CLLLC_NOMINAL_PERIOD_TICKS ((uint32_t)(CLLLC_TIMER_CLOCK_HZ / CLLLC_F_RESONANT_HZ))

/**
 * @brief PC scheduler tick.
 */
#define CTRL_SYSTEM_TICK_HZ (1000)

/**
 * @brief Short SIL CiA402 transition delay.
 */
#define CTRL_STARTUP_DELAY_MS (2)

/**
 * @brief Virtual ADC reference.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief Virtual ADC resolution.
 */
#define CTRL_ADC_BITS (12)

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be 1 (open loop), 2 (current loop), 3 (voltage loop), or 4 (parallel CC/CV)."
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_DPS_CLLLC_SIMULATE_SETTINGS_H_
