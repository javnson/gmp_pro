/**
 * @file sdpe_dps_clllc_dioscuri_settings.h
 * @brief SDPE project bindings for DPS CLLLC / DAB on F280025C Dioscuri.
 * @note F280025C clock, six-pair PWM routing, ADC, UART, scheduler timer and active-low SN74LVC8T245 gate-buffer bindings.
 */

#ifndef _PROJECT_SDPE_DPS_CLLLC_DIOSCURI_SETTINGS_H_
#define _PROJECT_SDPE_DPS_CLLLC_DIOSCURI_SETTINGS_H_

#include <ctl/hardware_preset/mcu_board/dioscuri_f280025c.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
#include <sdpe_dps_clllc_common_settings.h>
/* Dioscuri platform bindings follow the shared CLLLC/DAB contract. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define DPS_CLLLC_DIOSCURI_SDPE_PROJECT_ID "dps_clllc_f280025c_dioscuri"
#define DPS_CLLLC_DIOSCURI_SDPE_PROJECT_SUITE "dps_clllc"
#define DPS_CLLLC_DIOSCURI_SDPE_PROJECT_VERSION "1.0.0"
#define DPS_CLLLC_DIOSCURI_SDPE_PROJECT_UPDATED_AT "2026-07-20"

//=================================================================================================
/**
 * @brief Debug Interfaces.
 */

/**
 * @brief Enable communication-based PIL execution instead of direct hardware control.
 */
// #define ENABLE_GMP_DL_PIL_SIM

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
 * @brief CLLLC PWM Routing.
 */

/**
 * @brief Primary PRI_P / ControlPort1 bridge leg: EPWM4 on the Dioscuri schematic. Changing bridge order changes the defined positive current direction.
 *        Options: DIOSCURI_PWM_PAIR1_BASE, DIOSCURI_PWM_PAIR2_BASE, DIOSCURI_PWM_PAIR3_BASE, DIOSCURI_PWM_PAIR4_BASE, DIOSCURI_PWM_PAIR5_BASE, DIOSCURI_PWM_PAIR6_BASE
 */
#define CLLLC_PRIMARY_LEG_A_BASE DIOSCURI_PWM_PAIR4_BASE

/**
 * @brief Primary PRI_N / ControlPort2 bridge leg: EPWM7 on the Dioscuri schematic.
 *        Options: DIOSCURI_PWM_PAIR1_BASE, DIOSCURI_PWM_PAIR2_BASE, DIOSCURI_PWM_PAIR3_BASE, DIOSCURI_PWM_PAIR4_BASE, DIOSCURI_PWM_PAIR5_BASE, DIOSCURI_PWM_PAIR6_BASE
 */
#define CLLLC_PRIMARY_LEG_B_BASE DIOSCURI_PWM_PAIR6_BASE

/**
 * @brief Secondary SEC_P / ControlPort4 bridge leg: EPWM1 on the Dioscuri schematic.
 *        Options: DIOSCURI_PWM_PAIR1_BASE, DIOSCURI_PWM_PAIR2_BASE, DIOSCURI_PWM_PAIR3_BASE, DIOSCURI_PWM_PAIR4_BASE, DIOSCURI_PWM_PAIR5_BASE, DIOSCURI_PWM_PAIR6_BASE
 */
#define CLLLC_SECONDARY_LEG_A_BASE DIOSCURI_PWM_PAIR2_BASE

/**
 * @brief Secondary SEC_N / ControlPort3 bridge leg: EPWM2 on the Dioscuri schematic.
 *        Options: DIOSCURI_PWM_PAIR1_BASE, DIOSCURI_PWM_PAIR2_BASE, DIOSCURI_PWM_PAIR3_BASE, DIOSCURI_PWM_PAIR4_BASE, DIOSCURI_PWM_PAIR5_BASE, DIOSCURI_PWM_PAIR6_BASE
 */
#define CLLLC_SECONDARY_LEG_B_BASE DIOSCURI_PWM_PAIR1_BASE

//=================================================================================================
/**
 * @brief CLLLC PWM Synchronization.
 */

/**
 * @brief Master time base that emits synchronization at counter zero. It must be one of the four selected CLLLC legs.
 *        Options: DIOSCURI_PWM_PAIR1_BASE, DIOSCURI_PWM_PAIR2_BASE, DIOSCURI_PWM_PAIR3_BASE, DIOSCURI_PWM_PAIR4_BASE, DIOSCURI_PWM_PAIR5_BASE, DIOSCURI_PWM_PAIR6_BASE
 */
#define CLLLC_SYNC_MASTER_PWM_BASE DIOSCURI_PWM_PAIR1_BASE

/**
 * @brief Slave sync-input source corresponding to the selected master ePWM peripheral.
 *        Options: EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM2, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM3, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM4, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM5, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM7
 */
#define CLLLC_SYNC_IN_SOURCE EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1

//=================================================================================================
/**
 * @brief ADC Trigger.
 */

/**
 * @brief PWM that generates ADC SOCA. Keep this consistent with the SOC trigger selected in SysConfig.
 *        Options: DIOSCURI_PWM_PAIR1_BASE, DIOSCURI_PWM_PAIR2_BASE, DIOSCURI_PWM_PAIR3_BASE, DIOSCURI_PWM_PAIR4_BASE, DIOSCURI_PWM_PAIR5_BASE, DIOSCURI_PWM_PAIR6_BASE
 */
#define CLLLC_ADC_TRIGGER_PWM_BASE DIOSCURI_PWM_PAIR1_BASE

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief F280025C ePWM and CPU Timer clock.
 */
#define CLLLC_TIMER_CLOCK_HZ (100000000.0f)

/**
 * @brief PWM cycles between ADC/control interrupts.
 */
#define CLLLC_PWM_CYCLES_PER_CONTROL (2)

/**
 * @brief Nominal controller frequency used for offline tuning.
 */
#define CONTROLLER_FREQUENCY (CLLLC_F_RESONANT_HZ / CLLLC_PWM_CYCLES_PER_CONTROL)

/**
 * @brief Nominal 100 kHz period in ePWM timer ticks.
 */
#define CLLLC_NOMINAL_PERIOD_TICKS ((uint32_t)(CLLLC_TIMER_CLOCK_HZ / CLLLC_F_RESONANT_HZ))

/**
 * @brief PWM-independent scheduler tick generated by CPU Timer0.
 */
#define CTRL_SYSTEM_TICK_HZ (1000)

/**
 * @brief Minimum CiA402 delay per startup transition.
 */
#define CTRL_STARTUP_DELAY_MS (100)

/**
 * @brief F280025C ADC reference voltage.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief ADC resolution in bits.
 */
#define CTRL_ADC_BITS (12)

/**
 * @brief Primary AMC1311 result register base.
 */
#define CLLLC_PRIMARY_V_ADC_BASE DIOSCURI_PRIMARY_VOLTAGE_RESULT_BASE

/**
 * @brief Primary AMC1311 ADC SOC.
 */
#define CLLLC_PRIMARY_V_ADC_SOC DIOSCURI_PRIMARY_VOLTAGE_SOC

/**
 * @brief Primary TMCS1133B5A result register base.
 */
#define CLLLC_PRIMARY_I_ADC_BASE DIOSCURI_PRIMARY_CURRENT_RESULT_BASE

/**
 * @brief Primary TMCS1133B5A ADC SOC.
 */
#define CLLLC_PRIMARY_I_ADC_SOC DIOSCURI_PRIMARY_CURRENT_SOC

/**
 * @brief Secondary AMC1311 result register base.
 */
#define CLLLC_SECONDARY_V_ADC_BASE DIOSCURI_SECONDARY_VOLTAGE_RESULT_BASE

/**
 * @brief Secondary AMC1311 ADC SOC.
 */
#define CLLLC_SECONDARY_V_ADC_SOC DIOSCURI_SECONDARY_VOLTAGE_SOC

/**
 * @brief Secondary TMCS1133B5A result register base.
 */
#define CLLLC_SECONDARY_I_ADC_BASE DIOSCURI_SECONDARY_CURRENT_RESULT_BASE

/**
 * @brief Secondary TMCS1133B5A ADC SOC.
 */
#define CLLLC_SECONDARY_I_ADC_SOC DIOSCURI_SECONDARY_CURRENT_SOC

/**
 * @brief GPIO connected to SN74LVC8T245 OE#.
 */
#define CLLLC_GATE_ENABLE_GPIO DIOSCURI_GATE_ENABLE_GPIO

/**
 * @brief Low level enables the PWM buffer.
 */
#define CLLLC_GATE_ENABLE_ACTIVE_LEVEL DIOSCURI_GATE_ENABLE_LEVEL

/**
 * @brief High level disables the PWM buffer.
 */
#define CLLLC_GATE_DISABLE_LEVEL DIOSCURI_GATE_DISABLE_LEVEL

/**
 * @brief CPU timer used for the GMP system tick.
 */
#define CLLLC_SCHEDULER_TIMER_BASE DIOSCURI_TICK_TIMER_BASE

/**
 * @brief USB serial Datalink peripheral.
 */
#define CLLLC_UART_BASE DIOSCURI_UART_BASE

/**
 * @brief Dioscuri status LED GPIO.
 */
#define CLLLC_STATUS_LED_GPIO (40)

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be 1 (open loop), 2 (current loop), 3 (voltage loop), or 4 (parallel CC/CV)."
#endif
/* PWM-resource checks live in xplt.peripheral.c, after SysConfig board.h symbols are available. */

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_DPS_CLLLC_DIOSCURI_SETTINGS_H_
