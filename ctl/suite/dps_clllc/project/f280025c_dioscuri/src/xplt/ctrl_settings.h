/** @file ctrl_settings.h @brief Dioscuri CLLLC hardware/control settings. */
#ifndef DPS_CLLLC_CTRL_SETTINGS_H
#define DPS_CLLLC_CTRL_SETTINGS_H

#define BUILD_LEVEL                         (1)
#define CTRL_STARTUP_DELAY_MS               (100U)
#define CTRL_SYSTEM_TICK_HZ                 (1000U)
#define CLLLC_PWM_CYCLES_PER_CONTROL        (2U)

#define CLLLC_TIMER_CLOCK_HZ                (100000000.0f)
#define CLLLC_F_RESONANT_HZ                 (100000.0f)
#define CLLLC_F_MIN_HZ                      (75000.0f)
#define CLLLC_F_MAX_HZ                      (150000.0f)
#define CONTROLLER_FREQUENCY                (CLLLC_F_RESONANT_HZ / CLLLC_PWM_CYCLES_PER_CONTROL)
#define CLLLC_NOMINAL_PERIOD_TICKS          (1000U)
#define CLLLC_DEADBAND_S                    (200.0e-9f)
#define CLLLC_MAX_DAB_PHASE_PU              (0.25f)

#define CLLLC_LM_H                          (120.0e-6f)
#define CLLLC_LR_PRIMARY_H                  (20.0e-6f)
#define CLLLC_LR_SECONDARY_H                (20.0e-6f)
#define CLLLC_CR_PRIMARY_F                  (120.0e-9f)
#define CLLLC_CR_SECONDARY_F                (120.0e-9f)
#define CLLLC_TRANSFORMER_NS_NP             (1.0f)
#define CLLLC_COUT_F                        (440.0e-6f)
#define CLLLC_RLOAD_MIN_OHM                 (10.0f)
#define CLLLC_TANK_ESR_OHM                  (0.10f)

#define CTRL_ADC_VOLTAGE_REF                (3.3f)
#define CTRL_ADC_BITS                       (12U)
#define CTRL_VOLTAGE_BASE                   (120.0f)
#define CTRL_CURRENT_BASE                   (12.0f)
/* AMC1311 isolation amplifier and TMCS1133-B5A Hall sensor. */
#define CLLLC_VOLTAGE_SENSITIVITY_V_PER_V   (0.02705f)
#define CLLLC_VOLTAGE_BIAS_V                (0.0f)
#define CLLLC_CURRENT_SENSITIVITY_V_PER_A   (0.150f)
#define CLLLC_CURRENT_BIAS_V                (1.65f)

#define CLLLC_VOLTAGE_TARGET_PU             (0.40f)
#define CLLLC_CURRENT_LIMIT_PU              (0.50f)
#define CLLLC_OPEN_LOOP_COMMAND_PU          (0.10f)
#define CLLLC_CURRENT_LOOP_BW_HZ            (5000.0f)
#define CLLLC_VOLTAGE_LOOP_BW_HZ            (400.0f)
#define CLLLC_VOLTAGE_SLOPE_PU_S            (0.5f)
#define CLLLC_CURRENT_SLOPE_PU_S            (1.0f)

#define CLLLC_PRIMARY_V_ADC_BASE            ADCCRESULT_BASE
#define CLLLC_PRIMARY_V_ADC_SOC             ADC_SOC_NUMBER0
#define CLLLC_PRIMARY_I_ADC_BASE            ADCARESULT_BASE
#define CLLLC_PRIMARY_I_ADC_SOC             ADC_SOC_NUMBER0
#define CLLLC_SECONDARY_V_ADC_BASE          ADCARESULT_BASE
#define CLLLC_SECONDARY_V_ADC_SOC           ADC_SOC_NUMBER2
#define CLLLC_SECONDARY_I_ADC_BASE          ADCARESULT_BASE
#define CLLLC_SECONDARY_I_ADC_SOC           ADC_SOC_NUMBER1

/* Physical ePWM mapping from the Dioscuri schematic. */
#define CLLLC_PRIMARY_LEG_A_BASE             EPWM2_BASE
#define CLLLC_PRIMARY_LEG_B_BASE             EPWM1_BASE
#define CLLLC_SECONDARY_LEG_A_BASE           EPWM4_BASE
#define CLLLC_SECONDARY_LEG_B_BASE           EPWM3_BASE
#define CLLLC_ADC_TRIGGER_PWM_BASE           EPWM1_BASE

#define CLLLC_UART_BASE                      UART_USB_BASE
#define CLLLC_STATUS_LED_GPIO                40U

#define SPECIFY_ENABLE_ADC_CALIBRATE
#define TIMEOUT_ADC_CALIB_MS                 (3000U)
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC   (0)

#endif
