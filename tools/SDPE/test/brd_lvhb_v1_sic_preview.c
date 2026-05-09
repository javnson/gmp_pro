// ==========================================================================
// SDPE DEBUG PREVIEW FOR INSTANCE: brd_lvhb_v1_sic
// PARADIGM: inverter_board
// IS_COMPOSITE: True
// INTENDED PRODUCTION PATH: hardware_presets/boards
// ==========================================================================

// ==========================================================================
// >>> HEADER FILE DOMAIN (.h) <<<
// ==========================================================================
#ifndef _FILE_SDPE_DBG_PREVIEW_BRD_LVHB_V1_SIC_H_
#define _FILE_SDPE_DBG_PREVIEW_BRD_LVHB_V1_SIC_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/* --- CONFIG MACROS (sdpe_config.h) --- */
/* --- Three-Phase Inverter Board Composite (DBG_BRD_LVHB_V1_SIC) Level Settings --- */


/**
 * @brief Component parameter
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_BOARD_NAME
#define SDPE_BRD_LVHB_V1_SIC_BOARD_NAME                "LVHB_V1_SiC"
#endif


/**
 * @brief Component parameter
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_MAX_CURRENT_A
#define SDPE_BRD_LVHB_V1_SIC_MAX_CURRENT_A             ((50.0f))
#endif


/**
 * @brief Component parameter
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_MAX_VOLTAGE_V
#define SDPE_BRD_LVHB_V1_SIC_MAX_VOLTAGE_V             ((800.0f))
#endif



/* Board Fault Thresholds Derived from Limits */
#define SDPE_BRD_LVHB_V1_SIC_OVER_CURRENT_FAULT_TH       (SDPE_BRD_LVHB_V1_SIC_MAX_CURRENT_A * 1.1f)
#define SDPE_BRD_LVHB_V1_SIC_OVER_VOLTAGE_FAULT_TH       (SDPE_BRD_LVHB_V1_SIC_MAX_VOLTAGE_V * 1.05f)
// --------------------------------------------------
// Sub-component: modulator (spwm_modulator)
// --------------------------------------------------
/* --- Three-Phase PWM Modulator (Two-Level) (DBG_BRD_LVHB_V1_SIC_modulator) Parameters --- */


/**
 * @brief Current deadband threshold for compensation
 * @unit PU
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_MODULATOR_CURR_DB_PU
#define SDPE_BRD_LVHB_V1_SIC_MODULATOR_CURR_DB_PU           ((0.02f))
#endif


/**
 * @brief Current hysteresis for compensation
 * @unit PU
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_MODULATOR_CURR_HYS_PU
#define SDPE_BRD_LVHB_V1_SIC_MODULATOR_CURR_HYS_PU          ((0.005f))
#endif



/* --- Derived Timer Count Macros --- */

/**
 * @brief Full scale PWM counter value (Center-aligned mode assumption)
 * @unit Counts
 */
#define SDPE_BRD_LVHB_V1_SIC_MODULATOR_FULL_SCALE_COUNTS   (GMP_SDPE_CALC_PWM_PERIOD_CENTER_ALIGNED(/* FROM_OUT: timer_clock_hz */, /* FROM_OUT: pwm_freq_hz */))

/**
 * @brief Dead-time compensation value
 * @unit Counts
 */
#define SDPE_BRD_LVHB_V1_SIC_MODULATOR_DEADBAND_COUNTS     (GMP_SDPE_CALC_PWM_DEADTIME_COUNTS(/* FROM_OUT: timer_clock_hz */, SDPE_GD_SYSTEM_DEADTIME_NS))

// --------------------------------------------------
// Sub-component: phase_current (current_shunt)
// --------------------------------------------------
/* --- Shunt-based Current Sensor with Op-Amp (DBG_BRD_LVHB_V1_SIC_phase_current) Parameters --- */


/**
 * @brief Shunt resistor value
 * @unit Ohms
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_R_SHUNT_OHM
#define SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_R_SHUNT_OHM          ((0.002f))
#endif


/**
 * @brief Total amplifier gain
 * @unit V/V
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_AMP_GAIN
#define SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_AMP_GAIN             ((20.0f))
#endif


/**
 * @brief Hardware bias voltage before ADC
 * @unit V
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_BIAS_V
#define SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_BIAS_V               ((1.65f))
#endif


/**
 * @brief Maximum continuous current
 * @unit Arms
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_MAX_CONT_ARMS
#define SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_MAX_CONT_ARMS        ((50.0f))
#endif


/**
 * @brief Maximum peak current
 * @unit Ap
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_MAX_PEAK_AP
#define SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_MAX_PEAK_AP          ((100.0f))
#endif


/**
 * @brief Overall accuracy class
 * @unit %
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_ACCURACY_CLASS_PCT
#define SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_ACCURACY_CLASS_PCT   ((1.0f))
#endif


/**
 * @brief Main amplifier or CSA chip part number
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_CHIP_PN
#define SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_CHIP_PN              "INA240A1"
#endif



/* --- Derived Sensitivity and Bias Macros --- */

/**
 * @brief Calculated hardware sensitivity for ADC scaling
 * @unit V/A
 */
#define SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_SENSITIVITY         (GMP_SDPE_CALC_SHUNT_SENSITIVITY(SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_R_SHUNT_OHM, SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_AMP_GAIN))

/**
 * @brief Hardware bias voltage for ADC offset
 * @unit V
 */
#define SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_BIAS                (GMP_SDPE_CALC_BIAS(SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_BIAS_V))

// --------------------------------------------------
// Sub-component: bus_voltage (voltage_divider)
// --------------------------------------------------
/* --- Resistor Voltage Divider Sensor (DBG_BRD_LVHB_V1_SIC_bus_voltage) Parameters --- */


/**
 * @brief High-side resistor value
 * @unit Ohms
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_R_HIGH_OHM
#define SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_R_HIGH_OHM           ((2000000.0f))
#endif


/**
 * @brief Low-side resistor value
 * @unit Ohms
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_R_LOW_OHM
#define SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_R_LOW_OHM            ((10000.0f))
#endif


/**
 * @brief Buffer or amplifier gain after divider
 * @unit V/V
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_AMP_GAIN
#define SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_AMP_GAIN             ((1.0f))
#endif


/**
 * @brief Hardware bias voltage before ADC
 * @unit V
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_BIAS_V
#define SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_BIAS_V               ((0.0f))
#endif


/**
 * @brief Maximum measurable continuous voltage
 * @unit V
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_MAX_VOLTAGE_V
#define SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_MAX_VOLTAGE_V        ((800.0f))
#endif


/**
 * @brief Overall accuracy class
 * @unit %
 */
#ifndef SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_ACCURACY_CLASS_PCT
#define SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_ACCURACY_CLASS_PCT   ((1.0f))
#endif



/* --- Derived Sensitivity and Bias Macros --- */

/**
 * @brief Calculated hardware sensitivity for ADC scaling
 * @unit V/V
 */
#define SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_SENSITIVITY         (GMP_SDPE_CALC_VOLTAGE_DIVIDER_SENSITIVITY(SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_R_HIGH_OHM, SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_R_LOW_OHM, SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_AMP_GAIN))

/**
 * @brief Hardware bias voltage for ADC offset
 * @unit V
 */
#define SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_BIAS                (GMP_SDPE_CALC_BIAS(SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_BIAS_V))

/* --- DECLARATION (sdpe_instances.h) --- */
// --------------------------------------------------
// Sub-component: modulator (spwm_modulator)
// --------------------------------------------------
// Three-Phase PWM Modulator (Two-Level) object declaration
spwm_modulator_t DBG_BRD_LVHB_V1_SIC_modulator;

// --------------------------------------------------
// Sub-component: phase_current (current_shunt)
// --------------------------------------------------
// Shunt-based Current Sensor with Op-Amp object declaration
tri_ptr_adc_channel_t DBG_BRD_LVHB_V1_SIC_phase_current;

// --------------------------------------------------
// Sub-component: bus_voltage (voltage_divider)
// --------------------------------------------------
// Resistor Voltage Divider Sensor object declaration
ptr_adc_channel_t DBG_BRD_LVHB_V1_SIC_bus_voltage;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SDPE_DBG_PREVIEW_BRD_LVHB_V1_SIC_H_


// ==========================================================================
// >>> SOURCE FILE DOMAIN (.c / inline step functions) <<<
// ==========================================================================

/* --- INITIALIZATION (sdpe_instances.c) --- */
// --------------------------------------------------
// Sub-component: modulator (spwm_modulator)
// --------------------------------------------------
// Initialize Three-Phase PWM Modulator (Two-Level)
    ctl_init_spwm_modulator(
        &DBG_BRD_LVHB_V1_SIC_modulator,
        (pwm_gt)SDPE_BRD_LVHB_V1_SIC_MODULATOR_FULL_SCALE_COUNTS,
        (pwm_gt)SDPE_BRD_LVHB_V1_SIC_MODULATOR_DEADBAND_COUNTS,
        (ctl_vector3_t*)/* FROM_OUT: current_feedback_ptr */,
        SDPE_BRD_LVHB_V1_SIC_MODULATOR_CURR_DB_PU,
        SDPE_BRD_LVHB_V1_SIC_MODULATOR_CURR_HYS_PU);

// --------------------------------------------------
// Sub-component: phase_current (current_shunt)
// --------------------------------------------------
// Initialize Shunt-based Current Sensor with Op-Amp (Ref: /* FROM_OUT: adc_vref */V, /* FROM_OUT: adc_res */-bit)
    ctl_init_tri_ptr_adc_channel(
        &DBG_BRD_LVHB_V1_SIC_phase_current,
        (adc_gt*)/* FROM_OUT: curr_raw_data_source */,
        ctl_gain_calc_generic(/* FROM_OUT: adc_vref */f, SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_SENSITIVITY, /* FROM_OUT: current_base */f),
        ctl_bias_calc_via_Vref_Vbias(/* FROM_OUT: adc_vref */f, SDPE_BRD_LVHB_V1_SIC_PHASE_CURRENT_BIAS),
        /* FROM_OUT: adc_res */, /* FROM_OUT: adc_iqn */);

// --------------------------------------------------
// Sub-component: bus_voltage (voltage_divider)
// --------------------------------------------------
// Initialize Resistor Voltage Divider Sensor (Ref: /* FROM_OUT: adc_vref */V, /* FROM_OUT: adc_res */-bit)
    ctl_init_ptr_adc_channel(
        &DBG_BRD_LVHB_V1_SIC_bus_voltage,
        (adc_gt*)/* FROM_OUT: vbus_raw_data_source */,
        ctl_gain_calc_generic(/* FROM_OUT: adc_vref */f, SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_SENSITIVITY, /* FROM_OUT: voltage_base */f),
        ctl_bias_calc_via_Vref_Vbias(/* FROM_OUT: adc_vref */f, SDPE_BRD_LVHB_V1_SIC_BUS_VOLTAGE_BIAS),
        /* FROM_OUT: adc_res */, /* FROM_OUT: adc_iqn */);

/* --- INPUT CALLBACK (sdpe_io_step.h) --- */
// --------------------------------------------------
// Sub-component: phase_current (current_shunt)
// --------------------------------------------------
ctl_step_tri_ptr_adc_channel(&DBG_BRD_LVHB_V1_SIC_phase_current);

// --------------------------------------------------
// Sub-component: bus_voltage (voltage_divider)
// --------------------------------------------------
ctl_step_ptr_adc_channel(&DBG_BRD_LVHB_V1_SIC_bus_voltage);

/* --- OUTPUT CALLBACK (sdpe_io_step.h) --- */
// --------------------------------------------------
// Sub-component: modulator (spwm_modulator)
// --------------------------------------------------
// Execute modulation algorithm
    ctl_step_svpwm_modulator(&DBG_BRD_LVHB_V1_SIC_modulator);
