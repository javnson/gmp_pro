#ifndef DPS_FSBB_F280049C_CTL_INTERFACE_H
#define DPS_FSBB_F280049C_CTL_INTERFACE_H

#include <ctl_main.h>
#include <xplt.peripheral.h>

GMP_STATIC_INLINE uint16_t fsbb_active_faults(void)
{
    uint16_t faults = FSBB_FAULT_NONE;

    if (GPIO_readPin(FSBB_OVERCURRENT) == 0U)
        faults |= FSBB_FAULT_HARDWARE_OVERCURRENT;
#if defined FSBB_ENABLE_VIN_SAMPLE
    if (adc_v_in.control_port.value < real2ctrl(FSBB_INPUT_VOLTAGE_MIN / CTRL_VOLTAGE_BASE))
        faults |= FSBB_FAULT_VIN_UNDERVOLTAGE;
    if (adc_v_in.control_port.value > real2ctrl(FSBB_INPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE))
        faults |= FSBB_FAULT_VIN_OVERVOLTAGE;
#endif
    if (adc_v_out.control_port.value > real2ctrl(FSBB_OUTPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE))
        faults |= FSBB_FAULT_VOUT_OVERVOLTAGE;
    if (adc_i_L.control_port.value > real2ctrl(FSBB_PROTECT_IL_MAX / CTRL_CURRENT_BASE))
        faults |= FSBB_FAULT_IL_POSITIVE_OVERCURRENT;
    if (adc_i_L.control_port.value < real2ctrl(FSBB_PROTECT_IL_MIN / CTRL_CURRENT_BASE))
        faults |= FSBB_FAULT_IL_NEGATIVE_OVERCURRENT;
    return faults;
}

GMP_STATIC_INLINE bool fsbb_overcurrent_latched(void)
{
    return ((EPWM_getOneShotTripZoneFlagStatus(FSBB_BUCK_PWM_BASE) |
             EPWM_getOneShotTripZoneFlagStatus(FSBB_BOOST_PWM_BASE)) & EPWM_TZ_OST_FLAG_OST1) != 0U;
}

GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    fsbb_enable_delay = 0;
    GPIO_writePin(FSBB_ENABLE_GATE, 0);
    EPWM_forceTripZoneEvent(FSBB_BUCK_PWM_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(FSBB_BOOST_PWM_BASE, EPWM_TZ_FORCE_EVENT_OST);
}

GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    ctl_fast_disable_output();
#if !defined ENABLE_GMP_DL_PIL_SIM
    clear_all_controllers();
    // Leave one complete zero-to-zero interval for the new CMPA shadows to load.
    fsbb_enable_delay = 2U;
#endif
}

GMP_STATIC_INLINE void ctl_input_callback(void)
{
#if !defined ENABLE_GMP_DL_PIL_SIM
#if defined FSBB_ENABLE_VIN_SAMPLE
    ctl_step_adc_channel(&adc_v_in, ADC_readResult(FSBB_VIN_ADC_RESULT_BASE, ADC_SOC_NUMBER0));
#else
    adc_v_in.control_port.value = real2ctrl(FSBB_LAUNCHXL_FIXED_VIN / CTRL_VOLTAGE_BASE);
#endif
    ctl_step_adc_channel(&adc_v_out, ADC_readResult(FSBB_VOUT_ADC_RESULT_BASE, ADC_SOC_NUMBER0));
    ctl_step_adc_channel(&adc_i_L, ADC_readResult(FSBB_IL_ADC_RESULT_BASE, ADC_SOC_NUMBER0));
    adc_i_load.control_port.value = CTL_CTRL_CONST_ZERO;
    g_fsbb_faults |= fsbb_active_faults();
    if (fsbb_overcurrent_latched())
        g_fsbb_faults |= FSBB_FAULT_HARDWARE_OVERCURRENT;
#endif
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
#if !defined ENABLE_GMP_DL_PIL_SIM
    if (g_fsbb_faults != FSBB_FAULT_NONE)
    {
        ctl_fast_disable_output();
        g_fsbb_output_enabled = 0;
        return;
    }

    EPWM_setCounterCompareValue(FSBB_BUCK_PWM_BASE, EPWM_COUNTER_COMPARE_A, ctl_get_fsbb_buck_cmp(&fsbb_mod));
    EPWM_setCounterCompareValue(FSBB_BOOST_PWM_BASE, EPWM_COUNTER_COMPARE_A, ctl_get_fsbb_boost_cmp(&fsbb_mod));

    if (cia402_sm.current_cmd != CIA402_CMD_ENABLE_OPERATION)
    {
        ctl_fast_disable_output();
        g_fsbb_output_enabled = 0;
        return;
    }

    if (fsbb_enable_delay != 0U && --fsbb_enable_delay == 0U)
    {
        EPWM_clearTripZoneFlag(FSBB_BUCK_PWM_BASE, EPWM_TZ_FLAG_OST);
        EPWM_clearTripZoneFlag(FSBB_BOOST_PWM_BASE, EPWM_TZ_FLAG_OST);
        if (fsbb_overcurrent_latched() || GPIO_readPin(FSBB_OVERCURRENT) == 0U)
        {
            g_fsbb_faults |= FSBB_FAULT_HARDWARE_OVERCURRENT;
            ctl_fast_disable_output();
            g_fsbb_output_enabled = 0;
            return;
        }
        GPIO_writePin(FSBB_ENABLE_GATE, 1);
    }
#endif
}

#endif
