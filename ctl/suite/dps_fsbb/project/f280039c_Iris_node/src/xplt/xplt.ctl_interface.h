/**
 * @file xplt.ctl_interface.h
 * @brief F280039C hardware callbacks for the FSBB controller.
 */

#ifndef _FILE_XPLT_CTL_INTERFACE_H_
#define _FILE_XPLT_CTL_INTERFACE_H_

#include <xplt.peripheral.h>

#ifdef __cplusplus
extern "C"
{
#endif

void GPIO_WritePin(uint16_t gpioNumber, uint16_t outVal);

GMP_STATIC_INLINE uint16_t ctl_fsbb_active_faults(void)
{
    uint16_t faults = FSBB_FAULT_NONE;

#if defined FSBB_ENABLE_VIN_SAMPLE
    if (adc_v_in.control_port.value < float2ctrl(FSBB_INPUT_VOLTAGE_MIN / CTRL_VOLTAGE_BASE))
        faults |= FSBB_FAULT_VIN_UNDERVOLTAGE;
    if (adc_v_in.control_port.value > float2ctrl(FSBB_INPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE))
        faults |= FSBB_FAULT_VIN_OVERVOLTAGE;
#endif
    if (adc_v_out.control_port.value > float2ctrl(FSBB_OUTPUT_VOLTAGE_MAX / CTRL_VOLTAGE_BASE))
        faults |= FSBB_FAULT_VOUT_OVERVOLTAGE;
    if (adc_i_L.control_port.value > float2ctrl(FSBB_PROTECT_IL_MAX / CTRL_CURRENT_BASE))
        faults |= FSBB_FAULT_IL_POSITIVE_OVERCURRENT;
    if (adc_i_L.control_port.value < float2ctrl(FSBB_PROTECT_IL_MIN / CTRL_CURRENT_BASE))
        faults |= FSBB_FAULT_IL_NEGATIVE_OVERCURRENT;
#if defined FSBB_ENABLE_IOUT_SAMPLE
    if ((adc_i_load.control_port.value > float2ctrl(FSBB_OUTPUT_CURRENT_LIM / CTRL_CURRENT_BASE)) ||
        (adc_i_load.control_port.value < -float2ctrl(FSBB_OUTPUT_CURRENT_LIM / CTRL_CURRENT_BASE)))
        faults |= FSBB_FAULT_IOUT_OVERCURRENT;
#endif

    return faults;
}

GMP_STATIC_INLINE void ctl_input_callback(void)
{
#if defined FSBB_ENABLE_VIN_SAMPLE
    ctl_step_adc_channel(&adc_v_in, ADC_readResult(FSBB_VIN_ADC_BASE, FSBB_VIN));
#else
    adc_v_in.control_port.value = float2ctrl(FSBB_INPUT_VOLTAGE_NOMINAL / CTRL_VOLTAGE_BASE);
#endif
    ctl_step_adc_channel(&adc_v_out, ADC_readResult(FSBB_VOUT_ADC_BASE, FSBB_VOUT));
    ctl_step_adc_channel(&adc_i_L, ADC_readResult(FSBB_IL_ADC_BASE, FSBB_IL));
#if defined FSBB_ENABLE_IOUT_SAMPLE
    ctl_step_adc_channel(&adc_i_load, ADC_readResult(FSBB_IOUT_ADC_BASE, FSBB_IOUT));
#else
    adc_i_load.control_port.value = float2ctrl(0.0f);
#endif

    if (!flag_enable_adc_calibrator)
        g_fsbb_faults |= ctl_fsbb_active_faults();
}

GMP_STATIC_INLINE uint16_t ctl_fsbb_dac_value(ctrl_gt value)
{
    ctrl_gt bounded = ctl_sat(value, float2ctrl(1.0f), -float2ctrl(1.0f));
    return (uint16_t)((bounded + float2ctrl(1.0f)) * 2047.5f);
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
#if !defined ENABLE_GMP_DL_PIL_SIM
    if (g_fsbb_faults != FSBB_FAULT_NONE)
    {
        EPWM_forceTripZoneEvent(PHASE_BUCK_BASE, EPWM_TZ_FORCE_EVENT_OST);
        EPWM_forceTripZoneEvent(PHASE_BOOST_BASE, EPWM_TZ_FORCE_EVENT_OST);
        GPIO_WritePin(PWM_ENABLE_PORT, 0);
        g_fsbb_output_enabled = 0;
        return;
    }

    EPWM_setCounterCompareValue(PHASE_BUCK_BASE, EPWM_COUNTER_COMPARE_A, ctl_get_fsbb_buck_cmp(&fsbb_mod));
    EPWM_setCounterCompareValue(PHASE_BOOST_BASE, EPWM_COUNTER_COMPARE_A, ctl_get_fsbb_boost_cmp(&fsbb_mod));

#if BUILD_LEVEL >= 1
    DAC_setShadowValue(IRIS_DACA_BASE, ctl_fsbb_dac_value(adc_v_out.control_port.value));
    DAC_setShadowValue(IRIS_DACB_BASE, ctl_fsbb_dac_value(dcdc_core.v_out_formal));
#endif
#endif
}

GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    EPWM_clearTripZoneFlag(PHASE_BUCK_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(PHASE_BOOST_BASE, EPWM_TZ_FORCE_EVENT_OST);
    clear_all_controllers();
    GPIO_WritePin(PWM_ENABLE_PORT, 1);
    GPIO_WritePin(CONTROLLER_LED, 0);
}

GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    EPWM_forceTripZoneEvent(PHASE_BUCK_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_BOOST_BASE, EPWM_TZ_FORCE_EVENT_OST);
    GPIO_WritePin(PWM_ENABLE_PORT, 0);
    GPIO_WritePin(CONTROLLER_LED, 1);
}

typedef enum _tag_dcdc_adc_index_items
{
    DCDC_ADC_ID_VIN = 0,
    DCDC_ADC_ID_VOUT = 1,
    DCDC_ADC_ID_IL = 2,
    DCDC_ADC_ID_ILOAD = 3,
    DCDC_ADC_SENSOR_NUMBER = 4
} dcdc_adc_index_items;

GMP_STATIC_INLINE void ctl_input_callback_pil(const gmp_sim_rx_buf_t* rx)
{
    ctl_step_adc_channel(&adc_v_in, rx->adc_result[DCDC_ADC_ID_VIN]);
    ctl_step_adc_channel(&adc_v_out, rx->adc_result[DCDC_ADC_ID_VOUT]);
    ctl_step_adc_channel(&adc_i_L, rx->adc_result[DCDC_ADC_ID_IL]);
    ctl_step_adc_channel(&adc_i_load, rx->adc_result[DCDC_ADC_ID_ILOAD]);
}

GMP_STATIC_INLINE void ctl_output_callback_pil(gmp_sim_tx_buf_t* tx)
{
    tx->pwm_cmp[0] = ctl_get_fsbb_buck_cmp(&fsbb_mod);
    tx->pwm_cmp[1] = ctl_get_fsbb_boost_cmp(&fsbb_mod);
    tx->monitor[0] = adc_v_out.control_port.value;
    tx->monitor[1] = dcdc_core.v_out_formal;
}

#ifdef __cplusplus
}
#endif

#endif // _FILE_XPLT_CTL_INTERFACE_H_
