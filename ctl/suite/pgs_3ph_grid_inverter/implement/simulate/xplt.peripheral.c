//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all definitions of peripheral objects in this file.
//
// User should implement the peripheral objects initialization in setup_peripheral function.
//
// This file is platform-related.
//

// GMP basic core header
#include <gmp_core.h>

// user main header
#include "user_main.h"

#include <xplt.peripheral.h>

//////////////////////////////////////////////////////////////////////////
// definitions of peripheral
//

ptr_adc_channel_t inv_adc[INV_ADC_SENSOR_NUMBER];
pwm_tri_channel_t pwm_out;

pwm_channel_t inv_pwm_out[3];

//////////////////////////////////////////////////////////////////////////
// peripheral setup function
//

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
    //
    // input channel
    //

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &inv_adc[INV_ADC_ID_IDC],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[INV_ADC_ID_IDC],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_GAIN, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &inv_adc[INV_ADC_ID_VDC],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[INV_ADC_ID_VDC],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_GAIN, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &inv_adc[INV_ADC_ID_UAB],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[INV_ADC_ID_UAB],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_GAIN, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &inv_adc[INV_ADC_ID_UBC],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[INV_ADC_ID_UBC],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_GAIN, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &inv_adc[INV_ADC_ID_IA],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[INV_ADC_ID_IA],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_GAIN, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &inv_adc[INV_ADC_ID_IB],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[INV_ADC_ID_IB],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_GAIN, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &inv_adc[INV_ADC_ID_IC],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[INV_ADC_ID_IC],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_GAIN, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    //
    // output channel
    //
    ctl_init_pwm_tri_channel(&pwm_out, 0, CTRL_PWM_CMP_MAX);

    ctl_init_pwm_channel(&inv_pwm_out[0], 0, CTRL_PWM_CMP_MAX);
    ctl_init_pwm_channel(&inv_pwm_out[1], 0, CTRL_PWM_CMP_MAX);
    ctl_init_pwm_channel(&inv_pwm_out[2], 0, CTRL_PWM_CMP_MAX);

    //
    // attach
    //
    ctl_attach_three_phase_inv(
        // inv controller
        &inv_ctrl,
        // udc, idc
        &inv_adc[INV_ADC_ID_VDC].control_port, &inv_adc[INV_ADC_ID_IDC].control_port,
        // iabc
        &inv_adc[INV_ADC_ID_IA].control_port, &inv_adc[INV_ADC_ID_IB].control_port,
        &inv_adc[INV_ADC_ID_IC].control_port,
        // uabc
        &inv_adc[INV_ADC_ID_UAB].control_port, &inv_adc[INV_ADC_ID_UBC].control_port, (void*)0);

    // open hardware switch
    // ctl_output_enable();
}
