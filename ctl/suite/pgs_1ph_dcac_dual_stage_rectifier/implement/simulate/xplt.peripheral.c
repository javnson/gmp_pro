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

ptr_adc_channel_t sinv_adc[SINV_ADC_SENSOR_NUMBER];

pwm_channel_t sinv_pwm_out[3];

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
        &sinv_adc[SINV_ADC_ID_IDC],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[SINV_ADC_ID_IDC],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_GAIN, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &sinv_adc[SINV_ADC_ID_VDC],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[SINV_ADC_ID_VDC],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_GAIN, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &sinv_adc[SINV_ADC_ID_IL],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[SINV_ADC_ID_IL],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_GAIN, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &sinv_adc[SINV_ADC_ID_IC],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[SINV_ADC_ID_IC],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_GAIN, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &sinv_adc[SINV_ADC_ID_IG],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[SINV_ADC_ID_IG],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_GAIN, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &sinv_adc[SINV_ADC_ID_VG],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[SINV_ADC_ID_VG],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_GAIN, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

        ctl_init_ptr_adc_channel(
        // ptr_adc object
        &sinv_adc[SINV_ADC_ID_BUCK_IL],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[SINV_ADC_ID_BUCK_IL],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_GAIN, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_CURRENT_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &sinv_adc[SINV_ADC_ID_BUCK_VIN],
        // pointer to ADC raw data
            &simulink_rx_buffer.adc_result[SINV_ADC_ID_BUCK_VIN],
        // ADC Channel settings.
            ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_GAIN, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);

    ctl_init_ptr_adc_channel(
        // ptr_adc object
        &sinv_adc[SINV_ADC_ID_BUCK_VOUT],
        // pointer to ADC raw data
        &simulink_rx_buffer.adc_result[SINV_ADC_ID_BUCK_VOUT],
        // ADC Channel settings.
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_GAIN, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_VOLTAGE_ADC_BIAS),
        // iqn is valid only when ctrl_gt is a fixed point type.
        CTRL_ADC_RESOLUTION, 24);



    //
    // output channel
    //
    ctl_init_pwm_channel(&sinv_pwm_out[0], 0, CONTROLLER_PWM_CMP_MAX);
    ctl_init_pwm_channel(&sinv_pwm_out[1], 0, CONTROLLER_PWM_CMP_MAX);
    ctl_init_pwm_channel(&sinv_pwm_out[2], 0, CONTROLLER_PWM_CMP_MAX);

    //
    // attach
    //
    ctl_attach_sinv_with_adc(
        // sinv controller
        &sinv_ctrl,
        // udc, idc
        &sinv_adc[SINV_ADC_ID_VDC].control_port, &sinv_adc[SINV_ADC_ID_VDC].control_port,
        // il
        &sinv_adc[SINV_ADC_ID_IL].control_port,
        // u grid, i grid
        &sinv_adc[SINV_ADC_ID_VG].control_port, &sinv_adc[SINV_ADC_ID_IG].control_port);

    ctl_attach_buck_ctrl_input(
        // Buck controller
        &buck_ctrl,
        // output capacitor voltage
        &sinv_adc[SINV_ADC_ID_BUCK_VOUT].control_port,
        // inductor current
        &sinv_adc[SINV_ADC_ID_BUCK_IL].control_port,
        // input voltage
        &sinv_adc[SINV_ADC_ID_BUCK_VIN].control_port);
    // open hardware switch
    // ctl_output_enable();
}
