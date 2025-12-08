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

// SIL standard port for Motor control

tri_ptr_adc_channel_t uabc;
tri_ptr_adc_channel_t iabc;

ptr_adc_channel_t udc;
ptr_adc_channel_t idc;

pos_autoturn_encoder_t pos_enc;

pwm_tri_channel_t pwm_out;

uint32_t output_voltage_compare = 3000;

//////////////////////////////////////////////////////////////////////////
// peripheral setup function
//

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
//    ctl_init_ptr_adc_channel(
//        // bind idc channel with idc address
//        &idc, &simulink_rx_buffer.idc,
//        // ADC gain, ADC bias
//        ctl_gain_calc_shunt_amp(CTRL_ADC_VOLTAGE_REF, MTR_CTRL_CURRENT_BASE, BOOSTXL_3PHGANINV_PH_SHUNT_RESISTANCE_OHM,
//                                BOOSTXL_3PHGANINV_PH_CSA_GAIN_V_V),
//        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, BOOSTXL_3PHGANINV_PH_CSA_BIAS_V),
//        // ADC resolution, IQN
//        12, 24);
//
//    ctl_init_tri_ptr_adc_channel(
//        // bind ibac channel with iabc address
//        &iabc, simulink_rx_buffer.iabc,
//        // ADC gain, ADC bias
//        ctl_gain_calc_shunt_amp(CTRL_ADC_VOLTAGE_REF, MTR_CTRL_CURRENT_BASE, BOOSTXL_3PHGANINV_PH_SHUNT_RESISTANCE_OHM,
//                                BOOSTXL_3PHGANINV_PH_CSA_GAIN_V_V),
//        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, BOOSTXL_3PHGANINV_PH_CSA_BIAS_V),
//        // ADC resolution, IQN
//        12, 24);
//
//    ctl_init_ptr_adc_channel(
//        // bind udc channel with udc address
//        &udc, &simulink_rx_buffer.udc,
//        // ADC gain, ADC bias
//        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_GAIN, MTR_CTRL_VOLTAGE_BASE),
//        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_BIAS_V),
//        // ADC resolution, IQN
//        12, 24);
//
//    ctl_init_tri_ptr_adc_channel(
//        // bind vbac channel with vabc address
//        &uabc, simulink_rx_buffer.uabc,
//        // ADC gain, ADC bias
//        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_GAIN, MTR_CTRL_VOLTAGE_BASE),
//        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_BIAS_V),
//        // ADC resolution, IQN
//        12, 24);

    ctl_init_autoturn_pos_encoder(&pos_enc, MOTOR_PARAM_POLE_PAIRS, ((uint32_t)1 << 14) - 1);

    ctl_init_pwm_tri_channel(&pwm_out, 0, CTRL_PWM_CMP_MAX);

    // bind peripheral to motor controller
    ctl_attach_mtr_adc_channels(&pmsm_ctrl.mtr_interface,
                                // phase voltage & phase current
                                &iabc.control_port, &uabc.control_port,
                                // dc bus voltage & dc bus current
                                &idc.control_port, &udc.control_port);

    ctl_attach_mtr_position(&pmsm_ctrl.mtr_interface, &pos_enc.encif);

    ctl_attach_pmsm_output(&pmsm_ctrl, &pwm_out.raw);

    // output channel
    ctl_init_pwm_tri_channel(&pwm_out, 0, CTRL_PWM_CMP_MAX);

}

//////////////////////////////////////////////////////////////////////////
// interrupt functions and callback functions here

// ADC interrupt
interrupt void MainISR(void)
{
    //
    // call GMP ISR  Controller operation callback function
    //
    gmp_base_ctl_step();

    //
    // Call GMP Timer
    //
    gmp_step_system_tick();

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1);

    //
    // Check if overflow has occurred
    //
    if (true == ADC_getInterruptOverflowStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(IRIS_ADCA_BASE, ADC_INT_NUMBER1);
    }

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_ADCA_1_INTERRUPT_ACK_GROUP);
}

// EQEP index interrupt
//interrupt void INT_EQEP_Encoder_ISR(void)
//{
//
//    Interrupt_clearACKGroup(INT_EQEP_Encoder_INTERRUPT_ACK_GROUP);
//}

