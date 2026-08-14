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

//////////////////////////////////////////////////////////////////////////
// peripheral setup function
//

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
    ctl_init_ptr_adc_channel(
        // bind idc channel with idc address
        &idc, &simulink_rx_buffer.idc,
        // ADC gain, ADC bias
        real2ctrl(MTR_CTRL_CURRENT_GAIN), real2ctrl(MTR_CTRL_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_tri_ptr_adc_channel(
        // bind ibac channel with iabc address
        &iabc, simulink_rx_buffer.iabc,
        // ADC gain, ADC bias
        real2ctrl(MTR_CTRL_CURRENT_GAIN), real2ctrl(MTR_CTRL_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_ptr_adc_channel(
        // bind udc channel with udc address
        &udc, &simulink_rx_buffer.udc,
        // ADC gain, ADC bias
        real2ctrl(MTR_CTRL_VOLTAGE_GAIN), real2ctrl(MTR_CTRL_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_tri_ptr_adc_channel(
        // bind vbac channel with vabc address
        &uabc, simulink_rx_buffer.uabc,
        // ADC gain, ADC bias
        real2ctrl(MTR_CTRL_VOLTAGE_GAIN), real2ctrl(MTR_CTRL_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_autoturn_pos_encoder(&pos_enc, MOTOR_PARAM_POLE_PAIRS, ((uint32_t)1 << 14) - 1);

    // bind peripheral to motor controller
    ctl_attach_mtr_adc_channels(&acm_ctrl.mtr_interface,
                                // phase voltage & phase current
                                &iabc.control_port, &uabc.control_port,
                                // dc bus voltage & dc bus current
                                &idc.control_port, &udc.control_port);

    ctl_attach_mtr_position(&acm_ctrl.mtr_interface, &pos_enc.encif);

    ctl_attach_acm_sensored_bare_output(&acm_ctrl, &pwm_out.raw);

    // output channel
    ctl_init_pwm_tri_channel(&pwm_out, 0, CONTROLLER_PWM_CMP_MAX);

    // open hardware switch
    // ctl_output_enable();
}
