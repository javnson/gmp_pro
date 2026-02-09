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

//////////////////////////////////////////////////////////////////////////
// definitions of peripheral
//

// SIL standard port for Motor control

tri_ptr_adc_channel_t uabc;
tri_ptr_adc_channel_t iabc;

ptr_adc_channel_t udc;
ptr_adc_channel_t idc;

// pos_autoturn_encoder_t pos_enc;

pwm_tri_channel_t pwm_out;

// raw data
adc_gt uabc_raw[3];
adc_gt iabc_raw[3];
adc_gt udc_raw;
adc_gt idc_raw;

#if !defined PMSM_CTRL_USING_QEP_ENCODER
// Encoder Interface
ext_as5048a_encoder_t pos_enc;
#endif // PMSM_CTRL_USING_QEP_ENCODER

//uart_halt debug_uart;


/////////////////////////////////////////////////////////////////////////
// peripheral setup function
//

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
    // Setup Debug Uart
    debug_uart = debug_uart_BASE;

    gmp_base_print(TEXT_STRING("Hello World!\r\n"));

    asm(" RPT #255 || NOP");

    // Init ADC channel
    ctl_init_ptr_adc_channel(
        // bind idc channel with idc address
        &idc, &idc_raw,
        // ADC gain, ADC bias
        float2ctrl(MTR_CTRL_CURRENT_GAIN), float2ctrl(MTR_CTRL_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_tri_ptr_adc_channel(
        // bind ibac channel with iabc address
        &iabc, iabc_raw,
        // ADC gain, ADC bias
        float2ctrl(MTR_CTRL_CURRENT_GAIN), float2ctrl(MTR_CTRL_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_ptr_adc_channel(
        // bind udc channel with udc address
        &udc, &udc_raw,
        // ADC gain, ADC bias
        float2ctrl(MTR_CTRL_VOLTAGE_GAIN), float2ctrl(MTR_CTRL_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_tri_ptr_adc_channel(
        // bind vbac channel with vabc address
        &uabc, uabc_raw,
        // ADC gain, ADC bias
        float2ctrl(MTR_CTRL_VOLTAGE_GAIN), float2ctrl(MTR_CTRL_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

#if !defined PMSM_CTRL_USING_QEP_ENCODER
    // init AS5048 encoder
    ctl_init_as5048a_pos_encoder(&pos_enc, MOTOR_PARAM_POLE_PAIRS, SPI_ENCODER_BASE, SPI_ENCODER_NCS);
    // Set encoder offset
    ctl_set_as5048a_pos_encoder_offset(&pos_enc, MTR_ENCODER_OFFSET);
#endif // PMSM_CTRL_USING_QEP_ENCODER

    // bind peripheral to motor controller
    ctl_attach_mtr_adc_channels(&pmsm_ctrl.mtr_interface,
                                // phase voltage & phase current
                                &iabc.control_port, &uabc.control_port,
                                // dc bus voltage & dc bus current
                                &idc.control_port, &udc.control_port);

    ctl_attach_mtr_position(&pmsm_ctrl.mtr_interface, &pos_enc.encif);

    ctl_attach_pmsm_output(&pmsm_ctrl, &pwm_out.raw);

    // output channel
    ctl_init_pwm_tri_channel(&pwm_out, 0, CONTROLLER_PWM_CMP_MAX);

    // open hardware switch
    // ctl_output_enable();
}

//////////////////////////////////////////////////////////////////////////
// interrupt functions and callback functions here

// ADC interrupt
interrupt void MainISR(void)
{
    GPIO_WritePin(LED_STS, 0);

    //
    // call GMP ISR  Controller operation callback function
    //
    gmp_base_ctl_step();

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(ADC_A_BASE, ADC_INT_NUMBER1);

    //
    // Check if overflow has occurred
    //
    if (true == ADC_getInterruptOverflowStatus(ADC_A_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(ADC_A_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADC_A_BASE, ADC_INT_NUMBER1);
    }
    GPIO_WritePin(LED_STS, 1);

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_ADC_A_1_INTERRUPT_ACK_GROUP);
}

// EQEP index interrupt
interrupt void INT_EQEP_Encoder1_ISR(void)
{

    Interrupt_clearACKGroup(INT_EQEP_Encoder1_INTERRUPT_ACK_GROUP);
}

//////////////////////////////////////////////////////////////////////////
// Secure Function

// Enable Motor Controller
// Enable Output
void ctl_enable_output()
{
    //        csp_sl_enable_output();

    EPWM_clearTripZoneFlag(PHASE_U_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(PHASE_V_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(PHASE_W_BASE, EPWM_TZ_FLAG_OST);
}

// Disable Output
void ctl_disable_output()
{
    //        csp_sl_disable_output();

    EPWM_forceTripZoneEvent(PHASE_U_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_V_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_W_BASE, EPWM_TZ_FORCE_EVENT_OST);
}
