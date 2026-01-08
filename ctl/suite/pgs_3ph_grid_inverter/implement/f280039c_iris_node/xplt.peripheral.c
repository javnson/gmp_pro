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

// inverter side voltage feedback
tri_ptr_adc_channel_t uuvw;
adc_gt uuvw_src[3];

// inverter side current feedback
tri_ptr_adc_channel_t iuvw;
adc_gt iuvw_src[3];

// grid side voltage feedback
tri_ptr_adc_channel_t vabc;
adc_gt vabc_src[3];

// grid side current feedback
tri_ptr_adc_channel_t iabc;
adc_gt iabc_src[3];

// DC bus current & voltage feedback
ptr_adc_channel_t udc;
adc_gt udc_src;
ptr_adc_channel_t idc;
adc_gt idc_src;

// PWM output channel
pwm_tri_channel_t pwm_out;

//////////////////////////////////////////////////////////////////////////
// peripheral setup function
//

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{

    // Setup Debug Uart
    debug_uart = IRIS_UART_USB_BASE;

    gmp_base_print(TEXT_STRING("Hello World!\r\n"));

    asm(" RPT #255 || NOP");

    // inverter side ADC
    ctl_init_tri_ptr_adc_channel(
        &uuvw, uuvw_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_tri_ptr_adc_channel(
        &iuvw, iuvw_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);

    // grid side ADC
    ctl_init_tri_ptr_adc_channel(
        &vabc, vabc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_tri_ptr_adc_channel(
        &iabc, iabc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_ptr_adc_channel(
        &udc, &udc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_ptr_adc_channel(
        &idc, &idc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);

    // output channel
    ctl_init_pwm_tri_channel(&pwm_out, 0, CTRL_PWM_CMP_MAX);

    //
    // attach
    //
    ctl_attach_three_phase_inv(
        // inv controller
        &inv_ctrl,
        // output PWM wave
        &pwm_out.raw,
        // udc, idc
        &udc.control_port, &idc.control_port,
        // grid side iabc, vabc
        &iabc.control_port, &vabc.control_port,
        // inverter siede iuvw, uuvw
        &iuvw.control_port, &uuvw.control_port);

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
    // Blink LED
    //
    if (gmp_base_get_system_tick() % 1000 < 500)
        GPIO_WritePin(IRIS_LED1, 0);
    else
        GPIO_WritePin(IRIS_LED1, 1);

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

// 10000 -> 1.0
#define CAN_SCALE_FACTOR 10000

// 32位数据转换联合体
typedef union {
    int32_t  i32;
    uint16_t u16[2]; // C2000中uint16_t占1个word，32位占用2个word
} can_data_t;

// CAN interrupt
interrupt void INT_IRIS_CAN_0_ISR(void)
{
    uint32_t status = CAN_getInterruptCause(IRIS_CAN_BASE);

    uint16_t rx_data[4];
    can_data_t recv_content[2];


    if(status == 1)
    {
        CAN_readMessage(IRIS_CAN_BASE, 1, rx_data);
        CAN_clearInterruptStatus(CANA_BASE, 1);

        // Control Flag, Enable System
        flag_system_enable = rx_data[0];
    }
    else if(status == 2)
    {
        CAN_readMessage(IRIS_CAN_BASE, 2, (uint16_t*)recv_content);
        CAN_clearInterruptStatus(CANA_BASE, 2);

        // set target value
#if BUILD_LEVEL == 1
        // For level 1 Set target voltage
        ctl_set_three_phase_inv_voltage_openloop(&inv_ctrl, float2ctrl((float)recv_content[0].i32 / CAN_SCALE_FACTOR), float2ctrl((float)recv_content[1].i32 / CAN_SCALE_FACTOR));

#endif // BUILD_LEVEL

    }

    //
    // Clear the interrupt flag
    //
    CAN_clearGlobalInterruptStatus(IRIS_CAN_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_CAN_0_INTERRUPT_ACK_GROUP);

}

interrupt void INT_IRIS_CAN_1_ISR(void)
{
    // Nothing here

    //
    // Clear the interrupt flag
    //
    CAN_clearGlobalInterruptStatus(IRIS_CAN_BASE, CAN_GLOBAL_INT_CANINT1);

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_CAN_1_INTERRUPT_ACK_GROUP);
}

void send_monitor_data(void)
{
    uint16_t rx_raw[4];
    can_data_t tran_content[2];

    static time_gt last_time_tick;
    static time_gt current_time_tick;

    current_time_tick = gmp_base_get_system_tick();

    // skip current frame
    if(last_time_tick == current_time_tick)
        return;
    else
    {
        // 0x201: Monitor Grid Voltage
        tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
        tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

        CAN_sendMessage(IRIS_CAN_BASE, 4, 8, (uint16_t*)tran_content);

        //0x202: Monitor inverter voltage
        tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
        tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

        CAN_sendMessage(IRIS_CAN_BASE, 5, 8, (uint16_t*)tran_content);

        // 0x203: Monitor grid current
        tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
        tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

        CAN_sendMessage(IRIS_CAN_BASE, 6, 8, (uint16_t*)tran_content);

        // 0x204: TODO Monitor inverter current
        tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
        tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

        CAN_sendMessage(IRIS_CAN_BASE, 7, 8, (uint16_t*)tran_content);

        // 0x205: TODO Monitor DC Voltage / Current
        tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
        tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

        CAN_sendMessage(IRIS_CAN_BASE, 8, 8, (uint16_t*)tran_content);

        // 0x206: Monitor Grid Voltage A and PLL output angle
        tran_content[0].i32 = (int32_t)(inv_ctrl.vabc.dat[phase_A] * CAN_SCALE_FACTOR);
        tran_content[1].i32 = (int32_t)(inv_ctrl.pll.theta * CAN_SCALE_FACTOR);

        CAN_sendMessage(IRIS_CAN_BASE, 9, 8, (uint16_t*)tran_content);

        // 0x207: Monitor reserved
        tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
        tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

        CAN_sendMessage(IRIS_CAN_BASE, 10, 8, (uint16_t*)tran_content);

        last_time_tick = current_time_tick;
    }


}


interrupt void INT_IRIS_UART_RS232_RX_ISR(void)
{

    // Nothing here

    //
    // Clear the interrupt flag
    //
    //SCI_clearGlobalInterruptStatus(IRIS_UART_USB_BASE, )
    // CAN_clearGlobalInterruptStatus(IRIS_CAN_BASE, CAN_GLOBAL_INT_CANINT1);

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_UART_RS232_RX_INTERRUPT_ACK_GROUP);

}

