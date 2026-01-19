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
//pwm_tri_channel_t pwm_out;

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

    //
    // attach
    //
    ctl_attach_gfl_inv(
        // inv controller
        &inv_ctrl,
        // idc, udc
        &idc.control_port, &udc.control_port,
        // grid side iabc, vabc
        &iabc.control_port, &vabc.control_port);

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


//////////////////////////////////////////////////////////////////////////
// communication functions and interrupt functions here

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
        //flag_system_enable = rx_data[0];
    }
    else if(status == 2)
    {
        CAN_readMessage(IRIS_CAN_BASE, 2, (uint16_t*)recv_content);
        CAN_clearInterruptStatus(CANA_BASE, 2);

        // set target value
#if BUILD_LEVEL == 1
        // For level 1 Set target voltage
        ctl_set_gfl_inv_voltage_openloop(&inv_ctrl, float2ctrl((float)recv_content[0].i32 / CAN_SCALE_FACTOR), float2ctrl((float)recv_content[1].i32 / CAN_SCALE_FACTOR));

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

// 假设定义一个本地的小缓存大小，能够覆盖硬件 FIFO 的深度（通常是 16 字节）
#define ISR_LOCAL_BUF_SIZE  16

//interrupt void INT_IRIS_UART_USB_RX_ISR(void)
//{
//    uint16_t rxBuf[ISR_LOCAL_BUF_SIZE];
//        uint16_t fifoLevel;
//
//        // 1. 获取当前 FIFO 中有多少数据 (关键步骤！)
//        // SCI_getRxFIFOStatus 返回的是枚举或数值，代表当前 FIFO 里的字数
//        fifoLevel = SCI_getRxFIFOStatus(IRIS_UART_USB_BASE);
//
//        // 2. 只有当 FIFO 里有数据时才读取
////        if(fifoLevel == 1)
////        {
////            rxBuf = SCI_readCharNonBlocking()
////        }
//        if(fifoLevel > 0)
//        {
//            // 3. 使用 DriverLib 函数读取
//            // 因为我们已经确认了 fifoLevel <= 实际存在的数据
//            // 所以这个函数内部的 while 等待循环不会生效，它会立即读完返回
//            SCI_readCharArray(IRIS_UART_USB_BASE, rxBuf, fifoLevel);
//
//            // 4. 推送数据到 AT 核心
//            // 注意：C2000 上 char 是 16-bit，可以直接转换
//            // 如果是其他架构可能需要压缩数据，但在 SCI 上通常低8位有效
//            at_device_rx_isr(&at_dev, (char*)rxBuf, fifoLevel);
//        }
//
//        // 5. 错误处理：检查溢出 (Overrun)
//        // DriverLib 通常提供获取状态的函数
//        uint32_t rxStatus = SCI_getRxStatus(IRIS_UART_USB_BASE);
//        if((rxStatus & SCI_RXSTATUS_ERROR) || (rxStatus & SCI_RXSTATUS_OVERRUN))
//        {
//            // 发生溢出，复位 FIFO
//            SCI_resetRxFIFO(IRIS_UART_USB_BASE);
//        }
//
//        // 6. 清除中断标志
//        SCI_clearInterruptStatus(IRIS_UART_USB_BASE, SCI_INT_RXFF | SCI_INT_RXERR);
//
//        // 7. PIE ACK (如果在 C2000 上)
//        Interrupt_clearACKGroup(INT_IRIS_UART_USB_RX_INTERRUPT_ACK_GROUP);
//}

void at_device_flush_rx_buffer()
{
    uint16_t fifoLevel;
    uint16_t rxBuf[ISR_LOCAL_BUF_SIZE];

    // 使用while一次性读取FIFO中的所有内容
    while((fifoLevel = SCI_getRxFIFOStatus(IRIS_UART_USB_BASE)) > 0)
    {
        // 读取数据
        SCI_readCharArray(IRIS_UART_USB_BASE, rxBuf, fifoLevel);

        // 推送给设备
        at_device_rx_isr(&at_dev, (char*)rxBuf, fifoLevel);
    }
}


interrupt void INT_IRIS_UART_USB_RX_ISR(void)
{
    uint32_t rxStatus;

    // ---------------------------------------------------------
    // 策略优化：使用 while 循环一次性抽干 FIFO
    // ---------------------------------------------------------
    // 原因：设置为 1/16 触发虽然保证了响应度，但为了减少中断频率，
    // 我们应该在一次中断里尽可能把 FIFO 里堆积的数据全读完，
    // 而不是读几个字节就退出去等下一次中断。

//    while((fifoLevel = SCI_getRxFIFOStatus(IRIS_UART_USB_BASE)) > 0)
//    {
////        // 限制单次读取长度，防止越界
////        if(fifoLevel > ISR_LOCAL_BUF_SIZE) {
////            fifoLevel = ISR_LOCAL_BUF_SIZE;
////        }
//
//        // 读取数据
//        SCI_readCharArray(IRIS_UART_USB_BASE, rxBuf, fifoLevel);
//
//        // 推送数据到 AT 核心
//        at_device_rx_isr(&at_dev, (char*)rxBuf, fifoLevel);
//
//        // 如果 FIFO 此时已经空了，while 循环会自动结束
//    }
    at_device_flush_rx_buffer();

    // ---------------------------------------------------------
    // 错误处理优化：不要盲目复位 FIFO
    // ---------------------------------------------------------
    rxStatus = SCI_getRxStatus(IRIS_UART_USB_BASE);

    if(rxStatus & SCI_RXSTATUS_OVERRUN)
    {
        // 仅处理溢出错误：清除溢出标志位，而不是复位整个 FIFO
        // C2000 DriverLib 通常通过写入 RXFFOVRCLR 位来清除
        // 如果没有直接API，可以使用 HWREG 操作，或者保持 resetRxFIFO 但仅针对 Overrun

        // 修正建议：只在确实溢出卡死时才 Reset，普通 Error 不要 Reset
        SCI_clearOverflowStatus(IRIS_UART_USB_BASE); // 假设有此函数，或者手动操作寄存器

        // 如果必须使用 resetRxFIFO，请确保仅在严重故障下使用
        // SCI_resetRxFIFO(IRIS_UART_USB_BASE);
    }

    if(rxStatus & SCI_RXSTATUS_ERROR)
    {
        // 对于 Frame Error / Parity Error (比如噪声 0xFF)
        // 读取数据寄存器通常会自动清除这些错误标志
        // 这里只需要做一个软件复位给 SCI 状态机（不清除 FIFO），或者单纯清除标志
        // 绝对不要调用 SCI_resetRxFIFO() !!!
    }

    // ---------------------------------------------------------
    // 清除中断标志 (关键)
    // ---------------------------------------------------------
    // 必须清除 RXFF (FIFO中断) 和 RXERR (错误中断) 标志，否则中断会卡死
    SCI_clearInterruptStatus(IRIS_UART_USB_BASE, SCI_INT_RXFF | SCI_INT_RXERR);

    // PIE ACK
    Interrupt_clearACKGroup(INT_IRIS_UART_USB_RX_INTERRUPT_ACK_GROUP);
}


