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

#include "user_main.h"
#include <xplt.peripheral.h>


//=================================================================================================
// definitions of peripheral

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

//=================================================================================================
// peripheral setup function

extern iic_halt iic_bus;
extern gpio_halt user_led;
extern gpio_halt gpio_beep;

//
// Function to configure I2C A in FIFO mode.
//
void initI2C()
{
    //
    // Must put I2C into reset before configuring it
    //
    I2C_disableModule(I2CA_BASE);

    //
    // I2C configuration. Use a 400kHz I2CCLK with a 33% duty cycle.
    //
    I2C_initController(I2CA_BASE, DEVICE_SYSCLK_FREQ, 400000, I2C_DUTYCYCLE_33);
    I2C_setBitCount(I2CA_BASE, I2C_BITCOUNT_8);
    I2C_setTargetAddress(I2CA_BASE, 0x70);
    I2C_setEmulationMode(I2CA_BASE, I2C_EMULATION_FREE_RUN);

    //
    // Enable stop condition and register-access-ready interrupts
    I2C_enableInterrupt(I2CA_BASE, I2C_INT_STOP_CONDITION |
                                   I2C_INT_REG_ACCESS_RDY);

    //
    // FIFO configuration
    //
    I2C_enableFIFO(I2CA_BASE);
    I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_RXFF | I2C_INT_TXFF);

    //
    // Configuration complete. Enable the module.
    //
    I2C_enableModule(I2CA_BASE);
}



// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
    // Setup Debug Uart
    debug_uart = IRIS_UART_USB_BASE;

    // Test print function
    gmp_base_print(TEXT_STRING("Hello World!\r\n"));
    asm(" RPT #255 || NOP");

    //
    // Initialize GPIOs for use as SDA A and SCL A respectively
    //
    // GPIO_setPinConfig(DEVICE_GPIO_CFG_SDAA);
    GPIO_setPadConfig(IRIS_IIC_I2CSDA_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(IRIS_IIC_I2CSDA_GPIO, GPIO_QUAL_ASYNC);

    // GPIO_setPinConfig(DEVICE_GPIO_CFG_SCLA);
    GPIO_setPadConfig(IRIS_IIC_I2CSCL_GPIO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(IRIS_IIC_I2CSCL_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPadConfig(IRIS_IIC_I2CSDA_GPIO, GPIO_PIN_TYPE_PULLUP);

    //
    // Set I2C use, initializing it for FIFO mode
    //
    initI2C();

    iic_bus = I2CA_BASE;

    user_led = SYSTEM_LED;

    gpio_beep = IRIS_GPIO1;

}



//=================================================================================================
// ADC Interrupt ISR and controller related function

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
//    if (gmp_base_get_system_tick() % 1000 < 500)
//        GPIO_WritePin(SYSTEM_LED, 0);
//    else
//        GPIO_WritePin(SYSTEM_LED, 1);

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

void reset_controller(void)
{
    int i = 0;

    GPIO_WritePin(PWM_RESET_PORT, 0);

    for(i=0;i<10000;++i);

    GPIO_WritePin(PWM_RESET_PORT, 1);

}

//=================================================================================================
// communication functions and interrupt functions here

// 10000 -> 1.0
#define CAN_SCALE_FACTOR 10000

// 32 bit union
typedef union {
    int32_t i32;
    uint16_t u16[2]; // C2000中uint16_t占1个word，32位占用2个word
} can_data_t;

// CAN interrupt
interrupt void INT_IRIS_CAN_0_ISR(void)
{
    uint32_t status = CAN_getInterruptCause(IRIS_CAN_BASE);

    uint16_t rx_data[4];
    can_data_t recv_content[2];

    if (status == 1)
    {
        CAN_readMessage(IRIS_CAN_BASE, 1, rx_data);
        CAN_clearInterruptStatus(CANA_BASE, 1);

        // Control Flag, Enable System
//        if (rx_data[0] == 1)
//        {
//            cia402_send_cmd(&cia402_sm, CIA402_CMD_ENABLE_OPERATION);
//        }
//        if (rx_data[0] == 0)
//        {
//            cia402_send_cmd(&cia402_sm, CIA402_CMD_DISABLE_VOLTAGE);
//        }
    }
    else if (status == 2)
    {
        CAN_readMessage(IRIS_CAN_BASE, 2, (uint16_t*)recv_content);
        CAN_clearInterruptStatus(CANA_BASE, 2);

        // set target value
#if BUILD_LEVEL == 1
        // For level 1 Set target voltage
        ctl_set_gfl_inv_voltage_openloop(&inv_ctrl, float2ctrl((float)recv_content[0].i32 / CAN_SCALE_FACTOR),
                                         float2ctrl((float)recv_content[1].i32 / CAN_SCALE_FACTOR));

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
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 4, 8, (uint16_t*)tran_content);

    //0x202: Monitor inverter voltage
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 5, 8, (uint16_t*)tran_content);

    // 0x203: Monitor grid current
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 6, 8, (uint16_t*)tran_content);

    // 0x204: TODO Monitor inverter current
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 7, 8, (uint16_t*)tran_content);

    // 0x205: TODO Monitor DC Voltage / Current
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 8, 8, (uint16_t*)tran_content);

    // 0x206: Monitor Grid Voltage A and PLL output angle
//    tran_content[0].i32 = (int32_t)(inv_ctrl.vabc.dat[phase_A] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.pll.theta * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 9, 8, (uint16_t*)tran_content);

    // 0x207: Monitor reserved
//    tran_content[0].i32 = (int32_t)(inv_ctrl.idq.dat[phase_d] * CAN_SCALE_FACTOR);
//    tran_content[1].i32 = (int32_t)(inv_ctrl.idq.dat[phase_q] * CAN_SCALE_FACTOR);

    CAN_sendMessage(IRIS_CAN_BASE, 10, 8, (uint16_t*)tran_content);
}

#if BOARD_SELECTION == GMP_IRIS

interrupt void INT_IRIS_UART_RS232_RX_ISR(void)
{
    // Nothing here

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_UART_RS232_RX_INTERRUPT_ACK_GROUP);
}

#endif // BOARD_SELECTION == GMP_IRIS

// a local small cache size, capable of covering the depth of the hardware FIFO (typically 16 bytes)
#define ISR_LOCAL_BUF_SIZE 16

void at_device_flush_rx_buffer()
{
    uint16_t fifoLevel;
    uint16_t rxBuf[ISR_LOCAL_BUF_SIZE];

    // Read all FIFO content
    while ((fifoLevel = SCI_getRxFIFOStatus(IRIS_UART_USB_BASE)) > 0)
    {
        // Get data
        SCI_readCharArray(IRIS_UART_USB_BASE, rxBuf, fifoLevel);

        // send to AT device
        at_device_rx_isr(&at_dev, (char*)rxBuf, fifoLevel);
    }
}

extern gmp_datalink_t dl;

void flush_dl_rx_buffer()
{
    uint16_t fifoLevel;
    data_gt rxBuf[ISR_LOCAL_BUF_SIZE];

    // Read all FIFO content
    if ((fifoLevel = SCI_getRxFIFOStatus(IRIS_UART_USB_BASE)) > 0)
    {
        // Get data
        SCI_readCharArray(IRIS_UART_USB_BASE, rxBuf, fifoLevel);

        gmp_dev_dl_push_str(&dl, rxBuf, fifoLevel);
    }
}


interrupt void INT_IRIS_UART_USB_RX_ISR(void)
{
    uint32_t rxStatus;

    // clear receive FIFO
//    at_device_flush_rx_buffer();

    flush_dl_rx_buffer();

    // Fault reaction
    rxStatus = SCI_getRxStatus(IRIS_UART_USB_BASE);

    if (rxStatus & SCI_RXSTATUS_OVERRUN)
    {
        // 仅处理溢出错误：清除溢出标志位，而不是复位整个 FIFO
        // C2000 DriverLib 通常通过写入 RXFFOVRCLR 位来清除
        // 如果没有直接API，可以使用 HWREG 操作，或者保持 resetRxFIFO 但仅针对 Overrun

        // 修正建议：只在确实溢出卡死时才 Reset，普通 Error 不要 Reset
        SCI_clearOverflowStatus(IRIS_UART_USB_BASE);

        // 如果必须使用 resetRxFIFO，请确保仅在严重故障下使用
        // SCI_resetRxFIFO(IRIS_UART_USB_BASE);
    }

    if (rxStatus & SCI_RXSTATUS_ERROR)
    {
        // 对于 Frame Error / Parity Error (比如噪声 0xFF)
        // 读取数据寄存器通常会自动清除这些错误标志
        // 这里只需要做一个软件复位给 SCI 状态机（不清除 FIFO），或者单纯清除标志
        // 绝对不要调用 SCI_resetRxFIFO() !!!
    }

    //
    // Clear the interrupt flag
    //
    SCI_clearInterruptStatus(IRIS_UART_USB_BASE, SCI_INT_RXFF | SCI_INT_RXERR);

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_IRIS_UART_USB_RX_INTERRUPT_ACK_GROUP);
}

////

//=========================================================
// 1. SPI 读写底层函数封装
//=========================================================

// 向 FPGA 写入寄存器
// 协议: 帧1=[15位=1(写), 14:8=地址, 7:0=保留] -> 帧2=[16位数据]
void SPI_writeReg(uint16_t addr, uint16_t data)
{
    // 构造写命令，最高位为 0
    uint16_t cmd = 0x0000 | ((addr & 0x7F) << 8); // 最高位自然是 0

    // 将两个 16-bit word 压入 TX FIFO 发送
    SPI_writeDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE, cmd);
    SPI_writeDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE, data);

    // 等待 FPGA 接收并返回两个 16-bit word
    // 虽然是写操作，但是 SPI 全双工会收到对方发回的废数据
    while(SPI_getRxFIFOStatus(IRIS_SPI_FPGA_BRIDGE_BASE) < SPI_FIFO_RX2);

    // 把接收到的这两个废数据读出，清空 RX FIFO，防止影响后续通信
    SPI_readDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE);
    SPI_readDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE);
}

// 从 FPGA 读取寄存器
// 协议: 帧1=[15位=0(读), 14:8=地址, 7:0=保留] -> 帧2=[16位占位符数据(0x0000)]
uint16_t SPI_readReg(uint16_t addr)
{
    // 构造读命令，最高位为 1
    uint16_t cmd = 0x8000 | ((addr & 0x7F) << 8); // 强制把最高位拉高
    uint16_t dummy_data = 0x0000; // 用于产生时钟的哑数据

    // 压入命令帧和数据帧
    SPI_writeDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE, cmd);
    SPI_writeDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE, dummy_data);

    // 等待接收 2 个字
    while(SPI_getRxFIFOStatus(IRIS_SPI_FPGA_BRIDGE_BASE) < SPI_FIFO_RX2);

    // 读出的第一个字是发送命令帧时 FPGA 返回的（通常是状态位或全0，直接丢弃）
    SPI_readDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE);

    // 读出的第二个字才是我们要的真实数据帧
    uint16_t read_data = SPI_readDataBlockingFIFO(IRIS_SPI_FPGA_BRIDGE_BASE);

    return read_data;
}


