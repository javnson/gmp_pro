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

#include <pca9555.h>

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

void HT16K33_WriteCommand(uint32_t i2cBase, uint16_t cmd)
{
    // 1. 确保总线处于空闲状态，避免冲突
    while(I2C_isBusBusy(i2cBase))
    {
    }

    // 2. 设置从机7位地址为 0x70
    // (硬件在 Controller Send Mode 下会自动在总线上发送 0xE0)
    I2C_setTargetAddress(i2cBase, 0x70);

    // 3. 设置数据长度为 1 个字节 (只发送一个命令字节)
    I2C_setDataCount(i2cBase, 1);

    // 4. 将要发送的命令放入数据发送寄存器 (DXR)
    I2C_putData(i2cBase, cmd);

    // 5. 配置为 主机发送模式 (Controller Transmitter)
    I2C_setConfig(i2cBase, I2C_CONTROLLER_SEND_MODE);

    // 6. 发送 START 和 STOP 信号
    // C2000硬件特性：当同时设置 START 和 STOP 位时，
    // I2C 硬件会在发出 START 后发送数据，当发送完 CNT(这里是1) 个字节后，自动在总线上发出 STOP 信号
    I2C_sendStartCondition(i2cBase);
    I2C_sendStopCondition(i2cBase);

    // 7. 等待 STOP 信号在总线上实际发送完毕 (发送完后硬件会自动清零该标志)
    while(I2C_getStopConditionStatus(i2cBase))
    {
    }
}

// 0~9 的共阴极标准段码表
const uint8_t LED_FONT[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

// 如果需要点亮小数点，只需将对应数字的段码按位或上 0x80
#define DP_MASK 0x80

// 定义 8 字节的本地显存
// 数组索引 0~7 分别对应物理上的 COM0 ~ COM7 (即第 1 到第 8 个数码管)
uint8_t DisplayRAM[8] = {0};

//*****************************************************************************
//
//! 将本地显存数据一次性刷入 HT16K33
//!
//*****************************************************************************
void HT16K33_UpdateDisplay(void)
{
    // 1. 等待总线彻底空闲，防止上一帧还没发完就开始新的一帧
    while(I2C_isBusBusy(I2CA_BASE))
    {
    }

    // 强制锁住目标器件为 HT16K33
        I2C_setTargetAddress(I2CA_BASE, 0x70);
        // 清空由于其他外设操作留下的 FIFO 状态
//        I2C_resetTxFIFO(I2CA_BASE);
//        I2C_resetRxFIFO(I2CA_BASE);
        I2C_clearStatus(I2CA_BASE, I2C_STS_NO_ACK);


    // 2. 设置本次传输的总字节数：1(地址) + 16(数据) = 17 字节
    I2C_setDataCount(I2CA_BASE, 17);

    // 3. 配置为 Controller 发送模式，并发出 START 和 STOP 信号
    // (由于开启了 FIFO，硬件会在 FIFO 内有数据时自动发出 START，发满 17 字节后自动发出 STOP)
    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE);
    I2C_sendStartCondition(I2CA_BASE);
    I2C_sendStopCondition(I2CA_BASE);

    // 4. 首先放入 HT16K33 的显存起始地址 0x00
    I2C_putData(I2CA_BASE, 0x00);

    uint16_t i;

    // 5. 循环放入 8 个数码管的数据 (共 16 字节)
    for(i = 0; i < 8; i++)
    {
        // 检查 FIFO 状态，如果 FIFO 满了 (16个)，就等它硬件发出去一点腾出空位
        while(I2C_getTxFIFOStatus(I2CA_BASE) == I2C_FIFO_TX16)
        {
        }
        // 放入当前 COM 的低字节 (数码管段码)
        I2C_putData(I2CA_BASE, DisplayRAM[i]);

        // 再次检查 FIFO 状态
        while(I2C_getTxFIFOStatus(I2CA_BASE) == I2C_FIFO_TX16)
        {
        }
        // 放入当前 COM 的高字节 (因为未使用 ROW8-15，直接填 0)
        I2C_putData(I2CA_BASE, 0x00);
    }
}

//*****************************************************************************
//
//! 从 HT16K33 读取按键输入状态
//!
//! \return 返回被按下的按键编号 (1~39)。如果没有按键按下，则返回 0。
//! \note 此函数默认返回检测到的第一个被按下的按键。
//
//*****************************************************************************
uint8_t HT16K33_ReadKey(void)
{
    uint8_t keyData[6] = {0};
    uint8_t keyId = 0;

    // 1. 等待总线空闲，确保安全介入
    while(I2C_isBusBusy(I2CA_BASE))
    {
    }


    // =======================================================
        // 核心修复 1：在多设备总线中，每次通信前必须强制重设目标地址！
        // =======================================================
        I2C_setTargetAddress(I2CA_BASE, 0x70);

        // =======================================================
        // 核心修复 2：在启动新通信前，复位并清空发送和接收 FIFO
        // 防止上一个外设(如HDC1080)的残留数据干扰状态机
        // =======================================================
//        I2C_resetTxFIFO(I2CA_BASE);
//        I2C_resetRxFIFO(I2CA_BASE);

        // 清除可能存在的 NACK 等错误标志位
        I2C_clearStatus(I2CA_BASE, I2C_STS_NO_ACK | I2C_STS_ARB_LOST);

    // ==========================================================
    // 第一阶段：向芯片发送“要读取的寄存器地址” (0x40)
    // ==========================================================
    I2C_setDataCount(I2CA_BASE, 1);
    I2C_putData(I2CA_BASE, 0x40); // 将地址放入发送 FIFO

    // 配置为主机发送模式，并同时触发 START 和 STOP (单字节写入)
    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE);
    I2C_sendStartCondition(I2CA_BASE);
    I2C_sendStopCondition(I2CA_BASE);

    // 等待地址发送完毕
    while(I2C_getStopConditionStatus(I2CA_BASE))
    {
    }

    // ==========================================================
    // 第二阶段：改变通信方向，连续读取 6 字节按键状态
    // ==========================================================
    I2C_setDataCount(I2CA_BASE, 6); // 准备接收 6 个字节

    // 配置为主机接收模式，并重新触发 START 和 STOP
    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_RECEIVE_MODE);
    I2C_sendStartCondition(I2CA_BASE);
    I2C_sendStopCondition(I2CA_BASE); // 接收满 6 个字节后硬件自动发送 STOP

    uint16_t i;

    // 从接收 FIFO 中逐个提取数据
    for(i = 0; i < 6; i++)
    {
        // 轮询等待 FIFO 中进入新的数据 (RX FIFO 数量大于 0 也就是不等于 RX0)
        while(I2C_getRxFIFOStatus(I2CA_BASE) == I2C_FIFO_RX0)
        {
        }
        keyData[i] = I2C_getData(I2CA_BASE);
    }

    // 等待总线读取彻底收尾
    while(I2C_getStopConditionStatus(I2CA_BASE))
    {
    }

    // ==========================================================
    // 第三阶段：解析按键数据并计算唯一的 Key ID
    // ==========================================================
    uint16_t byteIdx;
    uint16_t bitIdx;

    for(byteIdx = 0; byteIdx < 6; byteIdx++)
    {
        // 这一字节不为 0，说明这 8 个位里面有按键被按下了
        if(keyData[byteIdx] != 0)
        {
            for(bitIdx = 0; bitIdx < 8; bitIdx++)
            {
                // 用掩码逐位判断是否为 1
                if(keyData[byteIdx] & (1 << bitIdx))
                {
                    // 计算这是第几行 (KS0, KS1, KS2)
                    uint16_t ksRow = byteIdx / 2;
                    // 计算这是第几列 (K0 ~ K12)
                    uint16_t kCol  = (byteIdx % 2) * 8 + bitIdx;

                    // 将行列转换为 1~39 的绝对编号
                    keyId = (ksRow * 13) + kCol + 1;

                    // 找到按键就立刻返回。
                    // (如果您需要处理多键同按，可以把这里改为存入数组，而不是直接 return)
                    return keyId;
                }
            }
        }
    }

    // 如果 6 个字节全是 0，证明目前没人按键
    return 0;
}

// 测试函数：让数码管显示 "12345.678"
void Test_ShowNumbers(void)
{
    // 填充显存 (查表获取段码)
    DisplayRAM[0] = LED_FONT[1];
    DisplayRAM[1] = LED_FONT[2];
    DisplayRAM[2] = LED_FONT[3];
    DisplayRAM[3] = LED_FONT[4];
    DisplayRAM[4] = LED_FONT[5] | DP_MASK; // 第 5 位加上小数点
    DisplayRAM[5] = LED_FONT[6];
    DisplayRAM[6] = LED_FONT[7];
    DisplayRAM[7] = LED_FONT[8];

    // 一键刷新到屏幕
    HT16K33_UpdateDisplay();
}

//*****************************************************************************
//
//! HT16K33 芯片初始化序列
//!
//! \param i2cBase I2C 模块的基地址 (例如 I2CA_BASE)
//
//*****************************************************************************
void HT16K33_Init(uint32_t i2cBase)
{
    // 步骤1：发送 0x21 启动系统振荡器
    HT16K33_WriteCommand(i2cBase, 0x21);

    // (可选) 建议在这里加极短的延时，确保HT16K33内部振荡器起振稳定
    DEVICE_DELAY_US(100);

    // 步骤2：发送 0x81 开启显示，并设置无闪烁
    HT16K33_WriteCommand(i2cBase, 0x81);

    // 步骤3(补充)：发送 0xEF 设置为最大亮度 (推荐初始化时配置)
    // 根据需求可以修改为 0xE0(最暗) 到 0xEF(最亮)
    HT16K33_WriteCommand(i2cBase, 0xEF);
}



//*****************************************************************************
//
//! 初始化 HDC1080 传感器
//!
//*****************************************************************************
void HDC1080_Init(void)
{
    while(I2C_isBusBusy(I2CA_BASE))
    {
    }

    I2C_setTargetAddress(I2CA_BASE, 0x40);

    // 写入配置寄存器：1字节指针地址(0x02) + 2字节配置数据(0x0000)
    I2C_setDataCount(I2CA_BASE, 3);
    I2C_putData(I2CA_BASE, 0x02); // 配置寄存器指针
    I2C_putData(I2CA_BASE, 0x00); // 配置数据高字节：独立测量，14位精度，关闭加热器
    I2C_putData(I2CA_BASE, 0x00); // 配置数据低字节

    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE);
    I2C_sendStartCondition(I2CA_BASE);
    I2C_sendStopCondition(I2CA_BASE);

    // 等待发送完毕
    while(I2C_getStopConditionStatus(I2CA_BASE))
    {
    }
}

//*****************************************************************************
//
//! 获取当前环境温度 (摄氏度)
//!
//*****************************************************************************
float HDC1080_GetTemperature(void)
{
    uint8_t msb = 0, lsb = 0;
    uint16_t rawTemp = 0;

    while(I2C_isBusBusy(I2CA_BASE))
    {
    }

    // ==========================================
    // 第一步：写入指针 0x00，触发温度测量
    // ==========================================
    I2C_setTargetAddress(I2CA_BASE, 0x40);
    I2C_setDataCount(I2CA_BASE, 1);
    I2C_putData(I2CA_BASE, 0x00);

    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE);
    I2C_sendStartCondition(I2CA_BASE);
    I2C_sendStopCondition(I2CA_BASE);

    while(I2C_getStopConditionStatus(I2CA_BASE))
    {
    }

    // ==========================================
    // 第二步：等待芯片内部 ADC 转换完成 (14位需要约 6.5ms)
    // ==========================================
    DEVICE_DELAY_US(10000); // 延时 10ms 确保转换结束

    // ==========================================
    // 第三步：发起读取请求，接收 2 字节结果
    // ==========================================
    I2C_setDataCount(I2CA_BASE, 2);
    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_RECEIVE_MODE);
    I2C_sendStartCondition(I2CA_BASE);
    I2C_sendStopCondition(I2CA_BASE);

    // 等待接收 2 个字节
    while(I2C_getStopConditionStatus(I2CA_BASE))
    {
    }

    // 从 FIFO 中读出高低字节
    msb = I2C_getData(I2CA_BASE);
    lsb = I2C_getData(I2CA_BASE);

    rawTemp = (msb << 8) | lsb;

    // ==========================================
    // 第四步：根据公式换算为物理值
    // ==========================================
    return ( ((float)rawTemp / 65536.0f) * 165.0f ) - 40.0f;
}

//*****************************************************************************
//
//! 获取当前环境相对湿度 (%RH)
//!
//*****************************************************************************
float HDC1080_GetHumidity(void)
{
    uint8_t msb = 0, lsb = 0;
    uint16_t rawHum = 0;

    while(I2C_isBusBusy(I2CA_BASE))
    {
    }

    // ==========================================
    // 第一步：写入指针 0x01，触发湿度测量
    // ==========================================
    I2C_setTargetAddress(I2CA_BASE, 0x40);
    I2C_setDataCount(I2CA_BASE, 1);
    I2C_putData(I2CA_BASE, 0x01);

    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE);
    I2C_sendStartCondition(I2CA_BASE);
    I2C_sendStopCondition(I2CA_BASE);

    while(I2C_getStopConditionStatus(I2CA_BASE))
    {
    }

    // ==========================================
    // 第二步：等待芯片内部 ADC 转换完成
    // ==========================================
    DEVICE_DELAY_US(10000); // 延时 10ms

    // ==========================================
    // 第三步：发起读取请求，接收 2 字节结果
    // ==========================================
    I2C_setDataCount(I2CA_BASE, 2);
    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_RECEIVE_MODE);
    I2C_sendStartCondition(I2CA_BASE);
    I2C_sendStopCondition(I2CA_BASE);

    // 等待接收 2 个字节
    while(I2C_getStopConditionStatus(I2CA_BASE))
    {
    }

    // 从 FIFO 中读出高低字节
    msb = I2C_getData(I2CA_BASE);
    lsb = I2C_getData(I2CA_BASE);

    rawHum = (msb << 8) | lsb;

    // ==========================================
    // 第四步：根据公式换算为相对湿度百分比
    // ==========================================
    return ( ((float)rawHum / 65536.0f) * 100.0f );
}

//*****************************************************************************
// 向 PCA9555 写入单个寄存器
//*****************************************************************************
void PCA9555_WriteReg(uint8_t reg, uint8_t data)
{
    while(I2C_isBusBusy(I2CA_BASE));

    I2C_setTargetAddress(I2CA_BASE, PCA9555_I2C_ADDR);
//    I2C_resetTxFIFO(I2CA_BASE);
//    I2C_resetRxFIFO(I2CA_BASE);
    I2C_clearStatus(I2CA_BASE, I2C_STS_NO_ACK | I2C_STS_ARB_LOST);

    I2C_setDataCount(I2CA_BASE, 2);
    I2C_putData(I2CA_BASE, reg);  // 发送寄存器指针
    I2C_putData(I2CA_BASE, data); // 发送数据

    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE);
    I2C_sendStartCondition(I2CA_BASE);
    I2C_sendStopCondition(I2CA_BASE);

    while(I2C_getStopConditionStatus(I2CA_BASE));
}

//*****************************************************************************
// 从 PCA9555 读取单个寄存器
//*****************************************************************************
uint8_t PCA9555_ReadReg(uint8_t reg)
{
    uint8_t data = 0;
    while(I2C_isBusBusy(I2CA_BASE));

    I2C_setTargetAddress(I2CA_BASE, PCA9555_I2C_ADDR);
//    I2C_resetTxFIFO(I2CA_BASE);
//    I2C_resetRxFIFO(I2CA_BASE);
    I2C_clearStatus(I2CA_BASE, I2C_STS_NO_ACK | I2C_STS_ARB_LOST);

    // 1. 发送要读取的寄存器地址 (不发 STOP)
    I2C_setDataCount(I2CA_BASE, 1);
    I2C_putData(I2CA_BASE, reg);
    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE);
    I2C_sendStartCondition(I2CA_BASE);

    while((I2C_getStatus(I2CA_BASE) & I2C_STS_REG_ACCESS_RDY) == 0)
    {
        if(I2C_getStatus(I2CA_BASE) & I2C_STS_NO_ACK)
        {
            I2C_sendStopCondition(I2CA_BASE);
            I2C_clearStatus(I2CA_BASE, I2C_STS_NO_ACK);
            return 0; // 防死锁退出
        }
    }

    // 2. Repeated Start 接收数据
    I2C_setDataCount(I2CA_BASE, 1);
    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_RECEIVE_MODE);
    I2C_sendStartCondition(I2CA_BASE);
    I2C_sendStopCondition(I2CA_BASE);

    while(I2C_getRxFIFOStatus(I2CA_BASE) == I2C_FIFO_RX0)
    {
        if(I2C_getStatus(I2CA_BASE) & I2C_STS_NO_ACK)
        {
            I2C_clearStatus(I2CA_BASE, I2C_STS_NO_ACK);
            return 0;
        }
    }

    data = I2C_getData(I2CA_BASE);
    while(I2C_getStopConditionStatus(I2CA_BASE));

    return data;
}

//*****************************************************************************
// 控制蜂鸣器开关 (仅修改 IO0_5，不影响其他引脚)
//*****************************************************************************
void PCA9555_SetBuzzer(bool enable)
{
    // 读取当前 Port0 的输出锁存器状态 [cite: 1364-1365]
    uint8_t out0 = PCA9555_ReadReg(PCA9555_REG_OUT_0);

    if(enable)
    {
        out0 |= IO0_BUZZER_MASK;  // 将 IO0_5 置 1
    }
    else
    {
        out0 &= ~IO0_BUZZER_MASK; // 将 IO0_5 清 0
    }

    // 写回输出寄存器
    PCA9555_WriteReg(PCA9555_REG_OUT_0, out0);
}

//*****************************************************************************
// 初始化 PCA9555 (带有硬件安全强制约束)
// \param user_port0_dir: 供用户自定义的 IO0_6 和 IO0_7 的方向 (1=输入, 0=输出)
// \param user_port1_dir: 供用户自定义的 IO1_0 ~ IO1_7 的方向
//*****************************************************************************
void PCA9555_Init(uint8_t user_port0_dir, uint8_t user_port1_dir)
{
    // 强制约束 Port 0: Bit0~4 强制置 1, Bit5 强制清 0
    uint8_t final_port0_dir = user_port0_dir;
    final_port0_dir |= IO0_JOYSTICK_MASK;  // 摇杆引脚设为输入
    final_port0_dir &= ~IO0_BUZZER_MASK;   // 蜂鸣器引脚设为输出

    // 写入配置寄存器
    PCA9555_WriteReg(PCA9555_REG_CFG_0, final_port0_dir);
    PCA9555_WriteReg(PCA9555_REG_CFG_1, user_port1_dir);

    // 初始化时关闭蜂鸣器 (假设高电平响，这里写入 0)
    PCA9555_SetBuzzer(1);
}



//*****************************************************************************
// 读取摇杆状态 (返回 Bit0 ~ Bit4 的状态，屏蔽其他位)
//*****************************************************************************
uint8_t PCA9555_GetJoystick(void)
{
    uint8_t in0 = PCA9555_ReadReg(PCA9555_REG_IN_0);
    return (in0 & IO0_JOYSTICK_MASK);
}

//*****************************************************************************
// 用户通用 GPIO 操作函数
//*****************************************************************************
// 读取整个 Port1 的电平
uint8_t PCA9555_ReadPort1(void)
{
    return PCA9555_ReadReg(PCA9555_REG_IN_1);
}

// 设置整个 Port1 的输出电平
void PCA9555_WritePort1(uint8_t data)
{
    PCA9555_WriteReg(PCA9555_REG_OUT_1, data);
}


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


uint8_t currentKey = 0;
uint8_t joystickStatus = 0;

float current_temp;
float current_hum;

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{

    // Setup Debug Uart
    debug_uart = IRIS_UART_USB_BASE;

    reset_controller();

    // Test print function
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
//    ctl_init_tri_ptr_adc_channel(
//        &vabc, vabc_src,
//        // ADC gain, ADC bias
//        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
//        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_VOLTAGE_BIAS),
//        // ADC resolution, IQN
//        12, 24);

//    ctl_init_tri_ptr_adc_channel(
//        &iabc, iabc_src,
//        // ADC gain, ADC bias
//        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
//        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_GRID_CURRENT_BIAS),
//        // ADC resolution, IQN
//        12, 24);

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
    // Initialize GPIOs for use as SDA A and SCL A respectively
    //
    // GPIO_setPinConfig(DEVICE_GPIO_CFG_SDAA);
    GPIO_setPadConfig(IRIS_IIC_I2CSDA_GPIO, GPIO_PIN_TYPE_PULLUP);
    // GPIO_setQualificationMode(DEVICE_GPIO_PIN_SDAA, GPIO_QUAL_ASYNC);

    // GPIO_setPinConfig(DEVICE_GPIO_CFG_SCLA);
    GPIO_setPadConfig(IRIS_IIC_I2CSCL_GPIO, GPIO_PIN_TYPE_PULLUP);
    // GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCLA, GPIO_QUAL_ASYNC);

    //
    // Set I2C use, initializing it for FIFO mode
    //
    initI2C();

    HT16K33_Init(IRIS_IIC_BASE);
    Test_ShowNumbers();

    HDC1080_Init();

    // 3. 初始化 PCA9555:
        // 让 Port0 的自定义引脚(6,7)为输出(0), Port1 全部为输出(0x00)
        PCA9555_Init(0x00, 0x00);
//        // 3. 初始化 PCA9555:
//            // 让 Port0 的自定义引脚(6,7)为输出(0), Port1 全部为输出(0x00)
//            PCA9555_Init(0x00, 0x00);


//    uint8_t currentKey = 0;

    while(1)
    {

        current_temp = HDC1080_GetTemperature();
        current_hum = HDC1080_GetHumidity();

        // 1. 读取按键状态
                currentKey = HT16K33_ReadKey();

                // 2. 如果有按键按下，执行对应的逻辑
                if(currentKey != 0)
                {
                   gmp_base_print("Key press: %d", currentKey);
                }

                // 适当延时，约 20~50ms 轮询一次即可，这也是很好的按键消抖周期
                DEVICE_DELAY_US(30000);



                // ---------------- A. 轮询读取输入设备 ----------------
                        joystickStatus = PCA9555_GetJoystick();
                        //htKey = HT16K33_ReadKey();

                        // ---------------- B. 业务逻辑处理 ----------------
                        // 假设摇杆未按下时全是高电平，按下某方向后对应位变低
                        if(joystickStatus != 0x1F)
                        {
                            // 有摇杆动作：让蜂鸣器短促滴一声
                            PCA9555_SetBuzzer(true);
                            DEVICE_DELAY_US(50000); // 响 50ms
                            PCA9555_SetBuzzer(false);

                            // 可以通过 Port1 控制外围的扩展 LED，这里把摇杆状态直接映射出去观察
                            PCA9555_WritePort1(~joystickStatus);
                        }


    }
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
    if (gmp_base_get_system_tick() % 1000 < 500)
        GPIO_WritePin(SYSTEM_LED, 0);
    else
        GPIO_WritePin(SYSTEM_LED, 1);

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
        if (rx_data[0] == 1)
        {
            cia402_send_cmd(&cia402_sm, CIA402_CMD_ENABLE_OPERATION);
        }
        if (rx_data[0] == 0)
        {
            cia402_send_cmd(&cia402_sm, CIA402_CMD_DISABLE_VOLTAGE);
        }
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

interrupt void INT_IRIS_UART_USB_RX_ISR(void)
{
    uint32_t rxStatus;

    // clear receive FIFO
    at_device_flush_rx_buffer();

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
