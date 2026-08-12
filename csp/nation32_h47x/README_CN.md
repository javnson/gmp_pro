# N32H47x/N32H48x CSP 与常用外设快速指南

本目录提供 Nations N32H47x/N32H48x 对 GMP 的芯片支持。它沿用 STM32 CSP 的分层方式：

- `csp.typedef.h` 定义 GPIO、USART、SPI、I2C 和 FDCAN 的硬件句柄；
- `common/` 提供系统时基和 GPIO 模型；
- `src/n32_csp_adapter.c` 实现 CSP 生命周期、毫秒时基和 GMP 日志输出；
- `src/n32_peripheral_driver.c` 实现 GMP 通用 GPIO、阻塞式 UART 和 SPI 接口；
- 具体板卡的时钟、引脚和控制外设放在工程的 `USER/` 与 `xplt/` 中。

下文 API 以 `Nations.N32H47x_48x_Library.1.2.0` 为准，寄存器细节以《N32H47x/48x 系列用户手册 V1.0.0》为准。官方例程相对路径均从 `projects/n32h47x_48x_EVAL/examples` 开始。

## 1. 工程接入与 SDPE

1. 在编译器中定义具体器件宏，例如 `N32H474` 和 `USE_STDPERIPH_DRIVER`。
2. 将 N32 CMSIS core、device 和标准外设库 `inc` 加入头文件搜索路径。
3. 用 `gmp_src_mgr/gmp_framework_config.json` 选择 `core|std`、所需 core/CTL 模块和 `csp|nation32_h47x`，同步模式使用 `all`。
4. 将 `gmp_src_mgr/gmp_inc` 加入 include path，将 `gmp_src_mgr/gmp_src/*.c` 加入 Keil 工程。
5. 工程宏统一在 SDPE 文件 `xplt/xplt.config.h` 中管理。不要在 Keil、用户源文件和 CSP 中重复定义控制类型、CTL 开关等宏。

`n32h474vel7/gmp_src_mgr/gmp_generate_all.bat` 可同时重建头文件镜像和源文件平铺目录。

## 2. Cortex-M SysTick 与 GMP 时基

N32H47x 的 CMSIS device 包含标准 `SysTick_Config()`。GMP 不需要占用 GTIM：

```c
SystemCoreClockUpdate();
if (SysTick_Config(SystemCoreClock / 1000U) != 0U) {
    /* reload 超过 24 位，时钟或目标周期配置错误 */
    for (;;) {}
}

void SysTick_Handler(void)
{
    ++mwTick;                /* N32Cube 延时 */
    gmp_step_system_tick();  /* GMP 1 ms 时基 */
}
```

`gmp_base_get_system_tick()` 返回 CSP 内部计数器。计数器允许自然回绕；比较超时应使用无符号差值，不要比较绝对结束时间。不要在关闭 SysTick 中断的 ISR 中调用带毫秒超时的阻塞 UART/SPI 接口。

## 3. GPIO

先打开 GPIO 与 AFIO 时钟，再配置引脚：

```c
GPIO_InitType io;
RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOA, ENABLE);
RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
GPIO_InitStruct(&io);
io.Pin = GPIO_PIN_3;
io.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
io.GPIO_Pull = GPIO_NO_PULL;
io.GPIO_Slew_Rate = GPIO_SLEW_RATE_SLOW;
io.GPIO_Current = GPIO_DS_2mA;
io.GPIO_Alternate = GPIO_NO_AF;
GPIO_InitPeripheral(GPIOA, &io);
GPIO_SetBits(GPIOA, GPIO_PIN_3);
GPIO_ResetBits(GPIOA, GPIO_PIN_3);
```

使用 GMP 句柄时：

```c
GMP_N32_GPIO_HANDLE(led, GPIOA, GPIO_PIN_3);
gmp_hal_gpio_set_dir(&led, GMP_HAL_GPIO_DIR_OUT);
gmp_hal_gpio_write(&led, GMP_HAL_GPIO_HIGH);
```

N32H474VEL7-STB 的 D1/D2/D3 分别连接 PA3/PA8/PB4，均为高电平点亮。参考 `GPIO/LedBlink`。

## 4. 高级定时器 PWM

### 4.1 ATIM 互补 PWM

电机或数字电源优先使用 ATIM 的互补输出、死区与刹车功能。边沿对齐时：

```text
f_pwm = f_tim / ((Prescaler + 1) * (Period + 1))
duty  = Pulse / (Period + 1)
```

中心对齐模式的完整 PWM 周期还需乘 2。`f_tim` 必须根据 RCC/APB 分频和定时器倍频规则计算，不能直接假定等于 `SystemCoreClock`。

```c
TIM_TimeBaseInitType tb;
OCInitType oc;
TIM_BDTRInitType bdtr;

TIM_InitTimBaseStruct(&tb);
tb.Prescaler = 0U;
tb.CounterMode = TIM_CNT_MODE_UP;
tb.Period = pwm_period_ticks - 1U;
tb.ClkDiv = TIM_CLK_DIV1;
tb.RepetCnt = 0U;
TIM_InitTimeBase(ATIM1, &tb);

TIM_InitOcStruct(&oc);
oc.OCMode = TIM_OCMODE_PWM1;
oc.OutputState = TIM_OUTPUT_STATE_ENABLE;
oc.OutputNState = TIM_OUTPUT_NSTATE_ENABLE;
oc.Pulse = pwm_period_ticks / 2U;
oc.OCPolarity = TIM_OC_POLARITY_HIGH;
oc.OCNPolarity = TIM_OCN_POLARITY_HIGH;
TIM_InitOc1(ATIM1, &oc);
TIM_ConfigOc1Preload(ATIM1, TIM_OC_PRE_LOAD_ENABLE);
TIM_ConfigArPreload(ATIM1, ENABLE);

TIM_InitBkdtStruct(&bdtr);
bdtr.DeadTime = deadtime_code;
bdtr.Break = TIM_BREAK_IN_ENABLE;
bdtr.BreakPolarity = TIM_BREAK_POLARITY_HIGH;
bdtr.AutomaticOutput = TIM_AUTO_OUTPUT_ENABLE;
TIM_ConfigBkdt(ATIM1, &bdtr);

TIM_Enable(ATIM1, ENABLE);
TIM_EnableCtrlPwmOutputs(ATIM1, ENABLE);
```

运行时通过对应 compare/CC 寄存器更新占空比，或重新调用通道 compare 设置函数；启用 preload 后，新值在更新事件生效。真正接功率级前必须先验证互补极性、死区、Break 输入和上电默认关闭状态。参考 `AD_GP_BS TIM/PWM_Output` 与 `AD_GP_BS TIM/ComplementarySignals`。

### 4.2 SHRTIM

需要更高分辨率、复杂置位/复位源或多相同步时使用 SHRTIM：

```c
SHRTIM_TIM_SetPrescaler(SHRTIM1, SHRTIM_TIMER_D, SHRTIM_PRESCALERRATIO_MUL32);
SHRTIM_TIM_SetCounterMode(SHRTIM1, SHRTIM_TIMER_D, SHRTIM_MODE_CONTINUOUS);
SHRTIM_TIM_SetPeriod(SHRTIM1, SHRTIM_TIMER_D, period);
SHRTIM_TIM_SetCompare1(SHRTIM1, SHRTIM_TIMER_D, compare);
SHRTIM_OUT_SetOutputSetSrc(SHRTIM1, SHRTIM_OUTPUT_TD1, SHRTIM_OUTPUTSET_TIMPER);
SHRTIM_OUT_SetOutputResetSrc(SHRTIM1, SHRTIM_OUTPUT_TD1, SHRTIM_OUTPUTRESET_TIMCMP1);
SHRTIM_ForceUpdate(SHRTIM1, SHRTIM_TIMERUPDATE_D);
SHRTIM_EnableOutput(SHRTIM1, SHRTIM_OUTPUT_TD1);
SHRTIM_TIM_CounterEnable(SHRTIM1, SHRTIM_TIMER_D);
```

先配置 fault，再使能输出。参考 `SHRTIM/SHRTIM_Basic_Single_PWM`、`SHRTIM/SHRTIM_Multiphase`。

## 5. ADC

基本单次读取顺序是：GPIO 模拟模式、ADC 时钟、初始化、Ready、校准、通道/采样时间、启动、等待完成、读数据。

```c
ADC_InitType adc;
ADC_InitStruct(&adc);
adc.WorkMode = ADC_WORKMODE_INDEPENDENT;
adc.MultiChEn = DISABLE;
adc.ContinueConvEn = DISABLE;
adc.ExtTrigSelect = ADC_EXT_TRIG_REG_CONV_SOFTWARE;
adc.DatAlign = ADC_DAT_ALIGN_R;
adc.ChsNumber = 1U;
adc.Resolution = ADC_DATA_RES_12BIT;
ADC_Init(ADC1, &adc);
ADC_Enable(ADC1, ENABLE);
while (ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY) == RESET) {}
ADC_CalibrationOperation(ADC1, ADC_CALIBRATION_SINGLE_MODE);
while (ADC_GetCalibrationStatus(ADC1, ADC_CALIBRATION_SINGLE_MODE)) {}

ADC_ConfigRegularChannel(ADC1, ADC1_Channel_01_PB0, 1U,
                         ADC_SAMP_TIME_CYCLES_239_5);
ADC_EnableSoftwareStartConv(ADC1, ENABLE);
while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ENDC) == RESET) {}
ADC_ClearFlag(ADC1, ADC_FLAG_ENDC | ADC_FLAG_STR);
uint16_t raw = ADC_GetDat(ADC1);
```

控制环中不要轮询：使用 ATIM/SHRTIM 触发 ADC，DMA 搬运规则序列，并由 ADC/DMA 中断调用 `gmp_base_ctl_step()`。采样时间需结合信号源阻抗选择；第一次使用和模拟电源变化后应重新评估校准。参考 `ADC/ADC_SingleRead`、`ADC/ADC1_DMA`、`ADC/ExtLinesTrigger`、`ADC/TIMTrigger_AutoInjection`。

## 6. DAC

DAC1/DAC2 输出脚为 PA4/PA5，先设模拟模式并配置 DAC12 时钟：

```c
DAC_InitType dac;
RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_DAC12, ENABLE);
DAC_ConfigClkPrescaler(DAC12, 100U); /* H474 示例；按实际 DAC 源时钟计算 */
DAC_SetHighFrequencyMode(DAC12, DAC_HIGH_FREQ_MODE_BELOW_160M);
DAC_StructInit(&dac);
dac.DAC_Trigger = DAC_Trigger_Software;
dac.DAC_WaveGeneration = DAC_WaveGeneration_None;
dac.DAC_OutputBuffer = ENABLE;
dac.DAC_ConnectOnChipPeripheral = DISABLE;
dac.DAC_ConnectExternalPin = ENABLE;
dac.DAC_TriggerEnable = DISABLE;
DAC_Init(DAC1, &dac);
DAC_Enable(DAC1, ENABLE);
DAC_SetData(DAC1, DAC_ALIGN_R_12BIT, code & 0x0FFFU);
```

周期波形使用 TIM TRGO + DMA，不要在控制中断里逐点软件写。参考 `DAC/NoiseWaveOutput`、`DAC/DoubleDataDMAModeSineWave` 和 `DAC/UserBufferCalibration`。

## 7. QEP/正交编码器

N32H47x 没有以 `eQEP` 命名的独立模块，官方方案是 LPTIM encoder mode。官方 LPTIM1 例程使用 PC0/PC2、AF5：

```c
RCC_ConfigLPTIM1Clk(RCC_LPTIMCLK_SRC_LSI);
RCC_EnableLPTIMPeriphClk(RCC_LPTIM1_PERIPH_EN, ENABLE);
LPTIM_DeInit(LPTIM1);
LPTIM_SetPrescaler(LPTIM1, LPTIM_PRESCALER_DIV1);
LPTIM_SetClockSource(LPTIM1, LPTIM_CLK_SOURCE_INTERNAL);
LPTIM_EncoderModeCmd(LPTIM1, ENABLE);
LPTIM_SetEncoderModeClockPolarity(LPTIM1, LPTIM_ENCODER_MODE_RISING);
LPTIM_SetAutoReloadValue(LPTIM1, 0xFFFFU);
LPTIM_Cmd(LPTIM1, ENABLE);
LPTIM_StartCounter(LPTIM1, LPTIM_OPERATING_MODE_CONTINUOUS);
uint32_t position = LPTIM_GetCounterValue(LPTIM1);
```

速度计算应保存上次计数并用模减法处理回绕。索引 Z 相若需要零位捕获，应接 EXTI 或输入捕获通道，并在机械/电气允许的条件下清零软件位置。参考 `LPTIM/LPTIM_ENC`。

## 8. UART 与 GMP Debug Tool

```c
USART_InitType uart;
USART_StructInit(&uart);
uart.BaudRate = 115200U;
uart.WordLength = USART_WL_8B;
uart.StopBits = USART_STPB_1;
uart.Parity = USART_PE_NO;
uart.HardwareFlowControl = USART_HFCTRL_NONE;
uart.OverSampling = USART_16OVER;
uart.Mode = USART_MODE_RX | USART_MODE_TX;
USART_Init(USART1, &uart);
USART_Enable(USART1, ENABLE);
```

GMP 侧设置 `debug_uart = USART1`，再使用 `gmp_hal_uart_write/read`。N32H474VEL7-STB 的 J5/NS-LINK 使用 PA9(TX)、PA10(RX)，默认工程为 115200-8-N-1。`xplt.peripheral.c` 已将接收字节推入 `gmp_datalink_t`，可供 GMP Debug Tool 的 echo、PIL、参数整定和内存观察命令使用。参考 `USART/Polling`、`USART/Printf`。

## 9. I2C

GPIO 必须为开漏复用并有外部上拉。主机发送基本状态机：等待 `I2C_FLAG_BUSY` 清零，START，等待 `I2C_EVT_MASTER_MODE_FLAG`，发送 7 位地址，等待 TX mode，逐字节发送，等待发送完成，STOP。

```c
I2C_InitType i2c;
I2C_InitStruct(&i2c);
i2c.BusMode = I2C_BUSMODE_I2C;
i2c.FmDutyCycle = I2C_FMDUTYCYCLE_2;
i2c.OwnAddr1 = 0U;
i2c.AckEnable = I2C_ACKEN;
i2c.AddrMode = I2C_ADDR_MODE_7BIT;
i2c.ClkSpeed = 400000U;
I2C_Init(I2C1, &i2c);
I2C_Enable(I2C1, ENABLE);
```

单字节、双字节和多字节接收的 ACK/STOP 顺序不同，不能用同一段简单循环替代；直接复用 `I2C/I2C_Master` 或 `I2C/EEPROM_Polling` 的对应分支，并为每个状态等待加入超时和 NACK/仲裁丢失处理。

## 10. SPI

```c
SPI_InitType spi;
SPI_InitStruct(&spi);
spi.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
spi.SpiMode = SPI_MODE_MASTER;
spi.DataLen = SPI_DATA_SIZE_8BITS;
spi.CLKPOL = SPI_CLKPOL_LOW;
spi.CLKPHA = SPI_CLKPHA_FIRST_EDGE;
spi.NSS = SPI_NSS_SOFT;
spi.BaudRatePres = SPI_BR_PRESCALER_8;
spi.FirstBit = SPI_FB_MSB;
spi.CRCPoly = 7U;
SPI_Init(SPI1, &spi);
SPI_Enable(SPI1, ENABLE);
```

片选用 `gmp_spi_dev_t` 绑定 GPIO，底层传输使用 `gmp_hal_spi_bus_write/read/transfer`。传输完成前读取 RNE 并在结束前等待 BUSY 清零。参考 `SPI/FullDuplex_SoftNSS`、`SPI/SPI_DMA_T&R`。

## 11. CAN/CAN-FD

N32H47x 使用 FDCAN。初始化必须同时规划位时序和共享 Message RAM：

```c
FDCAN_InitType fd;
FDCAN_FilterType filter;
/* 填写 FrameFormat、Mode、nominal/data bit timing、MsgRamStrAddr、
   MsgRamOffset、各 FIFO/buffer 数量及元素大小后： */
if (FDCAN_Init(FDCAN1, &fd) == ERROR) { for (;;) {} }

filter.IdType = FDCAN_STANDARD_ID;
filter.FilterIndex = 0U;
filter.FilterType = FDCAN_FILTER_MASK;
filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
filter.FilterID1 = accepted_id;
filter.FilterID2 = accepted_mask;
FDCAN_ConfigFilter(FDCAN1, &filter);
FDCAN_Start(FDCAN1);
FDCAN_AddMsgToTxFifoQ(FDCAN1, &tx_header, tx_data);
if (FDCAN_GetRxFifoFillLevel(FDCAN1, FDCAN_RX_FIFO0) > 0U)
    FDCAN_GetRxMsg(FDCAN1, FDCAN_RX_FIFO0, &rx_header, rx_data);
```

经典 CAN 选 `FDCAN_FRAME_CLASSIC`/经典帧头；CAN-FD 可选 BRS。位速率公式、收发器终端电阻和采样点必须一起验证。多个 FDCAN 实例共享 RAM 时，每个实例的 offset 不得重叠，且 `EndAddress` 不能越界。参考 `FDCAN/FDCAN_Classic`、`FDCAN/FDCAN_Polling`、`FDCAN/FDCAN_Interrupt`。

## 12. 中断、DMA 与控制应用建议

- 后台通信和日志可以使用 1 ms SysTick；PWM/ADC 控制 ISR 应由 PWM 更新或 ADC 转换完成触发。
- ISR 中只做采样、控制计算、PWM 写入和必要的无锁入队；格式化打印留在后台。
- DMA 缓冲区应考虑对齐、cache 一致性和生产者/消费者所有权。
- 功率输出使能必须晚于 ADC 偏置校准、故障输入检查和控制器初始化；故障时先硬件关断 PWM，再记录软件状态。
- N32Cube 重新生成代码后，复核 `SysTick_Handler()` 中的 `gmp_step_system_tick()` 以及 `main.c` 中未启用 GTIM10 时基。

## 13. 参考资料

- `Nations.N32H47x_48x_Library.1.2.0/projects/n32h47x_48x_EVAL/examples`
- 《N32H47x/48x Series User Manual V1.0.0》
- `n32h474vel7/hw/CN_UG_N32H474VEL7_STB Development Board Hardware User Guide V1.0.pdf`
- `n32h474vel7/hw/N32H474VEL7_STB_V1.0.pdf`

