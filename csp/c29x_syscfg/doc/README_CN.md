# GMP C29x SysConfig 外设使用指南

[English](readme.md) | **简体中文**

本文面向 F29H85x/C29x 的 GMP 工程。以下接口以 TI F29H85x SDK
1.02.01.00 和 DriverLib 为基准。引脚、外设实例、SOC 编号和中断号应在
`.syscfg` 中配置，并在 C 代码中使用生成的 `board.h` 名称；不要把开发板引脚号散落在控制算法中。

## 1. GMP 工程启动顺序

`csp/c29x_syscfg/src/sysconfig_adapter.c` 提供 GMP 生命周期：

1. `Device_init()` 初始化器件时钟和低层设备支持。
2. `Board_init()` 应用 SysConfig 生成的 GPIO、UART、ADC、ePWM、QEP 和中断配置。
3. `setup_peripheral()` 绑定 GMP 对象、ADC 标度和调试 UART，并把功率输出保持在 Trip 状态。
4. `gmp_csp_post_process()` 最后开放 CPU 和全局中断。
5. 高频 ISR 只执行“读取 ADC -> `gmp_base_ctl_step()` -> 写 PWM -> 清中断”；UART 和调试协议放在后台任务。

`f29h850tu9_lp` 是最小参考工程：LED 每 500 ms 翻转，UARTA 以 115200 bit/s 运行 GMP 的 8 位
Data Link。C29x 是字节寻址平台，上位机必须使用
`tools/gmp_pil_server/gmp_debugger/run_u8.bat`。

## 2. ePWM

推荐用 SysConfig 配置计数模式、周期、比较影子寄存器、动作限定、死区和 Trip Zone。若
`TBCLK` 为 ePWM 时基时钟：

- 向上计数：`TBPRD = TBCLK / fpwm - 1`。
- 对称上下计数：`TBPRD = TBCLK / (2 * fpwm)`。
- 占空比由 `CMPA/TBPRD` 决定；电机控制通常在安全限幅后同时更新三个相桥的 `CMPA`。

运行时可以使用：

```c
EPWM_setTimeBasePeriod(PHASE_U_BASE, period);
EPWM_setCounterCompareValue(PHASE_U_BASE,
                            EPWM_COUNTER_COMPARE_A, compare);
```

控制用 PWM 应启用比较影子装载，避免半周期内直接改写造成毛刺。互补输出应配置 RED/FED
死区。初始化和故障时先调用 `EPWM_forceTripZoneEvent(...,
EPWM_TZ_FORCE_EVENT_OST)`；确认 ADC、保护极性和占空比都正确后，才清除 OST 并使能门极。

官方参考：
`examples/driverlib/single_core/epwm/epwm_ex2_updown_aq`。电机板完整配置可参考
F29x Motor Control SDK 的
`solutions/servo_drive_qep/f29h85x/drivers/servo_drive_BXL_3PHGANINV.syscfg`。

## 3. ADC

电机控制应让 ePWM SOCA/SOCB 触发 ADC SOC，使采样点与 PWM 同步。SysConfig 中至少要设置：

- ADC 输入通道、SOC 编号和采样窗口；采样窗口必须满足模拟源阻抗和数据手册要求。
- 触发源，例如 `ADC_TRIGGER_EPWM1_SOCA`。
- ADC 中断的 EOC 来源；通常选择最后完成的 SOC。
- 中断脉冲在转换结束产生，并为 ISR 配置优先级。

ISR 中读取生成的结果基址和 SOC 编号：

```c
uint16_t sample = ADC_readResult(MTR1_IU_RESULT_BASE, MTR1_IU);
ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
if (ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1)) {
    ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}
```

在 GMP 中，原始码先写入 `adc_gt`，再调用 `ctl_step_ptr_adc_channel()` 或
`ctl_step_tri_ptr_adc_channel()` 做偏置、增益和 PU 换算。不要把上一次 PWM 输出写回动作放到
ADC 采样之前。

官方参考：`examples/driverlib/single_core/adc/adc_ex2_soc_epwm` 和
`adc_ex10_multiple_soc_epwm`。

## 4. eQEP

在 SysConfig 中选择 QEPA/QEPB/QEPI 引脚和 Input XBAR，设置最大位置、索引复位方式、单位定时器和
低速捕获预分频。常用读取接口为：

```c
uint32_t position = EQEP_getPosition(MTR1_QEP_BASE);
```

`positionCounterMax` 一般设为每机械转编码器计数减一。把位置交给 GMP 编码器接口前，应明确
四倍频计数、机械零位、极对数和旋转方向。高速可按单位定时器的位置差计算速度，低速可用捕获周期；
索引脉冲只应在机械条件允许时校正零位。

官方参考：`examples/driverlib/single_core/eqep/eqep_ex2_freq_cal_interrupt`。

## 5. GPIO 与外部中断

GPIO 的复用、上下拉、资格采样和初始输出值应由 SysConfig 生成。通用 DriverLib 操作为：

```c
GPIO_writePin(LED_GPIO, 0U);
GPIO_togglePin(LED_GPIO);
uint32_t level = GPIO_readPin(FAULT_GPIO);
```

功率板的门极使能和复位必须给出安全的初始电平。BOOSTXL-3PHGANINV 的 `nEN_uC` 为低有效：
禁用时写高，使能时写低。外部故障输入应结合 Input XBAR、Trip XBAR 和 ePWM Trip Zone 形成
不依赖软件 ISR 延迟的硬件关断链。

官方参考：`examples/driverlib/single_core/gpio/gpio_ex1_toggle` 和
`gpio_ex2_interrupt`。

## 6. UART 与 GMP Data Link

`csp/c29x_syscfg/src/peripheral_driver.c` 已实现 GMP 的 GPIO/UART HAL。应用只需把 SysConfig
生成的 UART 基址赋给 `debug_uart`，然后初始化 Data Link：

```c
debug_uart = DEBUG_UART_BASE;
gmp_dev_dl_init(&dl);
```

后台任务轮询 `gmp_hal_uart_get_rx_available()` 并把字节交给
`gmp_dev_dl_push_byte()`；出现 `GMP_DL_EVENT_TX_RDY` 时依次发送帧头和负载，再调用
`gmp_dev_dl_tx_state_done()`。不要在控制 ISR 中执行阻塞发送，也不要让文本日志和二进制 Data
Link 同时占用同一个 UART。

官方参考：`examples/driverlib/single_core/uart/uart_ex4_echoback`。

## 7. SPI、I2C、MCAN 与 FSI

- SPI：C29x CSP 已实现 `gmp_hal_spi_bus_write/read/transfer()`，采用 8 位帧和有界轮询。
  在 SysConfig 中设置控制器/外设模式、时钟极性/相位、字长、位率和 FIFO。先用
  `spi_ex1_loopback` 验证帧格式，再参考 `spi_ex4_external_loopback_fifo_interrupts` 验证外部连线。
- I2C：C29x CSP 已实现 GMP 的 command/register/memory 读写端口，按 MSB-first 序列化地址
  和数据，并处理 NACK、仲裁丢失、总线忙和超时。配置本机/目标地址、总线速率和 FIFO；
  参考 `i2c_ex1_loopback` 与 `i2c_ex3_external_loopback`。
- MCAN：先确定经典 CAN 或 CAN FD、仲裁/数据阶段位时序、Message RAM 分区和过滤器。参考
  `mcan_ex1_loopback_interrupts` 与 `mcan_ex2_loopback_polling`。
- FSI：适合 C2000/C29x 间确定性高速链路。配置帧类型、lane、触发、标签和错误检测，并实现超时恢复。
  参考 `fsi_ex1_loopback_cpucontrol` 与 `fsi_ex2_periodic_frame`。

这些通信外设的 ISR 只搬运 FIFO、记录错误并清标志；协议解析、重试和 GMP 服务应放在后台任务。

## 8. 新工程检查表

1. 使用 CCS 21 或更新版本，并选择 `GMP-Core-C29x` Product；C29x 不能用 CCS 18 构建。
2. SysConfig 的器件、封装、CPU context 与实际芯片一致。
3. 上电后功率 PWM 保持 OST Trip，门极处于禁用电平。
4. 示波器先确认 PWM 频率、互补极性、死区和 ADC 触发位置。
5. 再检查 ADC 原始码、偏置、标度、QEP 方向与每转计数。
6. 最后分级启用开环、电流环、编码器和速度环；任何故障都立即回到安全输出。
