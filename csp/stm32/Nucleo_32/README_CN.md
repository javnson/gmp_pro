# GMP STM32 Nucleo-32 通用平台规格

[English](README.md) | **简体中文**

状态：NUCLEO-G431KB 实板基线 v0.1

日期：2026-09-12

本文规定 `csp/stm32/Nucleo_32` 的目录结构、硬件资源合同、SDPE 板卡元件、
公共 `user`/`xplt` 代码边界、生成流程和验证标准。目标与 `Nucleo_64` 一致：
应用通过选择 SDPE 板卡元件迁移，不在公共平台代码中写死 HAL 句柄或 GPIO。

本文中的“必须”“应当”“可以”分别对应 MUST、SHOULD、MAY。

## 1. 范围和原则

本目录面向采用 32 引脚 STM32、兼容 Arduino Nano V3 连接器的 Nucleo-32
板卡。每块板卡必须拥有一个主 IOC、一个 SDPE 目标输入、一份引脚表和一份按实物
记录的验证报告。

权威源边界如下：

- IOC 负责引脚、时钟、DMA、中断和外设初始化；
- `ctl/hardware_preset/sdpe_src/mcu_board` 中的板卡实体负责稳定资源别名；
- `sdpe_requirement.json` 负责目标选项和应用相关数值；
- `src/user`、`src/xplt`、`src/gmp_src_mgr` 在本目录的板卡间共享；
- CubeMX、SDPE、source manager 和 CMake 输出均为派生物，不作为手工修复源。

编译、仿真和实板验证必须分别记录。一个 32 引脚器件不因 MCU 外设列表完整就自动
具备全部控制能力；封装和板级连线必须同时满足资源合同。

## 2. 独立能力声明

| 能力 | 合同 |
| --- | --- |
| `base` | ST-Link VCP、板载状态 LED、1 ms Tick，以及一组外接 I²C |
| `control` | 三对互补 PWM、PWM 触发的至少六路 ADC、QEP ABZ |
| `dac` | 至少一路片内 DAC 可安全外引且不占用基线 ADC |
| `can` | CAN/FDCAN TX/RX 可外引；无板载收发器时必须注明 |
| `pwm_break` | 高级定时器 Break 可外引且在 IOC 中配置 |

所有正式板卡必须满足 `base`。其余能力逐项为 0 或 1，不得由线性“等级”推断。
同一引脚的不同复用功能不能同时声明为可用能力。

## 3. 必需硬件资源合同

### 3.1 PWM 与控制采样

`control` 板卡必须使用 TIM1 或 TIM8 的 CH1/1N、CH2/2N、CH3/3N，采用中心对齐
计数、明确死区/极性/空闲状态，并通过硬件触发 ADC。控制 ISR 必须在 ADC 结果完成后
调用 `gmp_base_ctl_step()`。上电、初始化和普通 Data Link 验收期间，CCER 输出位和
BDTR.MOE 必须保持关闭。

### 3.2 QEP ABZ

优先使用 TIM3 或 TIM4 的编码器模式。A/B 必须为同一定时器 CH1/CH2；Z 可以使用
原生 ETR Index，也可以使用具备相同公共语义的 GPIO EXTI。引脚必须外引。

### 3.3 ADC

固定逻辑顺序为 `GMP_ADC_FB0` 至 `GMP_ADC_FB5`。每路必须具有唯一引脚、ADC 实例
和转换 rank；IOC、SDPE、`ctl_input_callback()` 与应用端不得改变顺序。优先使用
注入组；资源不足时可以使用定时器触发的规则组循环 DMA，但必须在实体中声明。

### 3.4 Data Link、LED、I²C 与 Tick

- Data Link 必须使用实际连接板载 ST-Link VCP 的 UART，8-N-1；RX 为循环 DMA，
  TX 为普通 DMA 或不进入控制 ISR 的非阻塞实现；
- 115200 baud 为兼容基线，921600 baud 必须逐板实测；
- LED 实体必须声明端口、引脚和有效电平；
- I²C 引脚由板卡实体声明，非标准 Nano SDA/SCL 映射必须进入引脚表；
- HAL Tick 维持 1 ms，控制频率只由 PWM/ADC 快速路径推进。

### 3.5 可选能力

DAC、CAN/FDCAN 和 Break 只有在基线拓扑中无复用冲突且真实外引时才可声明。
需要交换资源时，应建立名称明确的应用变体，并保留本主 IOC 的合同不变。

## 4. SDPE 合同

通用 schema 为：

```text
ctl/hardware_preset/sdpe_schemas/stm32_nucleo_32_board.json
```

板卡实体位于：

```text
ctl/hardware_preset/sdpe_src/mcu_board/<board_id>.json
```

公共代码只使用 `GMP_NUCLEO_*` 稳定宏，包括板卡/MCU 标识、时钟、PWM、QEP、
六路 ADC、DL UART/DMA、LED、I²C 和各 `HAS_*`。可选外设的 `HAS_*` 必须始终
定义为 0 或 1。

## 5. 公共源码合同

`src/user` 提供安全的最小 GMP 应用、调度器、心跳和 Data Link 验收设施。
`src/xplt` 绑定 HAL 句柄、启动 ADC/QEP/UART DMA，并实现快速输入、输出和关断。
公共源码不得按板卡 ID 分支，不得直接写 `huart2`、`GPIOB` 等具体资源名。

启用 PWM 前必须先装载安全比较值；快速关闭必须先清除 MOE 和通道输出，再处理
日志或慢速状态。默认验收固件不提供远程开启功率输出的命令。

## 6. 目录形式

```text
csp/stm32/Nucleo_32/
├── README.md / README_CN.md
├── start_sdpe.bat
├── src/
│   ├── user/
│   ├── xplt/
│   └── gmp_src_mgr/
├── tools/validate_ioc.py
└── stm32g431kb_nucleo/
    ├── stm32g431kb_nucleo.ioc
    ├── sdpe_mgr/sdpe_requirement.json
    ├── pin_assign.md
    ├── validation.md
    ├── generate.ps1 / build.ps1 / flash.ps1
    ├── CMakeLists.txt / cmake/
    └── smoke_test.py
```

每板只维护一个首选主 IOC。只有真实的引脚复用交换才允许增加后缀变体。

## 7. 生成和构建

规范顺序为：

```text
IOC/SDPE 静态校验
  -> 生成板卡实体与目标设置
  -> source manager 先生成头文件、再生成源文件
  -> CubeMX 从 IOC 再生成 Core/Drivers
  -> 注入 GMP 入口
  -> GCC/CMake 构建
```

必须先设置 `GMP_PRO_LOCATION`。Firmware Package 必须固定版本，禁止把
`LastFirmware` 作为可复现合同。工程内路径必须相对于仓库或板卡目录。

## 8. 静态校验

`tools/validate_ioc.py` 至少检查器件/封装/schema、IOC 权威路径、PWM 六路输出、
QEP ABZ、ADC 六路顺序与触发、VCP/DMA、LED、I²C、时钟来源、固定固件包以及
绝对路径污染。连接器编号和焊桥等 IOC 不能证明的信息由 `pin_assign.md` 与实板
报告共同确认。

## 9. NUCLEO-G431KB 基线

首块板卡实体为 `nucleo_g431kb`。它声明 `base=1`、`control=1`，而
`dac=0`、`can=0`、`pwm_break=0`。基线使用：

- HSI16→PLL 产生 170 MHz，释放 PF0 作为 TIM1_CH3N；
- TIM1 三对互补 PWM，中心对齐 20 kHz，CH4 内部触发 ADC；
- ADC1 两路 + ADC2 四路注入转换，覆盖 Nano A0 至 A5；
- TIM3 PB4/PB5/PB3 提供 ABZ；
- USART2 PA2/PA3 连接 ST-Link VCP；PB8 驭动 LD2；
- I2C1 使用 PA15/PB7，外部设备需要上拉。

详细物理映射与 PF0/HSE 焊桥约束见
`stm32g431kb_nucleo/pin_assign.md`。

## 10. 单板验收

每块板卡至少完成：SDPE 与 IOC 校验、CubeMX 再生成、GCC 完整链接、SWD 写入/校验、
LED/Tick、Data Link 帧与 CRC 恢复、PIL/参数/内存/Scope、控制 ISR 频率，以及 PWM
保持禁用的寄存器证据。ADC 校准、PWM 波形、QEP、I²C、DAC/CAN 等需要外部设备的
测试必须单独声明是否完成，不得用编译结果代替。

G431KB 的可复现入口：

```powershell
cd csp/stm32/Nucleo_32/stm32g431kb_nucleo
.\build.ps1 -Configuration Release
.\flash.ps1
python .\smoke_test.py --port COM73 --baudrate 921600
```
