# GMP STM32 Nucleo-64 通用平台规格与实施计划

[English](README.md) | **简体中文**

状态：G431 基线实施中 v0.2

日期：2026-09-10

本文规定 `csp/stm32/Nucleo_64` 的目录结构、硬件资源合同、SDPE 板卡元件、公共
`user`/`xplt` 代码边界、生成流程和验证标准。目标是让一个控制应用只通过切换
SDPE 板卡元件，就能在不同 Nucleo-64 板卡之间迁移，而不修改公共平台源代码。

本文中的“必须”“应当”“可以”分别对应 MUST、SHOULD、MAY。

当前已落地 G431 第一阶段基线：通用 SDPE schema、`nucleo_g431rb` 板卡实体、
可切换 TIM1/TIM8 与 TIM3/TIM4 的目标绑定、G431RB IOC、共享 user/xplt 源、共享
source-manager 配置和 IOC 静态校验器。该目标当前状态为 `configured`；SDPE 校验、
目标生成、IOC 校验和共享源语法检查已通过，但尚未执行 CubeMX 再生成、完整链接或
实板测试，详见 `stm32g431rb_nucleo/validation.md`。

## 1. 范围和设计原则

本目录面向采用 64 引脚 STM32、兼容 Arduino Uno V3 和/或 ST Morpho 连接器的
Nucleo-64 板卡。板卡进入目录不代表其自动满足完整控制能力；支持状态必须由能力和
验证记录共同决定。

设计遵循以下原则：

1. 每块板卡有一个独立子目录和一个主 `.ioc` 文件。
2. 所有板卡共用一套 `src/user`、`src/xplt` 和 `src/gmp_src_mgr`。
3. `.ioc` 是引脚、时钟、DMA、中断和外设配置的权威实现。
4. SDPE 板卡元件负责把板卡专用句柄、实例和能力转换为统一宏。
5. 公共代码只能使用统一宏，不得直接写 `huart2`、`htim1`、`GPIOA` 等板卡名称。
6. 控制频率、死区、反馈增益和物理方向属于应用/目标 SDPE 配置，不属于 Nucleo
   板卡元件。
7. CubeMX、SDPE 和 GMP source manager 的生成物不是主仓库中的权威源文件。
8. 编译通过、仿真通过和实板通过是三种不同的验证状态，不得互相替代。

## 2. 平台能力等级

每块板卡必须声明以下能力。能力相互独立，不能用一个线性等级代替。

| 能力 | 含义 | 要求 |
| --- | --- | --- |
| `base` | GMP 基础运行 | DL 串口、用户 LED、I2C、1 ms 系统 Tick |
| `control` | 控制基础 | 高级 PWM、QEP ABZ、至少 6 路 ADC、PWM 触发 ADC |
| `dac` | 模拟调试输出 | 至少一路片内 DAC 引到外部连接器 |
| `can` | CAN 通信 | CAN/FDCAN TX/RX 引到外部连接器 |
| `pwm_break` | 硬件保护 | 高级定时器 Break 输入引到外部连接器 |

所有正式支持的板卡必须满足 `base`。不满足 `control` 的板卡可以保留为基础开发板，
但不得标记为控制就绪。`dac`、`can` 和 `pwm_break` 均为可选能力。

## 3. 必需硬件资源合同

### 3.1 高级 PWM 定时器

板卡必须选择 TIM1 或 TIM8 作为主控制 PWM 定时器，并满足：

- 使用 CH1/CH1N、CH2/CH2N、CH3/CH3N 三对互补输出；
- 使用中心对齐计数模式；
- 支持统一装载三相比较值；
- 支持死区、输出极性和空闲状态配置；
- 产生 ADC 注入组硬件触发信号；
- 上电和初始化阶段保持功率输出关闭；
- 若 Break 引脚可以外引，应配置为硬件保护输入并声明 `pwm_break` 能力。

TIM1 与 TIM8 的选择由板卡 SDPE 参数完成。公共 `xplt` 不得用板卡 ID 分支选择
定时器，也不得假设主定时器固定为 TIM1。

推荐 CubeMX 信号标签：

```text
GMP_PWM_U_H    GMP_PWM_U_L
GMP_PWM_V_H    GMP_PWM_V_L
GMP_PWM_W_H    GMP_PWM_W_L
GMP_PWM_BREAK
```

PWM 频率、死区时间、输出极性和采样点必须由目标工程 SDPE 参数给出。IOC 应包含
可生成和可启动的完整定时器拓扑；公共代码只做参数装载、启动和安全使能，不应重新
构造一套与 IOC 不一致的定时器拓扑。

### 3.2 QEP ABZ 定时器

板卡必须选择 TIM3 或 TIM4 作为 QEP 定时器：

- A、B 分别连接定时器 CH1、CH2，并使用 Encoder Mode；
- Z 连接一个支持 EXTI 的 GPIO；
- Z 中断负责清零或锁存计数器，具体行为由公共接口定义；
- 计数器位宽、输入滤波和计数方向必须在板卡资料中记录；
- QEP 引脚必须实际引到 Arduino 或 ST Morpho 连接器。

如果某个 MCU 支持原生 Index 功能，可以使用原生实现，但必须保持与 GPIO+EXTI
实现相同的公共语义。

推荐标签：

```text
GMP_QEP_A
GMP_QEP_B
GMP_QEP_Z
```

### 3.3 ADC 反馈通道

每块 `control` 板卡必须至少提供六个固定、外部可访问的 ADC 反馈通道：

```text
GMP_ADC_FB0
GMP_ADC_FB1
GMP_ADC_FB2
GMP_ADC_FB3
GMP_ADC_FB4
GMP_ADC_FB5
```

要求如下：

- 优先保留 Arduino A0 至 A5 的兼容关系；
- 如果使用其他 Morpho 引脚，必须在 `pin_assign.md` 中明确说明；
- 六个通道的逻辑顺序在 IOC、SDPE、`ctl_input_callback()` 和应用输入端必须一致；
- ADC 必须由所选 TIM1/TIM8 硬件触发；
- 控制采样优先使用注入转换组；
- 可以按照 3+3、4+2 或适合目标 MCU 的方式分配到多个 ADC 实例；
- 额外通道从 `GMP_ADC_AUX0` 开始命名，不改变 FB0 至 FB5 的顺序；
- ADC 分辨率、参考电压、采样时间和同步方式必须进入生成配置或验证记录。

控制 ISR 应在 ADC 结果就绪后调用 `gmp_base_ctl_step()`，不应仅因定时器更新就读取
尚未完成的 ADC 数据。

### 3.4 GMP Data Link 串口

每块板卡必须选择一路与板载 ST-Link 虚拟串口相连的 USART、UART 或 LPUART：

- 线路必须确实连接到 ST-Link VCP，而不只是引到连接器；
- 数据格式为 8-N-1；
- 115200 baud 是基础兼容速率；
- 921600 baud 是推荐高速验证项，必须逐板确认时钟误差和稳定性；
- RX 应使用循环 DMA，并处理 Half Complete、Complete 和 UART Idle；
- TX 可以使用 DMA；若使用阻塞发送，不得在控制 ISR 中调用；
- USART 实例差异通过 SDPE 统一宏隐藏。

推荐标签：

```text
GMP_DL_TX
GMP_DL_RX
```

### 3.5 用户 LED

每块板卡必须提供一个板载用户 LED：

- 推荐标签为 `GMP_STATUS_LED`；
- SDPE 必须声明 GPIO 端口、引脚、点亮电平和熄灭电平；
- 基础测试固件使用 LED 表示启动、正常心跳和故障状态；
- 公共代码不得假设 LED 一定高电平点亮。

### 3.6 I2C

每块板卡必须提供一路 I2C：

- 优先使用 Arduino D14/SDA 和 D15/SCL；
- 必须记录对应的 I2C 实例、GPIO 和复用功能；
- 基础配置应支持 100 kHz 和 400 kHz；
- 板卡资料必须说明外部设备是否需要额外上拉电阻；
- 中断和 DMA 不是第一版强制要求。

推荐标签：

```text
GMP_I2C_SDA
GMP_I2C_SCL
```

### 3.7 系统 Tick

系统 Tick 必须提供 1 ms 语义：

- STM32 平台默认由 HAL Tick 提供；
- `gmp_base_get_system_tick()` 必须与 `HAL_GetTick()` 的时间基准一致；
- 控制步频由 PWM/ADC 快速路径决定，与 1 ms 系统 Tick 分离；
- 禁止同时由 HAL Tick 和 PWM ISR 重复推进同一个系统 Tick；
- 如果板卡改用 TIM 作为 HAL timebase，必须在板卡文档中记录。

### 3.8 可选 DAC

存在片内 DAC 且引脚无冲突时，可以声明 `dac`：

- 至少一路输出必须引到外部连接器；
- 统一宏必须给出 DAC 句柄、通道数和各通道 ID；
- 公共测试支持输出固定值、斜坡和选择后的内部观察量；
- PWM 加 RC 滤波形成的输出不得标记为片内 DAC。

### 3.9 可选 CAN/FDCAN

存在 CAN 或 FDCAN 且 TX/RX 引脚无冲突时，可以声明 `can`：

- SDPE 必须区分经典 CAN 和 FDCAN；
- 必须记录句柄、实例、TX/RX 引脚和连接器位置；
- 必须说明 Nucleo 板上是否存在收发器；没有收发器时明确要求外接收发器；
- 第一阶段先通过内部回环，硬件支持标记要求再通过外部总线测试；
- CAN 与 PWM/ADC 引脚冲突时才允许创建单独 IOC 变体。

## 4. SDPE 板卡元件合同

### 4.1 新建通用 schema

应新增通用 `stm32_nucleo_64_board` schema，而不是直接扩充现有的
`stm32_motor_control_board`。现有 schema 含门极使能和电机编码器等应用假设，不能
完整描述通用 Nucleo-64 平台。

建议源文件位置：

```text
ctl/hardware_preset/sdpe_schemas/stm32_nucleo_64_board.json
ctl/hardware_preset/sdpe_src/mcu_board/<board_id>.json
```

每个板卡实体必须引用其 IOC 文件，并描述能力、句柄和统一逻辑通道。板卡实体属于
硬件资源预设；PWM 频率、ADC 增益、电机参数等仍属于使用该板卡的目标工程。

### 4.2 公共代码需要的统一宏

生成的目标设置必须解析出下列宏。可选外设的 `HAS_*` 宏必须始终定义为 0 或 1；
公共代码通过编译期分支移除未使用功能。

| 领域 | 必需统一宏 |
| --- | --- |
| 标识 | `GMP_NUCLEO_BOARD_ID`, `GMP_NUCLEO_MCU_ID` |
| 时钟 | `GMP_NUCLEO_SYSTEM_CLOCK_HZ`, `GMP_NUCLEO_SYSTEM_TICK_HZ` |
| PWM | `GMP_NUCLEO_PWM_TIMER_HANDLE`, `GMP_NUCLEO_PWM_TIMER_INSTANCE`, `GMP_NUCLEO_PWM_ADC_TRIGGER` |
| QEP | `GMP_NUCLEO_QEP_TIMER_HANDLE`, `GMP_NUCLEO_QEP_TIMER_INSTANCE`, `GMP_NUCLEO_QEP_Z_PORT`, `GMP_NUCLEO_QEP_Z_PIN` |
| ADC | `GMP_NUCLEO_ADC_FB_COUNT`, `GMP_NUCLEO_ADC_FB<n>_HANDLE`, `GMP_NUCLEO_ADC_FB<n>_RANK` |
| DL | `GMP_NUCLEO_DL_UART_HANDLE`, `GMP_NUCLEO_DL_UART_INSTANCE`, `GMP_NUCLEO_DL_RX_DMA_HANDLE`, `GMP_NUCLEO_DL_BAUD_RATE` |
| LED | `GMP_NUCLEO_STATUS_LED_PORT`, `GMP_NUCLEO_STATUS_LED_PIN`, `GMP_NUCLEO_STATUS_LED_ON`, `GMP_NUCLEO_STATUS_LED_OFF` |
| I2C | `GMP_NUCLEO_I2C_HANDLE`, `GMP_NUCLEO_I2C_INSTANCE` |
| DAC | `GMP_NUCLEO_HAS_DAC`，启用时提供句柄和通道宏 |
| CAN | `GMP_NUCLEO_HAS_CAN`，启用时提供类型、句柄和实例宏 |
| Break | `GMP_NUCLEO_HAS_PWM_BREAK` |

`<n>` 至少覆盖 0 至 5。若一个板卡需要额外别名，必须保持这些基础宏不变。

### 4.3 每板 SDPE 工程

每个 IOC 子目录包含独立的 `sdpe_mgr/sdpe_requirement.json`，负责：

1. 选择一个 Nucleo-64 板卡实体；
2. 选择该板允许的 TIM1 或 TIM8；
3. 生成公共 `xplt` 所需的统一宏；
4. 声明 DAC、CAN 和 Break 的可用性；
5. 输出板卡专用目标设置供编译使用。

主仓库只把 `sdpe_requirement.json` 和硬件预设 JSON 作为权威输入。生成的头文件和
MATLAB 初始化文件可以忽略；导出为独立工程时再一并提交生成结果。

## 5. 公共源代码合同

### 5.1 `src/user`

公共用户代码提供最小、与板卡无关的 GMP 应用：

- `user_main.c/.h`：调度器、LED 心跳、DL 后台服务和可选 CAN 服务；
- `user_dl.c/.h`：GMP Data Link 实例和设施注册；
- `ctl_main.c/.h`：安全的测试控制器和可观察变量；
- 默认不使能功率输出；只有显式测试状态才允许启动 PWM；
- 不直接包含某个 STM32 系列的 HAL 头文件；
- 不直接引用板卡专用句柄或 GPIO。

### 5.2 `src/xplt`

公共平台代码建议包含：

```text
xplt.config.h
xplt.peripheral.h
xplt.peripheral.c
xplt.ctl_interface.h
```

职责如下：

- `xplt.config.h`：组合 GMP 功能选择和 SDPE 生成设置；
- `xplt.peripheral.*`：绑定句柄、ADC 校准和启动、UART DMA、QEP、LED、I2C 以及可选外设；
- `xplt.ctl_interface.h`：读取六路 ADC、写入三相 PWM、快速使能和快速关闭；
- ADC 注入转换完成回调调用 `gmp_base_ctl_step()`；
- 快速关闭路径不得阻塞，不得等待 UART，也不得发布旧 PWM 比较值；
- 可选 DAC/CAN 使用 `#if GMP_NUCLEO_HAS_*` 编译期裁剪；
- 板卡差异只能来自 SDPE 统一宏和 CubeMX HAL 句柄。

### 5.3 `src/gmp_src_mgr`

该目录在所有板卡之间共享：

- `gmp_framework_config.json` 是本平台的项目级模块选择源；
- source manager BAT/SH 脚本来自仓库规范模板，不在此目录单独维护；
- 先生成 `gmp_inc`，再生成 `gmp_src`；
- `gmp_inc`、`gmp_src` 和机器相关 include 列表为生成物；
- 第一版只选择公共示例实际需要的 GMP、CTL、scheduler 和 Data Link 模块；
- 必须检查扁平化 C/C++ 源文件名无碰撞。

## 6. 目录结构

目标目录结构如下：

```text
csp/stm32/Nucleo_64/
├── README.md
├── README_CN.md
├── src/
│   ├── user/
│   ├── xplt/
│   └── gmp_src_mgr/
├── tools/
│   ├── validate_ioc.py
│   ├── generate_board.ps1
│   └── build_all.ps1
├── stm32g431rb_nucleo/
│   ├── stm32g431rb_nucleo.ioc
│   ├── pin_assign.md
│   ├── validation.md
│   ├── .gitignore
│   └── sdpe_mgr/
│       └── sdpe_requirement.json
├── stm32g474re_nucleo/
└── stm32g491re_nucleo/
```

每个板卡目录只维护一个首选主 IOC。只有无法通过 SDPE 宏解决的真实引脚复用冲突，
才允许增加带后缀的 IOC，例如 `*_tim8.ioc` 或 `*_can.ioc`。不得为 DAC 开关、CAN
开关和编译器选择建立组合式 IOC 矩阵。

## 7. 生成与工程集成

推荐的统一流程为：

```text
选择板卡子目录
  -> 运行该板 SDPE target generation
  -> 运行公共 gmp_src_mgr generation
  -> CubeMX headless 生成临时工程
  -> 注入公共 user/xplt 和相对 include path
  -> CMake/GCC 构建
  -> 可选生成 Keil 工程
  -> 静态检查预期输出
```

要求：

- 设置 `GMP_PRO_LOCATION` 后才能运行 GMP 生成工具；
- 固定支持的 CubeMX 版本和每个 MCU 系列的 Firmware Package；
- 不使用 `LastFirmware` 作为可复现构建合同；
- GCC/CMake 是自动验证基线；Keil/IAR 是可选生成目标；
- 不为不同编译器复制 IOC；
- 生成脚本必须检查预期的 `main.c`、HAL 初始化源、链接脚本和最终 ELF，不能只依赖
  CubeMX 或生成器退出码；
- 项目 include path 必须使用工程相对路径，不嵌入开发者机器绝对路径。

## 8. 静态验证规则

`tools/validate_ioc.py` 应当读取 IOC 和对应 SDPE 输入，至少检查：

1. IOC 中的 board、MCU、封装与 SDPE 实体一致；
2. 必需 IP 包含高级定时器、QEP 定时器、ADC、DL 串口、DMA、I2C、GPIO；
3. 三对互补 PWM 信号完整且没有复用冲突；
4. FB0 至 FB5 六路 ADC 都有唯一物理引脚、ADC 实例和转换 rank；
5. ADC 外部触发来自所选 TIM1/TIM8；
6. QEP A/B 属于同一个 TIM3/TIM4 的 CH1/CH2，Z 具有 EXTI；
7. DL 串口引脚与板载 ST-Link VCP 设计一致；
8. UART RX DMA 为循环模式；
9. LED、I2C、DAC、CAN 和 Break 能力与 IOC 实现一致；
10. 所有资源都能在 `pin_assign.md` 找到连接器映射；
11. CubeMX 与 Firmware Package 版本属于支持矩阵；
12. 生成设置不包含仓库绝对路径。

无法从 IOC 单独证明的板级连线，例如 ST-Link VCP 路由和连接器编号，应由经过审核的
板卡 SDPE 实体和 `pin_assign.md` 提供，并在实板验证中确认。

## 9. 公共测试固件与安全行为

公共测试固件分阶段启用功能：

1. 启动后立即保持 PWM 输出关闭；
2. LED 显示启动状态，然后进入固定心跳；
3. 启动 DL RX DMA 并提供板卡信息、回环和变量观察；
4. 启动 QEP 计数，但不影响输出；
5. 启动 ADC 触发采样并允许读取六路原始值；
6. 只有收到显式测试命令后才输出受限占空比 PWM；
7. 超时、故障或通信失效后立即关闭输出；
8. 可选 DAC 输出选定观察量；
9. 可选 CAN 先进行内部回环，再进行外部总线测试。

任何 `ctl_fast_enable_output()` 调用前必须写入安全比较值。`ctl_fast_disable_output()`
必须优先关闭高级定时器输出，再处理慢速状态和日志。

## 10. 单板验收标准

### 10.1 配置和构建

- SDPE 生成成功，并人工检查生成宏；
- GMP source manager 生成成功，并检查预期头文件和源文件；
- CubeMX headless 生成成功；
- GCC/CMake Debug 构建成功；
- 可选工具链构建结果分别记录；
- `git diff --check` 和 IOC 静态验证通过。

### 10.2 基础硬件

- LED 启动状态和 1 Hz 心跳正确；
- 1 ms Tick 长期运行无明显漂移或重复计数；
- DL 在 115200 baud 完成回环和基本设施测试；
- 若声明高速能力，921600 baud 完成压力测试；
- I2C 在 100 kHz 和 400 kHz 与已知设备通信成功。

### 10.3 控制硬件

- 示波器确认三对互补 PWM、中心对齐、频率、极性和死区；
- 启动、停止和重复使能过程中没有窄脉冲或旧占空比输出；
- 六路 ADC 逐通道注入已知电压，验证顺序、量程和触发时刻；
- ADC ISR 周期与 PWM 周期关系正确；
- QEP A/B 方向、计数和 Z 清零/锁存行为正确；
- 若声明 Break，外部触发能够在不依赖软件的情况下关闭 PWM。

### 10.4 可选硬件

- DAC 固定值和斜坡输出通过；
- CAN/FDCAN 内部回环通过；
- 外接收发器后的双节点通信通过后，才标记 CAN 实板验证完成。

`validation.md` 必须记录板卡修订、CubeMX、Firmware Package、编译器、测试日期、
仪器和测试范围。未连接真实硬件时只能标记编译验证，不能标记硬件验证。

## 11. 分阶段实施计划

### 阶段 A：目录与规格基线

1. 审核并冻结本文的公共资源、宏和目录合同；
2. 创建公共 `src/user`、`src/xplt`、`src/gmp_src_mgr` 骨架；
3. 建立板卡子目录、`pin_assign.md` 和 `validation.md` 模板；
4. 固定首版 CubeMX 和 Firmware Package 版本；
5. 确认生成物 `.gitignore` 策略。

完成条件：不依赖具体板卡的公共头文件可以通过预处理检查，目录中没有复制的生成物或
C2000 遗留代码。

### 阶段 B：SDPE 板卡模型

1. 新增 `stm32_nucleo_64_board` schema；
2. 定义本文第 4.2 节的统一宏；
3. 创建 G431RB 板卡实体和目标 `sdpe_requirement.json`；
4. 生成并检查目标设置头文件；
5. 增加 IOC/SDPE 一致性检查。

完成条件：切换 SDPE 实体即可改变所有板卡句柄，公共源文件无差异。

### 阶段 C：G431RB 黄金板

1. 以现有 G431 控制工程为参考重新建立目录内 IOC；
2. 配置 TIM1/TIM8、TIM3/TIM4、六路以上 ADC、USART2 VCP、I2C、LED；
3. 配置可用的 DAC、FDCAN 和 Break；
4. 完成公共 user/xplt、DL 和控制回调；
5. 建立 headless generation、CMake 构建和硬件 smoke test；
6. 完成 `pin_assign.md` 和 `validation.md`。

完成条件：G431RB 满足第 10 节全部适用项，并成为后续板卡的黄金参考。

### 阶段 D：首批同系列板卡

按顺序适配：

1. NUCLEO-G474RE；
2. NUCLEO-G491RE。

重点验证公共代码是否真的不需要修改。若增加第二块板时必须在公共代码中按板卡 ID
分支，应回到 SDPE 宏合同修正抽象。

完成条件：三个 G4 工程使用同一份 `src/user`、`src/xplt`、`src/gmp_src_mgr`，并分别
完成生成、编译和实板验证。

### 阶段 E：跨系列扩展

建议顺序：

1. NUCLEO-F302R8，验证旧系列高级定时器和 ADC 差异；
2. NUCLEO-C092RC、NUCLEO-U083RC，评估低资源器件是否满足完整 `control`；
3. NUCLEO-H533RE，处理 M33、安全域和新 HAL 差异；
4. 根据实际板卡库存继续扩展。

每进入一个新的 STM32 系列，先完成一块黄金板和实板验证，再批量扩展同系列。

### 阶段 F：批量校验与发布

1. `validate_ioc.py` 批量检查所有板卡；
2. `build_all.ps1` 在临时目录中生成和构建全部板卡；
3. README 支持矩阵区分 configured、compiled、hardware 三种状态；
4. 对迁移完成的旧 IOC 建立重定向说明或明确弃用；
5. 将实板测试结果作为精确到板卡和配置的证据发布。

## 12. 首版交付清单

首版以三块 G4 板卡为目标，交付：

- 本规格的审核版本；
- 一个新的 Nucleo-64 SDPE schema；
- G431RB、G474RE、G491RE 三个板卡实体；
- 三个主 IOC 和三个板卡 SDPE 工程；
- 一套公共 `user`、`xplt` 和 `gmp_src_mgr`；
- IOC/SDPE 静态校验工具；
- CubeMX 无界面生成和 GCC/CMake 批量构建脚本；
- 三份引脚表和三份验证记录；
- 至少 G431RB 完整实板验证，其他板卡按实际硬件状态准确标注。

## 13. 非目标

首版不包括：

- 为每一种编译器维护不同 IOC；
- 在 Nucleo 板卡元件中固化电机、变流器或传感器物理参数；
- 保证所有 Nucleo-64 都具有 DAC、CAN 收发器或完整互补 PWM 引脚；
- 用编译结果代替示波器、ADC 注入、QEP 和总线实测；
- 在公共 `xplt` 中按板卡名称维护大量 `#if` 分支；
- 提交或手工维护 CubeMX、SDPE、source manager 的派生输出。

## 14. 现有参考

- [G431RB Nucleo 示例 IOC](../Nucleo_Example/stm32g431rb_nucleo/stm32g431rb_nucleo.ioc)：
  已有 Nucleo-64 引脚规划参考，但不满足本文完整合同；
- [G431 控制套件 IOC](../../../ctl/suite/mcs_pmsm_nt/project/stm32g431/stm32g431.ioc)：
  TIM1/TIM8、双 ADC 注入转换、UART DMA 和 FDCAN 参考；
- [STM32 Data Link 验证工程](../../../tools/gmp_datalink/stm32_dl_dbger)：STM32
  CubeMX、CMake、Keil 和 DL 硬件验证流程参考；
- [现有 STM32 电机控制板 schema](../../../ctl/hardware_preset/sdpe_schemas/stm32_motor_control_board.json)：
  作为 SDPE 模型参考，不作为通用 Nucleo-64 schema 直接复用。
