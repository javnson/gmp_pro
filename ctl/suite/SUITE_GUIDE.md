# GMP 应用解决方案套件指南 (Suite Guide)

## 文档概述

本文档详细介绍了 GMP CTL Suite 中的各类成套解决方案工程。Suite 提供了从电机控制到数字电源、从逆变器到并网系统的完整应用代码，每个解决方案都包含跨平台的算法实现和多种硬件平台的实际工程文件。

---

## 1. Suite 定位与架构

### 1.1 什么是 Suite？

**Suite（应用解决方案套件）** 是基于 GMP CTL 组件库构建的完整应用级解决方案。它提供：

- **即用型控制算法**：集成了电机控制、数字电源、并网逆变等领域的成熟算法
- **增量化调试支持**：通过 `BUILD_LEVEL` 宏实现分层次调试，从开环到闭环逐步验证
- **跨平台部署**：同一套算法可在 Simulink、TI C2000、STM32 等多种平台运行
- **完整工程模板**：包含硬件配置、外设初始化、中断服务、AT命令调试等完整代码

### 1.2 Suite 与 Component 的关系

```
├── ctl/component/               # 组件库（底层模块）
│   ├── motor_control/          # 电机控制基础模块（PID、观测器、编码器等）
│   ├── intrinsic/              # 基础算法（滤波器、生成器等）
│   ├── digital_power/          # 数字电源基础模块（GFL/GFM控制器等）
│   └── ...                     
└── ctl/suite/                   # 应用解决方案（上层集成）
    ├── mcs_pmsm/               # 使用 component 中的模块构建完整的 PMSM 驱动器
    ├── dps_boost/              # 使用 component 中的模块构建完整的 Boost 变换器
    └── pgs_3ph_GFL_inverter/   # 使用 component 中的模块构建完整的并网逆变器
```

**关系链：** `Component（零件）` → `Suite（整机）`

---

## 2. 命名规范与分类

### 2.1 命名规范

Suite 中的工程按照以下规范命名：

```
<类别前缀>_<拓扑结构>_[特性标签]
```

**示例：**
- `mcs_pmsm_smo` = Motor Control Suite（电机控制） + PMSM（永磁同步电机） + SMO（滑模观测器）
- `dps_boost` = Digital Power Suite（数字电源） + Boost（升压拓扑）
- `pgs_3ph_GFL_inverter` = Power Grid Suite（电网系统） + 3ph（三相） + GFL（跟网型）

### 2.2 三大类别

| 类别前缀 | 全称 | 应用领域 |
|---------|------|---------|
| **MCS** | Motor Control Suite | 电机驱动器：PMSM、ACM、BLDC 等电机的矢量控制 |
| **DPS** | Digital Power Suite | 数字开关电源：DC-DC 变换器、AC-DC 整流器等 |
| **PGS** | Power Grid Suite | 电网互动系统：并网逆变器、储能系统、光伏变流器等 |

---

## 3. 电机控制解决方案 (MCS)

### 3.1 MCS 概述

电机控制套件提供了多种电机类型、多种传感器方案、多种控制策略的完整实现。所有 MCS 工程均支持 **CiA402 状态机标准**，提供统一的控制接口。

### 3.2 MCS 项目列表

#### 3.2.1 永磁同步电机 (PMSM) 系列

| 项目名称 | 功能描述 | 传感器方案 | 控制策略 | 适用场景 |
|---------|---------|-----------|---------|---------|
| **mcs_pmsm** | 基础 PMSM 矢量控制 | ABZ 编码器 / AS5048A | FOC 矢量控制 | 通用伺服驱动、测试平台 |
| **mcs_pmsm_nt** | PMSM 矢量控制（新模板） | 绝对式编码器 | FOC + BUILD_LEVEL 增量调试 | 伺服系统开发模板 |
| **mcs_pmsm_smo** | 无感 PMSM 控制 | 无传感器（SMO） | FOC + 滑模观测器 | 低成本无传感器应用 |
| **mcs_pmsm_hfi** | 低速/零速 PMSM 控制 | 无传感器（HFI） | FOC + 高频注入 | 需要低速/零速启动的凸极电机 |
| **mcs_pmsm_mtpa** | 高效 PMSM 控制 | 编码器 | FOC + MTPA | 需要最大转矩/安培比的凸极电机 |

**PMSM 核心技术：**
- **FOC（磁场定向控制）**：通过 Clarke/Park 变换实现三相电流的解耦控制
- **SMO（滑模观测器）**：通过电机数学模型估算转子位置和速度
- **HFI（高频注入）**：在低速区域注入高频信号，利用凸极效应估算转子位置
- **MTPA（最大转矩/安培比）**：动态调整 $i_d$ 和 $i_q$ 分配，实现最优效率

#### 3.2.2 异步电机 (ACM) 系列

| 项目名称 | 功能描述 | 传感器方案 | 控制策略 | 适用场景 |
|---------|---------|-----------|---------|---------|
| **mcs_acm** | 基础 ACM 矢量控制 | QEP 编码器 | FOC 矢量控制 | 工业泵机、风机驱动 |
| **mcs_acm_fe** | 无感 ACM 控制 | 无传感器（磁链观测器） | FOC + 龙贝格观测器 | 低成本异步电机应用 |

**ACM 核心技术：**
- **RFOC（转子磁场定向）**：基于转子磁链定向的矢量控制
- **磁链观测器**：通过电压模型或电流模型估算转子磁链和速度

#### 3.2.3 无刷直流电机 (BLDC) 系列

| 项目名称 | 功能描述 | 传感器方案 | 控制策略 | 适用场景 |
|---------|---------|-----------|---------|---------|
| **mcs_bldc_smo** | 高性能 BLDC 控制 | Hall 传感器 + SMO | 方波驱动 + SMO 观测 | 需要精确控制的 BLDC 应用 |

### 3.3 MCS 核心模块

所有 MCS 工程都包含以下核心模块（位于 `implement/common/ctl_main.c`）：

```c
// 1. CiA402 状态机：管理电机启停、故障处理
cia402_sm_t cia402_sm;

// 2. 电流控制器：d-q 轴电流环 PI 控制
mtr_pmsm_ctrl_t mtr_ctrl;  // PMSM 电流控制器
mtr_acm_ctrl_t acm_ctrl;   // ACM 电流控制器

// 3. 运动控制器：速度环/位置环控制
mtr_motion_ctrl_t motion_ctrl;

// 4. 位置传感器：编码器、霍尔、观测器
mtr_aqep_pos_encoder_t pos_enc;    // 增量编码器
mtr_abs_encoder_t abs_enc;         // 绝对式编码器
mtr_smo_observer_t smo_obs;        // 滑模观测器
mtr_hfi_observer_t hfi_obs;        // 高频注入观测器

// 5. 速度计算：差分、滤波
mtr_spd_encoder_t spd_enc;

// 6. 调制器：SPWM/SVPWM/NPC
spwm_modulator_t spwm;

// 7. ADC 校准：自动偏置校准
ctl_adc_calibrator_t adc_calibrator;
```

### 3.4 BUILD_LEVEL 增量调试

MCS 工程支持 4 级增量调试（在 `ctrl_settings.h` 中配置）：

| BUILD_LEVEL | 功能描述 | 验证目标 | 控制方式 |
|------------|---------|---------|---------|
| **LEVEL 1** | 电压开环控制 | PWM 输出、ADC 采样、基础外设 | 直接给定 $V_d$, $V_q$ 电压 |
| **LEVEL 2** | 电流闭环（IF 模式） | 电流环 PI 参数、电流采样准确性 | 给定 $I_d$, $I_q$ 目标，角度由斜坡生成器提供 |
| **LEVEL 3** | 速度闭环（带传感器） | 编码器正确性、速度环参数 | 给定速度目标，角度由传感器提供 |
| **LEVEL 4** | 完整功能 | 完整系统调试 | CiA402 状态机、AT 命令、完整功能 |

**调试流程：**
```
LEVEL 1 → 验证硬件和 PWM → LEVEL 2 → 验证电流环 → LEVEL 3 → 验证速度环 → LEVEL 4 → 完整系统
```

---

## 4. 数字电源解决方案 (DPS)

### 4.1 DPS 概述

数字电源套件提供了常见 DC-DC 和 AC-DC 拓扑的数字控制实现，包括单级和双级拓扑、单相和三相拓扑。

### 4.2 DPS 项目列表

| 项目名称 | 拓扑类型 | 功能描述 | 控制策略 | 适用场景 |
|---------|---------|---------|---------|---------|
| **dps_boost** | DC-DC Boost | 升压变换器 | 电压外环 + 电流内环 | 光伏、燃料电池升压 |
| **dps_buck** | DC-DC Buck | 降压变换器 | 电压外环 + 电流内环 | 充电器、DC 电源 |
| **dps_dcac_rectifier_spfc** | AC-DC 整流器 | 单相功率因数校正整流 | SPFC（单相 PFC） | AC-DC 电源前端 |

### 4.3 DPS 核心模块

数字电源工程的典型控制结构：

```c
// 1. 电压外环控制器：稳定输出电压
pid_controller_t voltage_loop;

// 2. 电流内环控制器：快速电流跟踪
pid_controller_t current_loop;

// 3. 调制器：PWM 占空比生成
pwm_modulator_t pwm_mod;

// 4. ADC 校准器：电压/电流采样校准
ctl_adc_calibrator_t adc_calibrator;

// 5. 前馈控制：提高动态响应
feedforward_ctrl_t feedforward;
```

### 4.4 DPS 控制框架

典型的双环控制结构：

```
输入电压 ────────┐
                │
输出电压反馈 ──→ [电压外环 PI] ──→ 电流参考 ──→ [电流内环 PI] ──→ [PWM 调制器] ──→ 功率开关
                                                      ↑
                                    电流反馈 ─────────┘
```

**特点：**
- **电压外环**：带宽较低（几十 Hz），稳定输出电压
- **电流内环**：带宽较高（几 kHz），快速响应，提供过流保护

---

## 5. 电网互动解决方案 (PGS)

### 5.1 PGS 概述

电网系统套件提供了并网逆变器、离网逆变器、光伏储能系统的完整实现，支持单相和三相拓扑。

### 5.2 PGS 项目列表

| 项目名称 | 拓扑类型 | 功能描述 | 控制策略 | 适用场景 |
|---------|---------|---------|---------|---------|
| **pgs_1ph_dcac_inv** | 单相离网逆变器 | 单相 DC-AC 逆变 | 电压环 + 电流环 | UPS、离网电源 |
| **pgs_1ph_dcac_dual_stage_rectifier** | 单相双级整流器 | PFC + DC-DC 双级拓扑 | SPFC + Buck/Boost | 单相 AC-DC 变换 |
| **pgs_3ph_GFL_inverter** | 三相跟网型逆变器 | 三相并网逆变（跟网） | GFL（跟网型） | 光伏并网、储能并网 |
| **pgs_3ph_grid_inverter** | 三相并网逆变器 | 三相并网逆变（通用） | PQ 功率控制 / 电流控制 | 新能源并网系统 |
| **pgs_dc_photovoltaic_storage** | 光伏储能系统 | 光伏 + 储能 + 并网 | MPPT + 储能管理 + 并网控制 | 光伏储能一体化系统 |

### 5.3 PGS 核心技术

#### 5.3.1 GFL（跟网型）与 GFM（构网型）

| 特性 | GFL (Grid Following) | GFM (Grid Forming) |
|-----|---------------------|-------------------|
| **定位** | 跟随电网 | 构建电网 |
| **核心控制** | PLL 锁相 + 电流控制 | 电压源控制 + 虚拟同步机 |
| **适用场景** | 并网运行（电网强） | 离网运行 / 弱电网 / 微网 |
| **功率源** | 需要电网提供参考 | 自身作为电压源 |

#### 5.3.2 PLL（锁相环）

用于跟网型逆变器的电网同步：

```c
// PLL 观测器：估算电网频率和相位
pll_observer_t pll_obs;

// 初始化
ctl_pll_observer_init(&pll_obs, pll_params);

// 每个控制周期调用
ctl_pll_observer_step(&pll_obs, v_alpha, v_beta);

// 获取电网角度和频率
ctrl_gt theta = ctl_pll_observer_get_theta(&pll_obs);
ctrl_gt freq = ctl_pll_observer_get_freq(&pll_obs);
```

#### 5.3.3 功率控制

PGS 系统通常采用 **PQ 功率控制**：

```
有功功率参考 P* ──┐
                  ├──→ [功率控制器] ──→ [电流参考生成] ──→ [电流内环] ──→ PWM
无功功率参考 Q* ──┘                      ↑
                                      PLL锁相
```

**控制链：** `功率指令 (P, Q)` → `电流参考 (Id, Iq)` → `电压指令 (Vd, Vq)` → `PWM 波形`

### 5.4 PGS 核心模块

```c
// 1. PLL 锁相环：电网同步
pll_observer_t pll_obs;

// 2. GFL 控制器：跟网型逆变器控制
gfl_inv_ctrl_t gfl_ctrl;
gfl_pq_ctrl_t pq_ctrl;

// 3. 电流控制器：d-q 轴解耦控制
pid_controller_t current_ctrl_d;
pid_controller_t current_ctrl_q;

// 4. 调制器：SPWM / SVPWM / NPC
spwm_modulator_t spwm;

// 5. ADC 校准器
ctl_adc_calibrator_t adc_calibrator;
```

---

## 6. 工程目录结构

### 6.1 标准目录结构

每个 Suite 工程都遵循相同的目录结构：

```
<project_name>/
├── README.md              # 项目说明文档
├── doc/                   # 文档和图片资源（可选）
│   └── img/              
├── img/                   # 图片资源（可选）
├── implement/             # 控制算法实现代码
│   ├── common/            # ★ 所有平台公用的核心控制代码
│   │   ├── ctl_main.c    # ★ 控制器主逻辑（初始化、中断服务）
│   │   ├── ctl_main.h    # 控制器接口声明
│   │   ├── user_main.c   # 用户主程序（调度器、AT命令）
│   │   └── user_main.h   
│   ├── f280039c_iris_node/  # TI C2000 F280039C 平台特化代码
│   │   ├── ctrl_settings.h  # ★ 控制参数配置（BUILD_LEVEL、PWM频率等）
│   │   ├── xplt.config.h    # ★ 平台配置（引脚、外设）
│   │   ├── xplt.ctl_interface.h  # ★ 控制接口回调函数
│   │   └── xplt.peripheral.c/h   # 外设初始化代码
│   ├── f280049c/          # TI C2000 F280049C 平台特化代码
│   ├── stm32g431/         # STM32G431 平台特化代码
│   ├── stm32f407/         # STM32F407 平台特化代码
│   └── simulate/          # PC 仿真（Simulink/Visual Studio）特化代码
└── project/               # 各平台的工程文件
    ├── f280039c_Iris_node/  # TI Code Composer Studio 工程
    │   ├── *.projectspec   # CCS 工程配置文件
    │   └── *.syscfg        # TI SysConfig 配置文件
    ├── f280049c/
    ├── stm32g431/           # STM32CubeIDE 工程
    │   ├── *.ioc           # STM32CubeMX 配置文件
    │   └── *.project       # Eclipse CDT 工程文件
    └── simulate/            # Simulink 仿真工程
        ├── *.slx           # Simulink 模型文件
        └── *.m             # 初始化脚本
```

### 6.2 核心文件说明

#### 6.2.1 `implement/common/` - 平台无关的核心算法

| 文件 | 功能 | 修改频率 |
|-----|------|---------|
| **ctl_main.c** | 控制器核心逻辑：对象定义、初始化、中断服务函数 | 高 |
| **ctl_main.h** | 控制器接口声明：外部可调用的函数 | 中 |
| **user_main.c** | 用户主程序：调度器、AT 命令、任务管理 | 低 |
| **user_main.h** | 用户主程序头文件 | 低 |

**关键代码结构（ctl_main.c）：**

```c
// 1. 控制对象定义
static cia402_sm_t cia402_sm;
static mtr_pmsm_ctrl_t mtr_ctrl;
static mtr_motion_ctrl_t motion_ctrl;
// ...

// 2. 初始化函数（主程序调用一次）
void ctl_init(void) {
    // 初始化所有控制对象
    ctl_cia402_sm_init(&cia402_sm, ...);
    ctl_mtr_pmsm_ctrl_init(&mtr_ctrl, ...);
    // ...
}

// 3. 控制周期中断服务函数（每个 PWM 周期调用一次）
void ctl_step(void) {
    // 1. 读取 ADC 采样值（电流、电压）
    ctl_input_callback();
    
    // 2. 执行控制算法
    ctl_mtr_pmsm_ctrl_step(&mtr_ctrl, ...);
    ctl_mtr_motion_ctrl_step(&motion_ctrl, ...);
    
    // 3. 输出 PWM 占空比
    ctl_output_callback();
}
```

#### 6.2.2 `implement/<platform>/` - 平台特化代码

| 文件 | 功能 | 修改频率 |
|-----|------|---------|
| **ctrl_settings.h** | 控制参数配置：BUILD_LEVEL、PWM 频率、电机参数 | 高 |
| **xplt.config.h** | 平台配置：引脚定义、外设映射、时钟配置 | 中 |
| **xplt.ctl_interface.h** | 控制接口：ADC 采样、PWM 输出的回调函数实现 | 中 |
| **xplt.peripheral.c/h** | 外设驱动：ADC、PWM、GPIO、UART 等初始化 | 低 |

**关键代码（xplt.ctl_interface.h）：**

```c
// 输入回调：从硬件读取 ADC 采样值
static inline void ctl_input_callback(void) {
    // 读取电流采样（示例）
    extern ctrl_gt i_a, i_b, i_c;
    i_a = float2ctrl(ADC_Read_Channel_A());
    i_b = float2ctrl(ADC_Read_Channel_B());
    i_c = float2ctrl(ADC_Read_Channel_C());
}

// 输出回调：输出 PWM 占空比到硬件
static inline void ctl_output_callback(void) {
    // 输出三相 PWM 占空比（示例）
    extern ctrl_gt duty_a, duty_b, duty_c;
    PWM_Set_Duty_A(ctrl2float(duty_a));
    PWM_Set_Duty_B(ctrl2float(duty_b));
    PWM_Set_Duty_C(ctrl2float(duty_c));
}
```

**关键配置（ctrl_settings.h）：**

```c
// BUILD_LEVEL 选择（1=开环, 2=电流环IF, 3=速度环, 4=完整功能）
#define BUILD_LEVEL 3

// 控制频率配置
#define CONTROLLER_FREQUENCY           10000.0f  // 10kHz
#define CONTROLLER_PWM_PERIOD_US       100.0f    // 100us
#define CONTROLLER_PWM_CMP_MAX         5000      // PWM 计数器最大值

// 电机参数配置
#define MTR_ENCODER_LINES              2500      // 编码器线数
#define MTR_ENCODER_OFFSET             0.0f      // 编码器零位偏置
#define MTR_CTRL_CURRENT_LOOP_BW       500.0f    // 电流环带宽 (rad/s)
#define MTR_CTRL_SPEED_LOOP_BW         50.0f     // 速度环带宽 (rad/s)
```

---

## 7. 跨平台支持

### 7.1 支持的平台

| 平台名称 | 芯片系列 | 开发工具 | 仿真/实物 | 典型型号 |
|---------|---------|---------|---------|---------|
| **simulate** | x86/x64 PC | MATLAB Simulink + Visual Studio | 仿真 | N/A |
| **f280039c_iris_node** | TI C2000 (C28x) | Code Composer Studio + SysConfig | 实物 | F280039C |
| **f280049c** | TI C2000 (C28x) | Code Composer Studio + SysConfig | 实物 | F280049C |
| **f28p650** | TI C2000 (C28x+) | Code Composer Studio + SysConfig | 实物 | F28P650DK |
| **stm32f405** | STM32 F4 | STM32CubeIDE + CubeMX | 实物 | STM32F405 |
| **stm32g431** | STM32 G4 | STM32CubeIDE + CubeMX | 实物 | NUCLEO-G431RB |
| **stm32g474** | STM32 G4 | STM32CubeIDE + CubeMX | 实物 | NUCLEO-G474RE |
| **stm32f407** | STM32 F4 | STM32CubeIDE + CubeMX | 实物 | STM32F407 |

### 7.2 平台选择指南

#### 7.2.1 按应用场景选择

| 应用场景 | 推荐平台 | 原因 |
|---------|---------|------|
| **算法验证** | simulate (Simulink) | 快速迭代、波形可视化、无需硬件 |
| **教学演示** | STM32G431/G474 + NUCLEO 板 | 低成本、易获取、丰富的社区资源 |
| **工业产品** | TI C2000 (F280049C/F28P650) | 高性能 FPU、丰富的电机控制外设、成熟工具链 |
| **低成本量产** | STM32F405/F407 | 性价比高、国产替代方案丰富 |
| **高性能伺服** | TI F28P650 + CLA | 双核（C28x + CLA）、高主频、低延迟 |

#### 7.2.2 按性能需求选择

| 性能指标 | TI C2000 (F280049C) | STM32 G4 | STM32 F4 |
|---------|---------------------|----------|----------|
| **主频** | 100 MHz (C28x) | 170 MHz (M4) | 168 MHz (M4) |
| **FPU** | 32-bit 单周期 FPU | 32-bit FPU + DSP | 32-bit FPU |
| **ADC 速度** | 3.45 MSPS (12-bit) | 5.33 MSPS (12-bit) | 2.4 MSPS (12-bit) |
| **PWM 分辨率** | 150 ps (HRPWM) | 标准 PWM | 标准 PWM |
| **电机控制外设** | ★★★★★ | ★★★☆☆ | ★★★☆☆ |
| **开发工具** | CCS (免费) | CubeIDE (免费) | CubeIDE (免费) |
| **价格** | 较高 | 中等 | 较低 |

### 7.3 跨平台开发流程

#### 流程图：

```
┌─────────────────────────────────────────────────────────────┐
│ 步骤 1: Simulink 仿真验证                                   │
│ - 在 simulate/ 平台编写和调试控制算法                       │
│ - 使用 Simulink Scope 观察波形                             │
│ - 验证控制参数（PID 参数、带宽等）                          │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 步骤 2: 硬件平台适配                                        │
│ - 复制 implement/simulate/ → implement/<target_platform>/  │
│ - 修改 ctrl_settings.h（保持控制参数不变）                 │
│ - 实现 xplt.ctl_interface.h（ADC 和 PWM 回调）            │
│ - 配置 xplt.config.h（引脚映射）                           │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 步骤 3: BUILD_LEVEL 增量调试                                │
│ - BUILD_LEVEL 1: 验证 PWM 输出和 ADC 采样                  │
│ - BUILD_LEVEL 2: 验证电流环（IF 模式）                     │
│ - BUILD_LEVEL 3: 验证速度环（带传感器）                    │
│ - BUILD_LEVEL 4: 完整功能测试                              │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 步骤 4: 多平台部署                                          │
│ - 使用相同的 common/ 代码                                   │
│ - 为不同芯片创建不同的平台特化代码                          │
│ - 实现统一的 AT 命令接口，便于跨平台调试                   │
└─────────────────────────────────────────────────────────────┘
```

---

## 8. 使用指南

### 8.1 快速启动流程

#### 8.1.1 选择合适的 Suite 工程

根据应用需求选择工程：

**场景 1：我要开发一个 PMSM 伺服驱动器**
- 选择：`mcs_pmsm_nt`（新模板，文档完善）
- 平台：先用 `simulate` 验证算法，再移植到 `f280049c` 或 `stm32g431`

**场景 2：我要开发一个光伏并网逆变器**
- 选择：`pgs_3ph_GFL_inverter`（跟网型三相逆变器）
- 平台：先用 `simulate` 验证 PLL 和功率控制，再移植到 `f280049c`

**场景 3：我要开发一个数字 Buck 电源**
- 选择：`dps_buck`
- 平台：先用 `simulate` 验证双环控制，再移植到 `stm32f407`

#### 8.1.2 Simulink 仿真测试（推荐第一步）

**步骤：**

1. **打开工程**：
   ```
   cd <suite_name>/project/simulate/
   打开 <project_name>.slx
   ```

2. **配置仿真参数**：
   - 修改 `implement/simulate/ctrl_settings.h` 中的控制参数
   - 在 Simulink 中配置电机模型参数

3. **运行仿真**：
   - 点击 "Run" 按钮
   - 使用 Scope 观察电流、速度、位置波形

4. **调整参数**：
   - 调整 PID 参数，优化控制性能
   - 验证 BUILD_LEVEL 1/2/3 的各级功能

#### 8.1.3 硬件平台部署

**步骤：**

1. **选择硬件平台**：
   - 示例：选择 STM32G431

2. **打开工程**：
   ```
   cd <suite_name>/project/stm32g431/
   使用 STM32CubeIDE 打开 .project 工程
   ```

3. **检查外设配置**：
   - 打开 `.ioc` 文件，检查 PWM、ADC、GPIO 配置
   - 确认引脚映射与硬件板一致

4. **编译和烧录**：
   - 编译工程（Build Project）
   - 连接 ST-Link 调试器
   - 烧录到目标芯片（Run / Debug）

5. **BUILD_LEVEL 调试**：
   - 从 BUILD_LEVEL 1 开始逐级验证
   - 使用示波器或 DSA 数据采集工具观察波形
   - 使用 AT 命令调试（通过 UART）

### 8.2 AT 命令调试接口

所有 Suite 工程都集成了 **AT 命令调试接口**，通过 UART 串口通信。

#### 8.2.1 常用 AT 命令

| 命令 | 功能 | 示例 |
|-----|------|------|
| `PWRON` | 使能 PWM 输出（启动控制器） | `PWRON` |
| `PWROFF` | 禁用 PWM 输出（停止控制器） | `PWROFF` |
| `RST` | 复位控制器 | `RST` |
| `TEST` | 测试命令（回显 "OK"） | `TEST` |
| `GET <param>` | 读取参数 | `GET SPEED` |
| `SET <param> <value>` | 设置参数 | `SET SPEED 1500` |

#### 8.2.2 使用 AT 命令（示例）

**连接串口：**
- 波特率：115200
- 数据位：8
- 停止位：1
- 校验位：无

**调试流程：**
```
# 1. 测试连接
>>> TEST
OK

# 2. 使能控制器
>>> PWRON
Motor Controller Started

# 3. 设置速度目标（单位：RPM）
>>> SET SPEED 1500
Speed setpoint = 1500 RPM

# 4. 读取当前速度
>>> GET SPEED
Current speed = 1485 RPM

# 5. 停止控制器
>>> PWROFF
Motor Controller Stopped
```

### 8.3 DSA 数据采集与波形观测

GMP 提供 **DSA（Data Stream Analyzer）** 工具，可实时采集控制器内部变量并在 PC 上绘制波形。

#### 使用方法：

1. **在代码中添加 DSA 通道**（`ctl_main.c`）：
   ```c
   #include <ctl/component/dsa/dsa.h>
   
   // 定义 DSA 对象
   dsa_t dsa;
   
   void ctl_init(void) {
       // 初始化 DSA（最多 8 个通道）
       ctl_dsa_init(&dsa, 8);
       
       // 注册监测变量（电流、速度等）
       ctl_dsa_register_channel(&dsa, 0, &i_d);
       ctl_dsa_register_channel(&dsa, 1, &i_q);
       ctl_dsa_register_channel(&dsa, 2, &speed_rpm);
   }
   
   void ctl_step(void) {
       // 控制算法...
       
       // 采集数据
       ctl_dsa_sample(&dsa);
   }
   ```

2. **PC 端启动 DSA 上位机**：
   - 运行 `tools/gmp_hil/dsa_monitor.exe`
   - 选择串口并连接
   - 点击 "Start" 开始采集
   - 实时观察波形

---

## 9. 如何基于现有 Suite 开发新应用

### 9.1 开发流程

#### 步骤 1：选择最接近的 Suite 作为模板

**示例：** 我要开发一个带弱磁控制的 PMSM 驱动器

- 选择模板：`mcs_pmsm_nt`（已有 FOC + 编码器）
- 复制工程：
  ```
  cp -r mcs_pmsm_nt mcs_pmsm_fw
  ```

#### 步骤 2：修改 `common/ctl_main.c`，添加新功能

在 `ctl_main.c` 中添加弱磁控制器：

```c
#include <ctl/component/motor_control/field_weakening/fw_ctrl.h>

// 添加弱磁控制器对象
static mtr_fw_ctrl_t fw_ctrl;

void ctl_init(void) {
    // 原有初始化代码...
    
    // 初始化弱磁控制器
    mtr_fw_ctrl_params_t fw_params = {
        .v_max = float2ctrl(24.0f),      // 最大电压 (V)
        .v_margin = float2ctrl(2.0f),    // 电压裕度 (V)
        .kp = float2ctrl(0.1f),          // P 增益
        .ki = float2ctrl(10.0f),         // I 增益
    };
    ctl_mtr_fw_ctrl_init(&fw_ctrl, &fw_params);
}

void ctl_step(void) {
    // 原有控制代码...
    
    // 执行弱磁控制（调整 i_d 电流）
    ctrl_gt i_d_fw = ctl_mtr_fw_ctrl_step(&fw_ctrl, v_bus, omega_elec);
    
    // 将弱磁电流叠加到原 i_d 指令
    i_d_target = ctl_add(i_d_target, i_d_fw);
    
    // 后续电流控制...
}
```

#### 步骤 3：修改 `ctrl_settings.h`，添加配置宏

```c
// 弱磁控制使能
#define ENABLE_FIELD_WEAKENING    1

// 弱磁控制参数
#define FW_V_MAX                  24.0f     // 直流母线电压 (V)
#define FW_V_MARGIN               2.0f      // 电压裕度 (V)
```

#### 步骤 4：Simulink 仿真验证

- 在 `project/simulate/` 中更新 Simulink 模型
- 添加弱磁控制模块
- 仿真高速工况，验证弱磁效果

#### 步骤 5：硬件平台测试

- 编译并烧录到目标芯片
- 使用 BUILD_LEVEL 增量调试
- 使用 DSA 观察弱磁区电流波形

### 9.2 添加新的硬件平台

**示例：** 我要在国产芯片 HC32F460 上移植 `mcs_pmsm_nt` 工程

#### 步骤 1：创建平台特化文件夹

```bash
cd mcs_pmsm_nt/implement/
mkdir hc32f460
cd hc32f460
```

#### 步骤 2：创建必要文件

在 `hc32f460/` 文件夹中创建以下文件：

**文件 1: `ctrl_settings.h`**（从 `stm32g431/ctrl_settings.h` 复制并修改）

**文件 2: `xplt.config.h`**

```c
#ifndef XPLT_CONFIG_H
#define XPLT_CONFIG_H

// 芯片型号
#define XPLT_CHIP_HC32F460

// 时钟频率
#define XPLT_SYSTEM_CLOCK_HZ    200000000UL    // 200 MHz

// PWM 外设配置
#define XPLT_PWM_TIMER          M4_TMRA_1
#define XPLT_PWM_FREQUENCY_HZ   10000          // 10 kHz

// ADC 外设配置
#define XPLT_ADC_UNIT           M4_ADC1
#define XPLT_ADC_CH_IA          ADC1_CH0       // 电流 A 相
#define XPLT_ADC_CH_IB          ADC1_CH1       // 电流 B 相
#define XPLT_ADC_CH_VBUS        ADC1_CH2       // 母线电压

// GPIO 引脚配置
#define XPLT_GPIO_PWM_AH_PORT   PortA
#define XPLT_GPIO_PWM_AH_PIN    Pin00
// ...

#endif
```

**文件 3: `xplt.ctl_interface.h`**

```c
#ifndef XPLT_CTL_INTERFACE_H
#define XPLT_CTL_INTERFACE_H

#include "hc32_ddl.h"   // HC32 驱动库

// 外部变量（在 ctl_main.c 中定义）
extern ctrl_gt i_a, i_b, i_c;
extern ctrl_gt v_bus;
extern ctrl_gt duty_a, duty_b, duty_c;

// ADC 采样回调
static inline void ctl_input_callback(void) {
    // 读取 ADC 值
    uint16_t adc_ia = ADC_GetChannelData(XPLT_ADC_UNIT, XPLT_ADC_CH_IA);
    uint16_t adc_ib = ADC_GetChannelData(XPLT_ADC_UNIT, XPLT_ADC_CH_IB);
    uint16_t adc_vbus = ADC_GetChannelData(XPLT_ADC_UNIT, XPLT_ADC_CH_VBUS);
    
    // 转换为物理量（需根据硬件电路放大倍数调整）
    float ia_amp = (adc_ia - 2048.0f) * 0.01f;   // 假设放大 100 倍，0.005Ω 采样电阻
    float ib_amp = (adc_ib - 2048.0f) * 0.01f;
    float vbus_volt = adc_vbus * (48.0f / 4096.0f);  // 假设 48V 满量程
    
    // 转换为 ctrl_gt 类型
    i_a = float2ctrl(ia_amp);
    i_b = float2ctrl(ib_amp);
    i_c = ctl_neg(ctl_add(i_a, i_b));  // i_c = -(i_a + i_b)
    v_bus = float2ctrl(vbus_volt);
}

// PWM 输出回调
static inline void ctl_output_callback(void) {
    // 将 ctrl_gt 占空比转换为 PWM 计数值
    float duty_a_f = ctrl2float(duty_a);  // 范围 [0, 1]
    float duty_b_f = ctrl2float(duty_b);
    float duty_c_f = ctrl2float(duty_c);
    
    // 写入 PWM 寄存器（假设定时器周期为 10000）
    uint16_t cmp_a = (uint16_t)(duty_a_f * 10000.0f);
    uint16_t cmp_b = (uint16_t)(duty_b_f * 10000.0f);
    uint16_t cmp_c = (uint16_t)(duty_c_f * 10000.0f);
    
    TMRA_SetCompareValue(M4_TMRA_1, TmraCh_1, cmp_a);
    TMRA_SetCompareValue(M4_TMRA_1, TmraCh_2, cmp_b);
    TMRA_SetCompareValue(M4_TMRA_1, TmraCh_3, cmp_c);
}

#endif
```

**文件 4: `xplt.peripheral.c` 和 `.h`**

实现 ADC、PWM、GPIO、UART 等外设的初始化代码。

#### 步骤 3：创建硬件工程

使用 HC32 的开发工具（如 Keil MDK）创建工程，添加：
- GMP 核心库文件
- GMP CTL 组件库文件
- `implement/common/` 中的公共代码
- `implement/hc32f460/` 中的平台特化代码

#### 步骤 4：编译、烧录、调试

按照 BUILD_LEVEL 增量调试流程进行测试。

---

## 10. 常见问题与最佳实践

### 10.1 常见问题

#### Q1: 为什么同一套代码可以在不同平台运行？

**A:** GMP 使用 **CSP（Chip Support Package）** 实现平台抽象：
- 控制算法代码（`implement/common/`）只调用 `ctl_input_callback()` 和 `ctl_output_callback()`
- 这两个回调函数在每个平台的 `xplt.ctl_interface.h` 中实现
- 底层硬件差异被 CSP 层屏蔽，上层算法保持不变

#### Q2: BUILD_LEVEL 1 的 PWM 输出正常，但 BUILD_LEVEL 2 的电流环不稳定？

**可能原因：**
1. **ADC 采样增益错误**：检查 `xplt.ctl_interface.h` 中 ADC 到物理量的转换系数
2. **电流传感器极性反接**：尝试在代码中取反电流
3. **电流环 PI 参数不合适**：降低 `MTR_CTRL_CURRENT_LOOP_BW` 参数
4. **PWM 死区时间过大**：调整硬件死区设置

**调试方法：**
- 使用示波器测量实际电流波形
- 使用 DSA 工具采集 `i_d`, `i_q`, `i_d_target`, `i_q_target` 波形对比

#### Q3: 如何调整 PID 参数？

**电流环参数调整：**
```c
// 降低带宽（更保守，更稳定）
#define MTR_CTRL_CURRENT_LOOP_BW    300.0f    // 从 500 降低到 300

// 增加带宽（更激进，更快响应，可能振荡）
#define MTR_CTRL_CURRENT_LOOP_BW    800.0f    // 从 500 提高到 800
```

**速度环参数调整：**
```c
// 速度环带宽通常为电流环的 1/10
#define MTR_CTRL_SPEED_LOOP_BW      30.0f     // 电流环 300 rad/s → 速度环 30 rad/s
```

#### Q4: Simulink 仿真正常，但实物测试失败？

**可能原因：**
1. **硬件死区时间**：Simulink 模型通常不考虑死区，实物必须配置
2. **ADC 采样延迟**：真实 ADC 有转换时间，需要在 PWM 中断中提前触发采样
3. **电流传感器噪声**：实物系统需要增加 ADC 滤波
4. **电机参数差异**：实际电机参数与仿真模型不一致

**解决方法：**
- 在实物平台先运行 BUILD_LEVEL 1，验证 PWM 和 ADC
- 使用示波器对比仿真和实物的电流波形
- 适当降低控制带宽，增强鲁棒性

### 10.2 最佳实践

#### 实践 1：始终从 Simulink 开始

**优势：**
- 快速验证控制算法
- 可视化调试（Scope 实时波形）
- 无需硬件即可调试大部分逻辑
- 参数调优效率高

**流程：**
```
Simulink 仿真 → 参数优化 → 导出参数 → 硬件平台测试
```

#### 实践 2：使用 BUILD_LEVEL 增量调试

**不要跳级！** 必须按照 1 → 2 → 3 → 4 的顺序验证：

| BUILD_LEVEL | 验证目标 | 失败后果 |
|------------|---------|---------|
| Level 1 | PWM 输出、ADC 采样 | 无法进入后续调试 |
| Level 2 | 电流环性能 | 电流振荡、过流保护 |
| Level 3 | 速度环性能、传感器 | 速度不受控、传感器错误 |
| Level 4 | 完整功能 | 状态机异常、通信故障 |

#### 实践 3：充分利用 AT 命令和 DSA

**AT 命令用于：**
- 快速启停控制器
- 在线调整参数（速度、电流指令等）
- 读取状态信息（故障代码、运行时间等）

**DSA 用于：**
- 实时监测控制器内部变量
- 分析控制性能（超调、稳态误差、响应时间）
- 排查控制器异常（电流饱和、积分饱和等）

#### 实践 4：保持 `common` 代码平台无关

**规则：**
- `implement/common/` 中的代码不能包含平台特定的头文件（如 `stm32f4xx.h`）
- 所有硬件相关操作必须通过 `ctl_input_callback()` 和 `ctl_output_callback()` 实现
- 配置参数放在各平台的 `ctrl_settings.h` 中，不要硬编码在 `common` 代码中

**好处：**
- 一套算法代码可以在多个平台复用
- 减少平台移植工作量
- 降低代码维护成本

#### 实践 5：使用预设参数库

GMP 提供了丰富的预设参数：

**电机预设**（`ctl/component/motor_control/motor_preset/`）：
```c
#include <ctl/component/motor_control/motor_preset/GBM2804H_100T.h>
```

**控制器预设**（`ctl/component/motor_control/controller_preset/`）：
```c
#include <ctl/component/motor_control/controller_preset/TI_3PH_GAN_INV.h>
```

**优势：**
- 快速配置已知电机和硬件平台
- 减少手动计算错误
- 参考已验证的成功案例

---

## 11. 总结

### 11.1 Suite 核心价值

| 价值点 | 说明 |
|-------|------|
| **即用性** | 开箱即用的完整应用代码，无需从零开始 |
| **跨平台性** | 同一套算法可在 Simulink、TI C2000、STM32 等多平台运行 |
| **可维护性** | 模块化设计，公共代码与平台特化代码分离 |
| **可扩展性** | 基于 Component 组件库，易于添加新功能 |
| **可调试性** | BUILD_LEVEL 增量调试 + AT 命令 + DSA 工具 |

### 11.2 Suite 与其他 GMP 模块的关系

```
GMP 平台架构
├── core/                     # 核心基础设施（标准化、设备接口、内存管理、调度器）
├── csp/                      # 芯片支持包（平台抽象层）
├── ctl/
│   ├── math_block/           # 数学计算库（坐标变换、矩阵运算）
│   ├── component/            # 控制组件库（PID、观测器、编码器）
│   └── suite/               # ★ 应用解决方案（完整的控制器实现）
├── ext/                      # 外部设备驱动（ADC芯片、编码器芯片、通信模块）
└── tools/                    # 开发工具（DSA、SIL、HIL、代码生成器）
```

**使用关系：**
- **Suite** 依赖 **Component**（调用 PID、电流控制器、观测器等模块）
- **Component** 依赖 **math_block**（调用坐标变换、矩阵运算）
- **Suite** 依赖 **CSP**（通过 `xplt.*` 接口访问硬件）
- **Suite** 依赖 **core**（使用调度器、AT 命令、错误处理）

### 11.3 快速索引表

#### 按应用类型查找

| 应用类型 | 推荐 Suite 工程 |
|---------|----------------|
| PMSM 伺服驱动器 | `mcs_pmsm_nt` |
| 低成本无感 PMSM | `mcs_pmsm_smo` |
| 低速/零速启动 PMSM | `mcs_pmsm_hfi` |
| 高效率凸极 PMSM | `mcs_pmsm_mtpa` |
| 异步电机驱动器 | `mcs_acm` |
| DC-DC 升压变换器 | `dps_boost` |
| DC-DC 降压变换器 | `dps_buck` |
| 单相 PFC 整流器 | `dps_dcac_rectifier_spfc` |
| 光伏/储能并网逆变器 | `pgs_3ph_GFL_inverter` |
| UPS 离网逆变器 | `pgs_1ph_dcac_inv` |
| 光伏储能一体化系统 | `pgs_dc_photovoltaic_storage` |

#### 按关键技术查找

| 关键技术 | 相关 Suite 工程 |
|---------|----------------|
| FOC 矢量控制 | `mcs_pmsm_*`、`mcs_acm` |
| CiA402 状态机 | 所有 `mcs_*` 工程 |
| SMO 滑模观测器 | `mcs_pmsm_smo` |
| HFI 高频注入 | `mcs_pmsm_hfi` |
| MTPA 控制 | `mcs_pmsm_mtpa` |
| PLL 锁相环 | `pgs_3ph_GFL_inverter` |
| PQ 功率控制 | `pgs_3ph_*` 系列 |
| 双环控制（电压+电流） | `dps_boost`、`dps_buck` |
| SPFC 功率因数校正 | `dps_dcac_rectifier_spfc` |

---

## 12. 相关文档索引

| 文档名称 | 路径 | 说明 |
|---------|------|------|
| **COMPONENT_GUIDE.md** | `ctl/component/COMPONENT_GUIDE.md` | 组件库详细说明（Suite 的底层模块） |
| **MATH_GUIDE.md** | `ctl/math_block/MATH_GUIDE.md` | 数学计算库说明（坐标变换、矩阵运算） |
| **CSP_GUIDE.md** | `csp/CSP_GUIDE.md` | 芯片支持包说明（跨平台抽象层） |
| **CORE_GUIDE.md** | `core/CORE_GUIDE.md` | 核心基础设施说明（调度器、AT 命令等） |
| **mcs_pmsm_nt/README.md** | `ctl/suite/mcs_pmsm_nt/README.md` | PMSM 矢量控制项目详细说明 |
| **pgs_3ph_GFL_inverter/README.md** | `ctl/suite/pgs_3ph_GFL_inverter/README.md` | 跟网型逆变器项目详细说明 |

---

## 13. 版本历史

| 版本 | 日期 | 修订内容 |
|-----|------|---------|
| 1.0 | 2026-03-11 | 初始版本：涵盖 MCS/DPS/PGS 三大类、跨平台支持、使用指南 |

---

**文档维护者：** GMP Development Team  
**最后更新：** 2026-03-11  
**文档路径：** `ctl/suite/SUITE_GUIDE.md`
