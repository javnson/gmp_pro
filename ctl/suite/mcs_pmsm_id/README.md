# 永磁同步电机矢量控制与全自动参数辨识系统 (PMSM Vector Control & Offline ID)

## 项目概述

本项目实现了一个基于 GMP 平台的永磁同步电机（PMSM）矢量控制算法，支持多平台部署和增量化调试。 在基础的 FOC 矢量控制之上，本项目**内嵌了工业级的全自动离线参数辨识（Offline Parameter Identification）引擎**。该引擎能够在电机静止或空载运行状态下，自动测量电机的电气参数（电阻、电感、磁链、死区补偿）和机械参数（惯量、摩擦系数），并自动完成电流环 PI 参数的整定。

开发者可以通过 `BUILD_LEVEL` 宏进行底层逻辑的增量调试，也可以通过上层接口一键启动参数辨识。



---

## 目录结构说明

```
mcs_pmsm_nt/
├── doc/                          # 项目文档和图片资源
│   └── img/                      # 文档图片资源
├── implement/                    # 控制算法实现代码
│   ├── common/                   # 所有平台公用的控制代码
│   │   ├── ctl_main.c           # 控制器主逻辑实现
│   │   ├── ctl_main.h           # 控制器声明和接口
│   │   ├── user_main.c          # 用户主程序（AT命令、调度器）
│   │   └── user_main.h          # 用户主程序头文件
│   ├── f280039c_iris_node/      # TI C2000 F280039C (Iris节点) 平台特化代码
│   ├── f280049c/                # TI C2000 F280049C 平台特化代码
│   ├── simulate/                # PC仿真环境特化代码
│   └── stm32g431/               # STM32G431 平台特化代码
└── project/                      # 各平台的工程文件
    ├── f280039c_Iris_node/      # TI CCS 工程 (F280039C)
    ├── f280049c/                # TI CCS 工程 (F280049C)
    ├── simulate/                # Simulink 仿真工程
    └── stm32g431/               # STM32CubeIDE 工程
```

在原有架构基础上，参数辨识相关接口集中在以下文件中，对用户层保持极简暴露：

| **文件名**               | **功能说明**                                                 |
| ------------------------ | ------------------------------------------------------------ |
| `pmsm_offline_id_if.c/h` | **辨识模块用户配置接口**。包含模块初始化、任务循环、以及 FOC 核心的路由适配器（Adapter）。用户在此配置测试电压、电流和速度阈值。 |
| `ctl_main.c`             | **控制器主逻辑**。在 `ctl_dispatch()` 中，辨识模块（OID）会作为“超级大脑”在特定阶段接管 FOC 的指令下发和角度路由。 |



### 平台特化文件说明

每个平台特化文件夹（如 `f280039c_iris_node/`、`stm32g431/` 等）包含以下核心文件：

| 文件名 | 功能说明 |
|--------|----------|
| `ctrl_settings.h` | 控制参数配置（BUILD_LEVEL、PWM频率、电机参数等） |
| `xplt.config.h` | 平台配置（引脚定义、外设配置） |
| `xplt.ctl_interface.h` | 控制接口（ADC采样、PWM输出回调函数） |
| `xplt.peripheral.c/h` | 外设初始化和底层驱动代码 |

---

## 核心功能：全自动离线参数辨识 (PMSM OID)

如果用户不需要使用辨识功能，只需在 `pmsm_offline_id_if.c` 的初始化配置中将各个 `flag_enable_xxx` 设为 `0`，系统将完全回退为传统的电机控制器模式。

### 1. 启动辨识的使用流程

参数辨识模块作为一个高级状态机运行，必须在底层硬件和安全机制就绪后方可启动。**标准操作流程如下：**

1. **使能底层驱动 (CiA402 状态机准备)**：

   辨识系统需要能够输出 PWM。首先通过串口发送 AT 命令 `AT+ENABLE`，确保 CiA402 状态机进入 `Operation Enabled`（运行使能）状态。

2. **触发辨识状态机**：

   在用户程序中（或通过串口自定义指令），将主状态机变量切换至准备阶段：

   `pmsm_oid.sm = PMSM_OFFLINE_ID_PREPARE;`

3. **等待状态机完成**：

   系统将自动依次进行：ADC 偏置校准 $\rightarrow$ 定子电阻与死区测量 $\rightarrow$ 交直轴电感测量 $\rightarrow$ 磁链测量 $\rightarrow$ 机械参数测量。

   通过监控 `pmsm_oid.sm` 变量，当其达到 `PMSM_OFFLINE_ID_COMPLETE (8)` 时，代表辨识圆满结束。

4. **读取识别结果**：

   一旦到达 `COMPLETE` 状态，底层输出会自动安全关闭。此时用户可以从以下全局结构体中读取极其精确的电机参数：

   - `pmsm_oid.pmsm_param.Rs`：定子相电阻 (Ω)
   - `pmsm_oid.pmsm_param.Ld` / `Lq`：交直轴相电感 (H)
   - `pmsm_oid.pmsm_param.flux_linkage`：永磁体磁链 (Wb) / 可推算 KV 值
   - `pmsm_oid.pmsm_param.V_comp_pu`：**死区与管压降综合补偿参数** (标幺值)
   - `pmsm_oid.pmsm_mech_param.J_total` / `B_viscous`：转动惯量与黏性摩擦系数

### 2. 异常中断与排查 (重要)

**现象：** 如果电机在执行参数辨识（特别是启动旋转、加速或减速）到一半时**突然关闭输出，且 `pmsm_oid.sm` 变为 `PMSM_OFFLINE_ID_FAULT (9)`**。

**原因：** 这通常说明电机的保护机制（如过流保护、过压保护）被触发，或者采集的数据发生严重数学奇点（如矩阵计算溢出）。

**排查与解决：**

1. **检查母线电压：** 在减速阶段（DECEL_TEST），动能回馈容易导致母线过压（OVP）。如果频繁在此停机，请在 `pmsm_offline_id_if.c` 中减小 `cfg_mech.decel_iq_pu` 的绝对值，或提高 `max_vbus_pu` 的容忍度。
2. **检查测试电流：** 如果电机内阻极小，配置的 `pulse_voltage_pu`（测试电压脉冲）过大可能会瞬间触发过流（OCP）。请适当调低注入电压。
3. **失步振荡：** 确保在 `FLUX` 初始化中已经启用了基于已辨识 $R/L$ 的电流环 PI 参数自动整定（Auto-Tuning），防止动态拖拽时电流环崩溃。

## 参数辨识算法核心思路揭秘

本系统的辨识引擎摒弃了简单的单点差分法，全面采用了**基于 DSA (Data Scope Analyzer) 数据缓存矩阵的系统级算法**，以对抗工业现场的 ADC 噪声和逆变器非线性：

1. **定子电阻 ($R_s$) 与死区补偿 ($V_{comp}$)**
   - **思路：** 阶梯式注入直流偏置电流。
   - **算法：** 记录不同电流下的稳态电压指令，通过一元线性回归拟合直线（$V = R_s \cdot I + V_{comp}$）。直线的斜率为纯粹的电阻（完美剔除了管压降），直线的截距为非线性死区及管压降补偿量。
2. **交/直轴电感 ($L_d, L_q$)**
   - **思路：** 施加极短的高频电压脉冲（开环），记录电流的瞬态上升曲线。
   - **算法：** 采用**物理积分法 (Integral Method)** 代替易受噪声干扰的斜率 ($di/dt$) 微分法。通过对 RL 一阶响应过程求面积积分，并在计算中引入真实的母线电压 ($U_{dc}$) 补偿，得出不受脉冲长度影响的精确高频瞬态电感。
3. **永磁体磁链 ($\psi_m$)**
   - **思路：** 采用 V/F 开环与电流闭环结合的 I/F 拖拽模式，使电机稳定运行在不同转速梯度。
   - **算法：** 读取多组转速下的稳态交直轴电压、电流。在计算反电势 $E$ 时，**强耦合扣除了第一阶段辨识出的电阻压降和 $V_{comp}$ 死区压降**，随后通过反电势对电角速度的线性回归，极其精准地提取出永磁体磁链（反推 KV 值）。
4. **机械惯量 ($J$) 与摩擦系数 ($B$)**
   - **思路：** 包含平滑切换的加减速测试。从开环平滑切入闭环后，分别施加正向和反向恒定转矩电流。
   - **算法：** 记录加速段和减速段的速度-时间曲线。利用 DSA 引擎算出实际角加速度，通过电磁转矩差与角加速度差的联立方程，消去未知摩擦力，解析出转动惯量 $J$；再结合稳态速度下的拖拽电流，解析出黏性阻尼 $B$。

## 核心功能模块

### 1. 控制器主逻辑 (`common/ctl_main.c`)

**核心组件：**
- **CiA402 状态机** (`cia402_sm`): 符合CiA402标准的电机控制状态机
- **电流控制器** (`mtr_ctrl`): 永磁同步电机电流环控制
- **运动控制器** (`motion_ctrl`): 速度/位置环控制
- **调制器** (`spwm`): SPWM/SVPWM/NPC调制器
- **位置编码器** (`pos_enc`): 自动换向编码器
- **速度计算器** (`spd_enc`): 转速测量
- **角度生成器** (`rg`): IF（中频）控制的角度斜坡生成器
- **ADC校准器** (`adc_calibrator`): 自动ADC偏置校准

**核心函数：**
- `ctl_init()`: 初始化所有控制器对象
- `ctl_mainloop()`: 主循环任务（CiA402状态机调度）
- `ctl_dispatch()`: 周期性控制回调（由中断调用）

### 2. 用户主程序 (`common/user_main.c`)

**功能：**
- **AT命令接口**: 通过串口发送指令控制电机
  - `AT+ENABLE`: 使能电机运行
  - `AT+POWEROFF`: 关闭电机输出
  - `AT+RST`: 复位故障状态
- **GMP调度器** (`gmp_scheduler_t`): 管理后台任务

### 3. GMP组件库依赖

本项目使用了以下GMP平台组件：

| 组件路径 | 功能 |
|----------|------|
| `ctl/component/interface/` | ADC通道、PWM通道接口 |
| `ctl/component/motor_control/` | 电流控制器、运动控制器、编码器 |
| `ctl/component/interface/pwm_modulator.h` | PWM调制器 |
| `ctl/framework/cia402_state_machine.h` | CiA402状态机框架 |
| `core/mm/scheduler.h` | GMP任务调度器 |
| `core/dev/at_device.h` | AT命令解析器 |

---

## BUILD_LEVEL 增量调试机制

### BUILD_LEVEL 定义

在 `ctrl_settings.h` 文件中定义 `BUILD_LEVEL` 宏，控制编译时的功能启用级别：

```c
#define BUILD_LEVEL (2)  // 可设置为 1, 2, 3, 4
```

### 各级别功能说明

| Level | 控制模式 | 角度来源 | 电流环 | 功能描述 |
|-------|----------|----------|--------|----------|
| **1** | 电压开环 | IF斜坡生成器 | 关闭 | 验证PWM输出、调制器、基本外设 |
| **2** | 电流闭环 | IF斜坡生成器 | 开启 | 验证电流采样、电流控制器、Clarke/Park变换 |
| **3** | 带编码器电流闭环 | 真实编码器 | 开启 | 验证编码器接口、实际电机旋转 |
| **4** | 完整矢量控制 | 真实编码器 | 开启+前馈 | 完整功能（前馈补偿、运动控制） |

### 调试步骤详解

#### BUILD_LEVEL 1: 电压开环验证

**目标：** 验证硬件基础功能（PWM输出、调制器、保护电路）

**配置：**
```c
#define BUILD_LEVEL (1)
```

**代码逻辑：**
```c
// 关闭电流控制器
ctl_disable_mtr_current_ctrl(&mtr_ctrl);
// 设置固定电压前馈值 (Vd=0.2p.u., Vq=0.2p.u.)
ctl_set_mtr_current_ctrl_vdq_ff(&mtr_ctrl, 0.2, 0.2);
// 使用IF斜坡角度生成器
ctl_attach_mtr_current_ctrl_port(&mtr_ctrl, ..., &rg.enc, ...);
```

**验证要点：**
- [ ] PWM波形是否正常输出（示波器检查）
- [ ] 三相PWM相位是否正确（120°相差）
- [ ] 死区时间是否合理
- [ ] 电机是否有轻微震动（开环转矩）

**常见问题：**
- PWM无输出 → 检查 `ctl_fast_enable_output()` 是否调用
- 电机不动 → 调大电压幅值 (如改为 `0.3, 0.3`)
- 电机抖动严重 → 检查三相接线顺序

---

#### BUILD_LEVEL 2: 电流闭环验证（IF模式）

**目标：** 验证电流采样、电流控制器、坐标变换

**配置：**
```c
#define BUILD_LEVEL (2)
```

**代码逻辑：**
```c
// 开启电流控制器
ctl_enable_mtr_current_ctrl(&mtr_ctrl);
// 设置电流给定 (Id=0.1p.u., Iq=0.1p.u.)
ctl_set_mtr_current_ctrl_ref(&mtr_ctrl, float2ctrl(0.1), float2ctrl(0.1));
// 仍使用IF斜坡角度
ctl_attach_mtr_current_ctrl_port(&mtr_ctrl, ..., &rg.enc, ...);
```

**验证要点：**
- [ ] 电流采样值是否合理（通过调试器或串口打印查看）
- [ ] 电流跟踪给定值（Id、Iq实际值接近给定值）
- [ ] 电机平滑旋转（IF频率由 `rg` 控制，默认20Hz）
- [ ] ADC偏置校准是否成功

**调试技巧：**
```c
// 打印关键变量（在 ctl_dispatch() 中添加）
gmp_base_print("Id_ref=%d, Id_fb=%d, Iq_ref=%d, Iq_fb=%d\r\n",
    mtr_ctrl.id_ref, mtr_ctrl.id, mtr_ctrl.iq_ref, mtr_ctrl.iq);
```

**常见问题：**
- 电流振荡 → PI参数需要调整（在 `ctl_main.c` 的 `ctl_auto_tuning_mtr_current_ctrl()` 中）
- 电流偏置错误 → 启用ADC校准：`flag_enable_adc_calibrator = 1`

---

#### BUILD_LEVEL 3: 编码器闭环验证

**目标：** 接入真实编码器，实现实际转子位置控制

**配置：**
```c
#define BUILD_LEVEL (3)
```

**代码逻辑：**
```c
// 使用真实编码器位置
ctl_attach_mtr_current_ctrl_port(&mtr_ctrl, ..., &pos_enc.encif, ...);
// CiA402状态机加速切换
cia402_sm.minimum_transit_delay[CIA402_SM_READY_TO_SWITCH_ON] = 0;
```

**验证要点：**
- [ ] 编码器位置读取正确（检查 `pos_enc.encif.theta_m`）
- [ ] 电机能够稳定旋转
- [ ] 速度测量准确（检查 `spd_enc.w_m`）
- [ ] 可通过AT命令控制电机

**编码器调试：**
```c
// 在 xplt.ctl_interface.h 中添加编码器读取
#if BUILD_LEVEL == 1
    // Level 1 不使用编码器
#else
    // 读取编码器位置（示例：SPI编码器）
    ctl_step_autoturn_pos_encoder(&pos_enc, read_encoder_position());
#endif
```

---

#### BUILD_LEVEL 4: 完整功能验证

**目标：** 启用所有优化功能（前馈补偿、运动控制）

**配置：**
```c
#define BUILD_LEVEL (4)
```

**新增功能：**
- 电压前馈补偿（提升动态响应）
- 完整的速度/位置环控制
- 高级观测器（如有）

---

## 快速上手指南

### 1. 选择目标平台

根据你的硬件选择对应的工程：

| 硬件平台 | 工程路径 | IDE |
|----------|----------|-----|
| TI F280039C (Iris节点) | `project/f280039c_Iris_node/` | Code Composer Studio |
| TI F280049C LaunchPad | `project/f280049c/` | Code Composer Studio |
| STM32G431 | `project/stm32g431/` | STM32CubeIDE / Keil MDK |
| PC仿真（Simulink） | `project/simulate/` | MATLAB/Simulink |

### 2. 配置控制参数

编辑对应平台的 `ctrl_settings.h`：

```c
// 设置调试级别
#define BUILD_LEVEL (1)  // 从1开始逐步增加

// 配置电机参数（或使用预设）
#include <ctl/component/hardware_preset/pmsm_motor/TYI_5008_KV335.h>

// 配置变流器参数
#include <ctl/component/hardware_preset/inverter_3ph/GMP_Helios_3PhGaNInv_LV.h>
```

### 3. 编译并下载程序

- **TI平台**: 在CCS中导入工程，编译后通过XDS调试器下载
- **STM32平台**: 在STM32CubeIDE中编译，通过ST-Link下载
- **仿真平台**: 运行 `gmp_fac_generate_src.bat` 生成代码，编译VS工程

### 4. 运行调试

#### 串口控制命令

连接串口（波特率通常为115200），发送以下命令：

```bash
AT+RST         # 复位故障
AT+ENABLE      # 使能电机运行
AT+POWEROFF    # 关闭电机
```

#### CiA402 状态转换流程

```
┌──────────────────┐
│ START (上电复位)  │
└────────┬─────────┘
         ↓
┌──────────────────┐
│ Switch On        │ (等待使能命令)
│ Disabled         │
└────────┬─────────┘
         ↓ AT+ENABLE
┌──────────────────┐
│ Ready to         │
│ Switch On        │
└────────┬─────────┘
         ↓
┌──────────────────┐
│ Switched On      │
└────────┬─────────┘
         ↓
┌──────────────────┐
│ Operation        │ (电机正常运行)
│ Enabled          │
└──────────────────┘
```

### 5. 增量调试流程

**推荐调试顺序：**

```
Level 1 (2小时)
  ↓ 验证PWM输出、调制器
Level 2 (4小时)  
  ↓ 验证电流采样、电流闭环
Level 3 (4小时)
  ↓ 验证编码器、实际旋转
Level 4 (2小时)
  ↓ 完整功能测试
```

每个级别验证通过后，修改 `BUILD_LEVEL` 宏并重新编译。

---

## 关键代码流程

### 主中断服务程序（ISR）流程

```c
void ADC_ISR(void)  // 在 xplt.ctl_interface.h 中
{
    // 1. 输入采样
    ctl_input_callback();      // 读取ADC数据并转换为p.u.值
    
    // 2. 控制算法
    ctl_dispatch();            // 执行控制器计算（在 ctl_main.h 中）
    
    // 3. 输出更新
    ctl_output_callback();     // 更新PWM占空比
}
```

### 控制器调度逻辑

```c
void ctl_dispatch(void)  // 在 ctl_main.h 中
{
    if (flag_enable_adc_calibrator) {
        // ADC校准流程
        ctl_step_adc_calibrator(...);
    }
    else {
        // 正常控制流程
        ctl_step_vel_pos_ctrl(&motion_ctrl);        // 运动控制
        ctl_set_mtr_current_ctrl_ref(...);          // 设置电流给定
        ctl_step_current_controller(&mtr_ctrl);     // 电流控制
        ctl_step_svpwm_modulator(&spwm);            // SVPWM调制
    }
}
```

---

## 常见问题排查

### 1. 编译错误

**错误示例：**
```
undefined reference to 'ctl_init_mtr_current_ctrl'
```

**解决方法：**
- 确保GMP库路径已正确添加到工程的include路径
- 检查 `gmp_src_mgr/` 目录下的源文件是否已添加到工程

### 2. 电机不响应

**排查步骤：**
1. 检查 `flag_system_running` 是否为1
2. 检查CiA402状态机当前状态：`cia402_sm.current_state`
3. 确认PWM使能：调用 `ctl_fast_enable_output()`
4. 示波器检查PWM波形

### 3. 电流振荡

**可能原因：**
- PI参数不合适 → 调整 `ctl_auto_tuning_mtr_current_ctrl()` 参数
- 采样延迟过大 → 检查ADC触发时刻
- 电机参数错误 → 核对 `MOTOR_PARAM_RS`, `MOTOR_PARAM_LS`

### 4. 编码器位置跳变

**检查：**
- 编码器接线是否松动
- SPI/ABZ信号质量（示波器检查）
- 编码器分辨率配置：`CTRL_POS_ENC_FS`

---

## 扩展开发

### 添加新平台支持

1. 在 `implement/` 下创建新平台文件夹：
   ```
   implement/
   └── your_platform/
       ├── ctrl_settings.h
       ├── xplt.config.h
       ├── xplt.ctl_interface.h
       ├── xplt.peripheral.c
       └── xplt.peripheral.h
   ```

2. 在 `project/` 下创建对应IDE工程

3. 实现平台特定的回调函数：
   - `ctl_input_callback()`: ADC采样
   - `ctl_output_callback()`: PWM输出
   - `ctl_fast_enable_output()`: 使能PWM
   - `ctl_fast_disable_output()`: 禁用PWM

### 修改控制算法

控制算法修改主要在 `implement/common/ctl_main.c` 中：

- 修改电流环PI参数：`ctl_auto_tuning_mtr_current_ctrl()`
- 修改速度环参数：`ctl_init_vel_pos_ctrl()`
- 添加新的观测器：在 `ctl_init()` 中初始化，在 `ctl_dispatch()` 中调用

---

## 参考资料

- **GMP平台文档**: `manual/Readme_CN.md`
- **CiA402状态机**: `ctl/framework/cia402_state_machine.h`
- **电流控制器**: `ctl/component/motor_control/current_loop/motor_current_ctrl.h`
- **PWM调制器**: `ctl/component/interface/pwm_modulator.h`

---

## 技术支持

如有问题，请联系：
- 邮箱: javnson@zju.edu.cn
- 项目仓库: (根据实际填写)

---

**版本历史：**
- v1.0 (2024-09-30): 初始版本
- v1.1 (2026-01-27): 添加完整文档和BUILD_LEVEL说明
