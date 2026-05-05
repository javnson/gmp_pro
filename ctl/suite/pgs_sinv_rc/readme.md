# GMP Single-Phase Inverter/Rectifier (SINV) Framework

基于 GMP (Grid-tied Power) 核心控制库构建的工业级单相双向逆变器（Inverter）与有源前端（AFE Rectifier）标准工程模板。

本工程采用了先进的 **Flat Architecture（扁平化架构）** 与 **Zero-Copy（零拷贝数据绑定）** 技术，在极致压榨 DSP 算力的同时，提供了无与伦比的代码可读性与硬件跨平台移植能力。

## 🌟 核心特性 (Key Features)

- **纯软硬件解耦**：硬件特性（死区、ADC增益、最大耐压）全部由底层宏函数在编译期自动推导，控制层代码做到 100% 平台无关。
- **高阶控制算法矩阵**：
  - **QPR (准比例谐振)**：实现对电网基波电流的无静差极速跟踪。
  - **FDRC (频域重复控制)**：智能分离瞬态与稳态，精准扫平电网背景谐波与死区畸变。
  - **SPLL-SOGI (二阶广义积分锁相环)**：在电网电压畸变、跌落工况下依然保持精准锁相。
  - **内聚式安规发生器**：自带 P/Q 功率指令软启动斜率限制器，完美满足并网安规要求。
  - **带电流方向滞环的单极性 SPWM**：原生支持基于电流过零方向的死区脉宽补偿。
- **CiA 402 工业状态机**：规范化的设备启停、故障闭锁与恢复流转机制。
- **上位机实时透视**：原生集成 Tunable 在线调参字典与 Memory Perspective (内存透视) 协议。

------

## 🚀 快速上手指南 (Getting Started)

### 第一步：配置您的硬件 (Hardware Presets)

如果您的功率板或传感器发生了变更，**永远不要在算法代码中修改魔法数字**。 打开 `ctrl_settings.h`：

1. 包含对应的功率板与传感器板预设文件。
2. 填入物理板上的分压电阻与传感器型号：

C

```
// 例如：更换为 180kΩ 分压电阻测市电，换用 75mV/A 灵敏度的电流传感器
#define CTRL_AC_VOLTAGE_SENSITIVITY  QUAD_SENSOR_CALC_V_GAIN(180.0f) 
#define CTRL_AC_CURRENT_SENSITIVITY  QUAD_SENSOR_CALC_I_GAIN(TMCS1133_B3A_MV_A)
```

系统会在编译期自动重计算所有的 ADC 转换系数、基准值标幺化以及系统安全保护阈值（`CTRL_PROT_VBUS_MAX`、`CTRL_MAX_HW_CURRENT`）。

### 第二步：物理外设零拷贝绑定 (Peripheral Binding)

在 `xplt.peripheral.c` 的 `setup_peripheral()` 函数中，我们将底层 ADC 结构体的地址直接注入到核心算法引擎中：

C

```
ctl_attach_sinv_rc(&rc_core, 
                   &adc_v_bus.control_port, 
                   &adc_v_grid.control_port, 
                   &adc_i_ac.control_port);
```

自此之后，20kHz 高频中断 `ctl_dispatch()` 将直接通过指针解引用读取 ADC 最新值，免去了繁琐的压栈传参，大幅降低 CPU 开销。

### 第三步：分级安全测试 (Phased Testing via BUILD_LEVEL)

修改 `ctrl_settings.h` 中的 `BUILD_LEVEL` 宏，逐步验证您的逆变器：

- **`BUILD_LEVEL 1` (离网校验)**：
  - 强制关闭 FDRC 与电网前馈。
  - 目标：验证 ADC 采样方向、极性，单极性 SPWM 发波逻辑是否正常。可在交流侧挂载纯电阻负载进行恒压或开环电流测试。
- **`BUILD_LEVEL 2` (基础并网)**：
  - 开启电网电压前馈 (Lead Compensation)，关闭 FDRC。
  - 目标：通过 CAN 或上位机下发 P 和 Q 指令，验证逆变器/整流器能否将电流无偏差地注入电网（观察相位是否对齐）。
- **`BUILD_LEVEL 3` (全功能并网)**：
  - CiA 402 状态机允许 Operation Enabled 后，自动延时（如 200 个控制周期）平滑切入 FDRC。
  - 目标：观察电流 THD，验证系统抗电网背景谐波的能力与极致波形。

------

## 🕹️ 运行与操作 (Operation via CiA 402)

本工程受 CiA 402 状态机严格管控，直接发波是不允许的。标准启动流程如下：

1. **上电自检与校准**：系统启动后将自动阻塞，等待 `ctl_exec_adc_calibration` 完成交流电压/电流传感器的零点漂移校准。
2. **锁相环准入**：`ctl_check_pll_locked()` 将持续监测电网电压幅值与频率。只有当 PLL 频率误差趋近于 0（`< 0.005 PU`）且电压在合法区间时，才允许并网。
3. **下发控制字**：
   - 发送命令字 `0x06` (Shutdown) -> `0x07` (Switch On) -> `0x0F` (Enable Operation) 启动设备。
   - 写入 `g_p_ref_user` (有功标幺值) 和 `g_q_ref_user` (无功标幺值)。
   - *(注：指令生成器 `ref_gen` 内部已集成 `slope_limiter`，即便突加满载指令，设备也会按照安规规定的斜率平滑加载，绝不会过流炸管)*。

------

## 🛡️ 安全与保护 (Safety & Protection)

安全防护分为三层，层层递进：

1. **硬件级限幅 (Hardware PWM Clamp)**：`hpwm_modulator` 底层强制使用 `int32_t` 防下溢计算，绝对防止算术溢出导致的满占空比炸管。
2. **极速 ISR 保护 (Fast Protection)**：每次 20kHz 中断计算完控制指令后，立即调用 `ctl_step_sinv_protect_fast`。一旦发现母线快超压或交流快过流，直接通过软硬件 TZ 瞬间封锁发波。
3. **慢速保护任务 (Slow Task Protection)**：在 `user_main.c` 的 10ms 周期任务 `tsk_protect` 中，持续监测交流电压有效值 (RMS OVP/UVP)、电网频率超限以及设备过温，触发状态机的 Fault 状态。

------

*Happy Power Coding! —— Designed with GMP Library.*
