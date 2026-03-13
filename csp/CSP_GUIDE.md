# GMP CSP 平台支持包使用与扩展指南

## 目录
- 概述
- 架构与关键文件
- 已支持平台与目录导览
- 使用方法（按平台）
- 必须实现的标准接口
- 新增自定义平台支持包指南
- 端到端数据流与中断时序
- 端口适配与单位制（p.u.）
- 调试与验证流程
- 常见问题与排查清单

---

## 概述
GMP CSP（Chip Support Package）为控制算法提供跨平台抽象层，屏蔽外设差异（ADC、PWM、定时器、GPIO、串口等），使 `ctl/` 下的控制器与 `core/` 调度器可在多种 MCU/SoC 与 PC 仿真环境之间复用。

目标：
- 在不同芯片上复用相同控制算法（如 PMSM FOC、数字电源）。
- 提供统一的输入/输出回调与外设初始化模板。
- 以最小代价新增/移植至新平台。

---

## 架构与关键文件
CSP 作为“平台抽象层”，约定以下关键头文件与实现：

- `csp.general.h / .hpp`：平台通用声明（包含基础类型、断言、编译器兼容宏等）。
- `csp.typedef.h / .hpp`：平台侧类型别名（与 `ctrl_gt/fase_gt/time_gt` 等配合）。
- `csp.config.h`：平台配置（是否启用FPU、端序、对齐、编译开关、日志开关）。
- `xplt.config.h`：平台外设/引脚/时钟等配置（板级相关）。
- `xplt.peripheral.h/.c(pp)`：外设初始化与底层驱动组织入口。
- `xplt.ctl_interface.h`：控制回调接口（输入/算法/输出时序），核心对接点。
- 工程支撑文件（IDE 工程、链接脚本、SysConfig/PinMux配置、VS/Keil 工程等）。

关系：
- 控制算法在 `implement/common` 调用 `ctl_dispatch()`（算法一步）。
- 各平台在 `xplt.ctl_interface.h` 的 ISR 内组织：采样 → 算法 → 输出。

---

## 已支持平台与目录导览
```
csp/
├─ c28x_pinmux/         # TI C2000（经典PinMux方式）
├─ c28x_syscfg/         # TI C2000（SysConfig方式，含多个demo）
├─ stm32/               # ST STM32系列（多款Nucleo板）
├─ silan_spc6l64/       # Silan SPC6L64 平台
├─ hc32/                # HDSC/HC32 平台（示例）
├─ zynq7/               # Xilinx Zynq-7000 平台
├─ windows_simulink/    # 纯PC调试/可视化（Windows）
├─ matlab_simulink_level2/ # MATLAB Level-2 S-Function 接口
├─ null/                # 空平台模板（最小可运行骨架）
└─ readme.md            # 简介
```

子目录常见内容：
- `.../src/`：平台驱动与适配层实现。
- `.../demo_project`、`..._demo/`：可直接导入的IDE工程示例。
- `GMPCorePropertySheet.props`：Windows/MSBuild 属性表（含包含路径等）。

---

## 使用方法（按平台）

### TI C2000（C28x）
- 选择：`c28x_syscfg/f280039c_demo` 或 `f280049c_demo` 等 demo 工程。
- 在 Code Composer Studio（CCS）中导入工程并生成 SysConfig（.syscfg）。
- 核对：
  - PWM 频率/计数最大值与 `ctrl_settings.h` 参数一致。
  - ADC 触发点与 PWM 同步，采样点位于死区外。
  - 中断优先级：ADC(高) > PWM/EPWM > 其他外设。
- 运行：连接仿真器，编译下载，串口下发 AT 命令或使用工程内示例。

### STM32（CubeIDE/Keil）
- 选择对应板卡目录（如 `stm32/stm32g431rb_nucleo/`）。
- 使用 CubeMX/CubeIDE 生成外设初始化后，保留/合并 `xplt.*` 层代码。
- 确认定时器触发ADC、DMA配置、NVIC 优先级、GPIO 映射正确。
- 编译下载，使用串口或调试器观察运行状态。

### Windows/Simulink（PC）
- `windows_simulink/`：VS 解决方案用于 PC 端纯软件仿真与调试（含 Trace）。
- `matlab_simulink_level2/`：Level-2 S-Function 接入 Simulink，进行SIL/HIL验证。
- 可与 `ctl/suite/.../project/simulate` 中的 Simulink 工程配套使用。

### 其他平台（Silan/HC32/Zynq7）
- 参考目录结构与 `xplt.*` 接口实现外设初始化与 ISR 调用链。
- 使用对应原厂IDE/SDK 构建工程（Keil/IAR/Vivado/SDK等）。

---

## 必须实现的标准接口
以下接口定义在各平台的 `xplt.ctl_interface.h/.c(pp)` 中，是融合外设与控制内核的“关键路径”。

最小必需：
```c
// 1) 输入回调：采样 & 标幺化
gmp_static_inline void ctl_input_callback(void)
{
    // 读取原始ADC → 写入 *_src（如 iabc_src、vabc_src、udc_src 等）
    // 调用通道步进函数完成 p.u. 转换：
    //   ctl_step_tri_ptr_adc_channel(&iabc);
    //   ctl_step_ptr_adc_channel(&udc);
}

// 2) 算法调度：在 ISR 或调度器中调用
extern void ctl_dispatch(void); // 通常在 ctl_main.h 中定义

// 3) 输出回调：将算法输出映射到 PWM/数模
gmp_static_inline void ctl_output_callback(void)
{
    // 从调制器/控制器结构体取占空比/电压指令，写入 PWM 比较寄存器
}

// 4) 运行使能/失能（由状态机回调触发）
void ctl_fast_enable_output(void);
void ctl_fast_disable_output(void);
```

推荐实现：
- 系统启动顺序（时钟/外设/中断初始化）。
- ADC 校准流程（如偏置采集与自动标定）。
- 调试串口（AT 命令）接入，便于在线下发指令（ENABLE、POWEROFF、RST等）。

---

## 新增自定义平台支持包指南

### 1) 复制骨架与命名
- 拷贝 `csp/null/` 为新平台目录，如 `csp/your_mcu/`。
- 保留并完善以下文件：
  - `csp.config.h`、`csp.general.h/.hpp`、`csp.typedef.h/.hpp`
  - `xplt.config.h`（引脚/外设参数）、`xplt.peripheral.h/.c(pp)`、`xplt.ctl_interface.h`
  - `src/`（放置驱动封装/BSP 代码）

### 2) 对接外设
- ADC：配置触发源（与 PWM 同步），采样序列与结果寄存器映射到 `*_src`。
- PWM：设置计数模式、死区、比较寄存器映射。
- 定时器/中断：建立 ADC/EVENT → ISR → `ctl_input_callback()` → `ctl_dispatch()` → `ctl_output_callback()` 的调用链。
- UART（可选）：接入 AT 命令用于状态机与控制指令交互。

### 3) 连接控制通道
- 绑定接口：
  - 将 `adc_channel`、`pwm_channel`、`pwm_modulator` 与硬件端口一一对应。
  - 在 `ctl_init()`（实现于 `implement/common/ctl_main.c`）中完成控制器对象初始化与端口 attach。

### 4) 编译工程
- 选择工具链（CCS/CubeIDE/Keil/VS 等），创建/导入工程。
- 添加 `gmp_pro` 所需包含路径与源文件（通常有 `gmp_src_mgr/` 辅助）。
- 设置优化等级、FPU/硬件乘法器、链接脚本与堆栈大小。

### 5) 平台自测清单
- PWM 波形正确、死区有效、三相相位正确。
- ADC 采样与 PWM 同步，数据无抖动/丢采。
- 中断优先级合理，控制 ISR 周期稳定（抖动 <5%）。
- 串口/日志可用（若启用）。

---

## 端到端数据流与中断时序
典型 ISR（以 ADC 触发为例）：
```c
// ADC ISR
void ADCx_IRQHandler(void)
{
    // 1. 输入：复制ADC结果并做 p.u. 转换
    ctl_input_callback();

    // 2. 控制：执行一拍控制算法（含电流环/调制器等）
    ctl_dispatch();

    // 3. 输出：更新 PWM/数模/IO
    ctl_output_callback();
}
```
注意事项：
- 采样点选择在电流波形平坦期（避开死区/换相瞬间）。
- 确保 `ctl_dispatch()` 内部计算量满足控制周期时限。

---

## 端口适配与单位制（p.u.）
- 所有控制计算尽量使用 `ctrl_gt`（定点/浮点抽象），通过 `float2ctrl/ctrl2float` 互转。
- ADC→p.u.：使用 `gain` 与 `bias` 标定（`adc_channel` 系列封装已提供）。
- PWM：统一使用 0~1 的占空比表示，再映射到芯片计数器比较值。

---

## 调试与验证流程
1) 从 `BUILD_LEVEL=1`（电压开环）开始验证 PWM、采样与调制器。
2) `BUILD_LEVEL=2`（电流闭环），验证坐标变换、电流采样与PI参数。
3) `BUILD_LEVEL=3`（编码器闭环），验证位置/速度链路。
4) `BUILD_LEVEL=4`（前馈等高级功能），整机联调。

辅助工具：
- 串口 AT 命令（ENABLE/POWEROFF/RST）。
- `dsa/` 模块的触发采集（波形抓取、事件定位）。
- `windows_simulink/` 的 RT Trace。

---

## 常见问题与排查清单
- 编译无法通过：
  - 包含路径/宏未配置，或 `gmp_src_mgr/` 未正确添加。
  - FPU/ABI 配置与库不匹配（浮点/软浮点）。
- 无 PWM 输出：
  - 未调用 `ctl_fast_enable_output()`；GPIO 复用或时钟未开。
- 电流噪声大/偏置异常：
  - 采样点与PWM不同步；未完成ADC偏置校准；增益/偏置参数错误。
- 控制周期抖动大：
  - ISR 里打印/复杂计算过多；DMA/缓存未配置；中断优先级冲突。

---

如需参考实现细节，可对比：
- `csp/c28x_syscfg/f280039c_demo/` 的 `xplt.*` 与工程配置。
- `ctl/suite/mcs_pmsm_nt/implement/*/xplt.*` 的平台层对接做法。
