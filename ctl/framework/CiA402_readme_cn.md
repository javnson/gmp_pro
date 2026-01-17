# CiA 402 状态机库使用说明书

## 1. 简介

本模块提供了一个标准化的 CiA 402 状态机实现，用于管理驱动器的生命周期（从上电初始化、预充电、高压接通到闭环运行及故障处理）。它解耦了**状态流转逻辑**与**底层硬件操作**，您只需填充底层的硬件控制函数（HAL），即可获得标准的 CANopen/EtherCAT 驱动器行为。

状态/指令转换图如下图所示：

![CiA402_GMP状态转移图](Cia402_state_map.png)

## 2. 快速集成步骤

集成过程主要分为三个步骤：实现硬件接口、初始化状态机、在主循环中调度。

### 步骤 1: 实现硬件抽象层 (HAL)

库文件声明了一系列 `ctl_` (Control) 前缀的函数，但未给出具体实现。您需要在您的项目 `.c` 文件中实现这些函数，以便状态机能控制您的具体硬件。

**必须实现的接口清单：**

**A. 功率与继电器控制**

- `void ctl_enable_pwm()`: 开启 PWM 输出（如开启门极驱动器）。
- `void ctl_disable_pwm()`: 封锁 PWM（高阻态或关断）。
- `void ctl_enable_main_contactor()`: 闭合主继电器/接触器。
- `void ctl_disable_main_contactor()`: 断开主继电器。
- `void ctl_enable_precharge_relay()`: (可选) 闭合预充继电器。
- `void ctl_disable_precharge_relay()`: (可选) 断开预充继电器。

**B. 状态检查与校准**

- `fast_gt ctl_exec_dc_voltage_ready()`: 检查母线电压是否正常（返回 1 正常，0 不正常）。
- `fast_gt ctl_exec_adc_calibration()`: 执行电流零偏校准。
- `fast_gt ctl_check_encoder()`: 检查编码器通信是否正常。

**C. 抱闸与特殊功能 (视需求实现)**

- `ctl_release_brake()` / `ctl_restore_brake()`: 电机抱闸控制。
- `ctl_exec_rotor_alignment()`: 同步电机转子预定位。

### 步骤 2: 定义与初始化

在您的主控逻辑中定义状态机实例，并调用初始化函数。

```C
#include "cia402_state_machine.h"

// 1. 定义全局实例
cia402_sm_t g_drive_sm;

void init(void) {
    // 硬件底层初始化...
    
    // 2. 初始化状态机
    // 这会将状态重置为 NOT_READY_TO_SWITCH_ON，并加载默认回调函数
    init_cia402_state_machine(&g_drive_sm);
}
```

### 步骤 3: 周期调度

状态机需要被周期性调用，以处理状态跳转和超时逻辑。

如果需要多个子任务协同工作，建议使用`gmp_task`功能调度阻塞任务，在此情况下建议每1ms执行一次。

```C
void main_loop(void) {
    
        // 1. 更新控制字 (来源可以是 CANopen, EtherCAT, 或调试串口)
        // 例如：g_drive_sm.control_word.all = RxPDO_ControlWord;
        
        // 2. 调度状态机核心逻辑
        dispatch_cia402_state_machine(&g_drive_sm);
        
        // 3. 将状态字发送回上位机
        // TxPDO_StatusWord = g_drive_sm.state_word.all;
    
}
```

------

## 3. 控制与状态流转

### 标准启动序列 (Happy Path)

要使驱动器进入运行状态 (`Operation Enabled`)，上位机或主控逻辑需按以下顺序发送控制字（Control Word `0x6040`）：

1. **复位/初始化**: 确保无故障，Control Word = `0x0000`。
   - 状态机进入 `Switch On Disabled` (禁止合闸)。
2. **Shutdown (0x06)**: 发送 `0x0006`。
   - 状态机进入 `Ready to Switch On` (准备合闸)。
   - *硬件动作：执行 ADC 校准、检查母线电压。*
3. **Switch On (0x07)**: 发送 `0x0007`。
   - 状态机进入 `Switched On` (已合闸)。
   - *硬件动作：闭合主继电器，强电接通。*
4. **Enable Operation (0x0F)**: 发送 `0x000F`。
   - 状态机进入 `Operation Enabled` (允许运行)。
   - *硬件动作：释放抱闸，开启 PWM，电机开始受控。*

### 故障复位

当 `state_word` 的 Bit 3 (Fault) 为 1 时：

1. 发送控制字 `0x0080` (Bit 7 上升沿)。
2. 状态机将尝试清除故障并回到 `Switch On Disabled`。

------

## 4. 调试与配置

头文件中的宏定义允许您调整状态机行为以适应调试或特殊需求。

### 调试模式：跳过严格流程

如果您在调试初期不想按严格的 `0x06 -> 0x07 -> 0x0F` 顺序发送命令：

- **启用宏**: `#define CIA402_CONFIG_ENABLE_SEQUENCE_SWITCH`
- **效果**: 您可以直接发送 `0x0F`，状态机将自动处理中间跳转，快速进入运行状态。

### 禁用控制字解析

- **启用宏**: `#define CIA402_CONFIG_DISABLE_CONTROL_WORD_DEFAULT`
- **效果**: 初始化后，状态机不会解析 `sm->control_word`。您可以通过直接调用 API 函数 `cia402_transit()` 强制切换状态，这在某些不由总线控制的单机应用中很有用。

### 调整延时

通过修改 `CIA402_CONFIG_MIN_DELAY_*` 系列宏，您可以定义状态切换的最小驻留时间。

- 例如：继电器闭合后需要 100ms 稳定时间，可设置 `CIA402_CONFIG_MIN_DELAY_SWITCHON (100)`。

------

## 5. API 参考

| **函数名**                      | **描述**                                                     |
| ------------------------------- | ------------------------------------------------------------ |
| `init_cia402_state_machine`     | 初始化对象，设置默认回调，重置状态。                         |
| `dispatch_cia402_state_machine` | **核心函数**。解析控制字，检查跳转条件，执行回调，更新状态字。 |
| `cia402_update_status_word`     | 根据当前内部状态刷新 `state_word` 的位（如 Ready, Switched On, Fault 等）。 |
| `cia402_fault_request`          | 触发故障。将状态强行切换到 `FAULT_REACTION`，随后进入 `FAULT`。 |
| `get_cia402_state`              | 辅助工具：将状态字数值转换为枚举状态。                       |
| `get_cia402_control_cmd`        | 辅助工具：解析控制字意图（如解析出是 Shutdown 还是 QuickStop 命令）。 |

------

## 6. 常见问题 (FAQ)

Q: 为什么发送了 0x0F 却卡在 Switched On Disabled？

A1: 如果使能了控制字

1. 检查是否有故障位 (Bit 3) 置位。
2. 检查是否定义了严格顺序宏。如果是严格模式，必须先发 0x06，看到 Status Word 变为 `xxxx xxxx x01x 0001` 后，再发 0x07，最后发 0x0F。
3. 检查 `ctl_exec_dc_voltage_ready` 等硬件检查函数是否返回了 `false`。

A2: 如果没有使能控制字

在没有使能控制字的情况下，只能通过标志位`flag_fault_reset_request`置1的方法复位控制器。控制器复位后这个标志为将会自动清0。

Q: 如何处理急停 (Quick Stop)？

A: 发送控制字 0x000B (Quick Stop位为低电平有效，0=急停，1=正常)。状态机会进入 QUICK_STOP_ACTIVE 状态。您应在该状态的回调中执行快速减速逻辑。

Q: 如何添加自定义状态逻辑？

A: cia402_sm_t 结构体中包含函数指针（如 operation_enabled）。init 函数会挂载默认的 default_cb_fn_...。您可以编写自己的函数并覆盖指针：

C

```
// 自定义运行状态回调
cia402_sm_error_code_t my_operation_handler(cia402_sm_t* sm) {
    // 做自己的事...
    return default_cb_fn_operation_enabled(sm); // 保持原有逻辑
}

// 在初始化后覆盖
init_cia402_state_machine(&sm);
sm.operation_enabled = my_operation_handler;
```