# CTL框架使用指南 (CTL Framework Quick Start)

CTL库不仅提供了丰富的电机控制算法模块，还提供了一个名为 **CTL-Nano** 的上层应用框架，用于管理控制器的整个生命周期和任务调度。理解这个框架是成功使用本库的关键。

该框架主要由 `ctl_nano.h` 和 `ctl_dispatch.h` 两个文件定义，其核心思想是 **“状态机管理”** 和 **“分时调度”**。

### 1. 核心概念：`ctl_object_nano_t`

在 `ctl_nano.h` 中定义的核心结构体 `ctl_object_nano_t` 是整个控制系统的心脏。您可以把它想象成一个“主控板”，它包含了：

- **系统状态机 (`state_machine`)**: 管理控制器从上电到运行再到故障的每一步。
- **时间戳 (`isr_tick`, `mainloop_tick`)**: 跟踪高频和低频任务的执行节拍。
- **全局句柄 (`ctl_nano_handle`)**: 一个全局指针，让系统的任何部分都能访问到这个“主控板”。

### 2. 生命周期：状态机 (`ctl_nano_state_machine`)

控制器拥有一个明确的生命周期，由状态机严格管理，确保系统安全、有序地运行：

1. **`CTL_SM_PENDING` (等待)**: 上电后的初始状态，一切都被禁止。
2. **`CTL_SM_CALIBRATE` (校准)**: 在此状态下执行硬件自检，如ADC零点校准。
3. **`CTL_SM_READY` (就绪)**: 校准完成，等待启动指令。
4. **`CTL_SM_RUNUP` (启动)**: 执行电机启动程序，如参数辨识或开环预定位。
5. **`CTL_SM_ONLINE` (在线运行)**: 进入主控制模式，电机正常闭环运行。
6. **`CTL_SM_FAULT` (故障)**: 检测到严重错误，立即停机并等待处理。

### 3. 任务调度：高频与低频任务

CTL框架将控制任务分为两类：

- **高频任务 (ISR中执行)**: 在 `ctl_dispatch.h` 中，`gmp_base_ctl_step()` 是您需要在**定时器中断服务程序 (ISR)** 中调用的唯一入口。它负责执行对实时性要求最高的任务，如：
  - **输入 (`ctl_fmif_input_stage_routine`)**: 读取ADC电流、编码器位置等。
  - **核心计算 (`ctl_fmif_core_stage_routine`)**: 执行FOC、SMO等核心算法。
  - **输出 (`ctl_fmif_output_stage_routine`)**: 更新PWM占空比。
- **低频任务 (主循环中执行)**: 在 `ctl_nano.h` 中，`ctl_fm_state_dispatch()` 是您需要在**主循环 (`while(1)`)** 中调用的函数。它负责执行实时性要求不高的任务，包括：
  - **状态机调度**: 根据当前状态，调用对应的处理函数（如 `ctl_fmif_sm_online_routine`）。
  - **监控 (`ctl_fmif_monitor_routine`)**: 如向上位机发送调试数据。
  - **安全检查 (`ctl_fmif_security_routine`)**: 检查过流、过压等故障。

### 4. 如何使用：用户的责任

要将CTL框架集成到您的项目中，您需要做两件事：

1. **定义全局句柄**: 在您的一个 `.c` 文件中，定义并初始化 `ctl_object_nano_t` 结构体和全局指针 `ctl_nano_handle`。

   ```
   ctl_object_nano_t g_my_controller_obj;
   ctl_object_nano_t *ctl_nano_handle = &g_my_controller_obj;
   ```

2. **实现`ctl_fmif_\*`回调函数**: 框架已经搭好了骨架，您需要根据您的具体硬件和应用逻辑，去实现 `ctl_nano.h` 中声明的那些 `ctl_fmif_*` 函数。例如，在 `ctl_fmif_input_stage_routine` 中编写读取您MCU的ADC寄存器的代码；在 `ctl_fmif_sm_online_routine` 中编写处理用户按键、切换电机运行模式的逻辑。

通过这种方式，CTL框架将复杂的实时调度和状态管理与您的具体应用代码清晰地分离开来，让您可以更专注于实现核心的控制算法。

### 5. 简化模式：直接调度 (Simplified Mode: Direct Dispatch)

如果您的项目不需要 `CTL-Nano` 提供的复杂状态机管理，您可以选择一种更直接、更简单的调度模式。

**前提条件**: 在您的工程配置中，**不要**定义 `SPECIFY_ENABLE_CTL_FRAMEWORK_NANO` 宏。

**工作逻辑**: 在这种模式下，当您在ISR中调用 `gmp_base_ctl_step()` 时，它将不再执行 `Nano` 框架的逻辑，而是会依次调用三个由您定义的**全局回调函数**：

1. `ctl_input_callback()`
2. `ctl_dispatch()`
3. `ctl_output_callback()`

**如何使用：用户的责任** 您需要在您的代码中实现这三个函数，并将控制流程清晰地分到这三个阶段中：

1. **实现 `ctl_input_callback()`**:
   - **作用**: 负责所有的数据**输入**和**采样**。
   - **内容**: 在这里编写读取ADC、编码器、霍尔传感器等硬件外设的代码，并将采样值更新到您的顶层控制器结构体中。
2. **实现 `ctl_dispatch()`**:
   - **作用**: 负责核心控制算法的**计算**。
   - **内容**: 在这里调用您选择的顶层控制器模块的 `step` 函数，例如 `ctl_step_pmsm_ctrl()`。这个函数会执行FOC运算并计算出最终的PWM占空比。
3. **实现 `ctl_output_callback()`**:
   - **作用**: 负责所有数据的**输出**和**执行**。
   - **内容**: 在这里编写将 `step` 函数计算出的PWM占空比更新到MCU的定时器比较寄存器的代码。

**示例代码**: 您的中断服务程序看起来会非常简洁：

```
// 在您的一个 .c 文件中实现这三个回调
void ctl_input_callback(void) {
    // 读取ADC和编码器值...
    update_adc_inputs(&g_my_pmsm_controller.mtr_interface);
    update_encoder_inputs(&g_my_pmsm_controller.mtr_interface);
}

void ctl_dispatch(void) {
    // 执行FOC核心计算
    ctl_step_pmsm_ctrl(&g_my_pmsm_controller);
}

void ctl_output_callback(void) {
    // 将计算结果写入PWM寄存器
    update_pwm_outputs(g_my_pmsm_controller.pwm_out);
}

// 在您的定时器中断服务程序中
void TIM1_UP_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        
        // 只需调用这一个函数
        gmp_base_ctl_step();
    }
}
```

这种模式虽然没有状态机提供的安全保障和生命周期管理，但它结构简单，执行路径直接，非常适合快速原型开发或逻辑相对简单的项目。