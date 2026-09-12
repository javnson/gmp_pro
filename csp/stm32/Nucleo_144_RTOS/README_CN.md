# STM32 Nucleo-144 FreeRTOS 参考工程

[English](README.md)

本目录为 NUCLEO-H753ZI 提供一个显式开启的 FreeRTOS 参考工程，不修改、替换
或重新定义 `../Nucleo_144` 下的既有裸机工程。

当前规约的核心是“RTOS 管系统，GMP 管自己的非阻塞工作”：

- FreeRTOS 独占任务创建、优先级、延时、系统 tick 和内核启动；
- 一个高优先级 FreeRTOS 任务完成 GMP 初始化，并每 1 ms 执行一次有界的
  `gmp_base_loop()`；
- `core/pm` 仍是这个 GMP 任务内部的协作式、非阻塞函数调度器，不是第二个
  操作系统；
- ADC DMA 以 20 kHz 驱动控制中断，中断内不调用 FreeRTOS API；
- 独立用户任务每秒更新一次计数，证明应用可以继续使用 RTOS 增加线程。

## 启动顺序

```text
CubeMX 硬件初始化
  → 创建 GMP 服务任务和用户任务
  → vTaskStartScheduler
  → GMP 任务: gmp_base_prepare
  → GMP 任务: gmp_base_activate
       → gmp_csp_post_process
       → 启动 ADC DMA 和 TIM1 控制触发
  → GMP 任务: 每 1 ms 调用 gmp_base_loop
```

因此 `gmp_csp_post_process()` 不创建任务、也不启动内核；它只在 GMP/CTL/用户
对象都初始化完成后，最后开启控制数据流，避免初始化阶段的中断竞争。

## 工程边界

- `stm32h753zi_nucleo/stm32h753zi_nucleo.ioc`：CubeMX 权威硬件与 FreeRTOS
  配置；
- `stm32h753zi_nucleo/config/FreeRTOSConfig.h`：可审阅的内核配置；
- `src/user/rtos_app.c`：RTOS 任务创建与内核启动；
- `src/user/user_main.c`：运行在 GMP 服务任务内的 `core/pm` 任务表；
- `src/xplt/xplt.peripheral.c`：外设准备与 post-process 激活；
- `src/rtos`：STM32 FreeRTOS 的 GMP 循环适配器；
- `../Nucleo_144/src`：只读复用 UART Data Link、CTL 与公共外设头文件。

全局开关 `SPECIFY_GMP_OS_BACKEND` 默认是 `GMP_OS_BACKEND_NONE`；只有本工程
在 CMake 中选择 `GMP_OS_BACKEND_FREERTOS`，因此既有裸机工程默认行为不变。

## 构建与烧录

在 `stm32h753zi_nucleo` 目录执行：

```powershell
.\generate.ps1
.\build.ps1 -Configuration Release
.\flash.ps1
```

本机 STM32CubeMX 6.17 RC 能从 IOC 识别 FreeRTOS，但没有生成内核文件。
`tools/patch_generated_main.py` 会读取 IOC 选定的 H7 固件包版本，并从相同固件
包同步官方 FreeRTOS 源码，然后只修改 CubeMX 用户代码块。这一补偿不会改变
权威 IOC。

## 当前限制

- 当前只启用 UART GMP Data Link。裸机工程的 LwIP 使用 `NO_SYS=1` 轮询模型，
  不能直接放进多个 RTOS 任务，因此本参考工程不链接 LwIP；后续应单独采用
  `NO_SYS=0`/TCPIP 线程模型迁移以太网。
- 当前示例使用 `heap_4` 动态创建两个任务；量产项目建议改为静态任务和静态
  队列，并根据栈高水位重新定容。
- `core/pm` 仍存在固定从索引 0 扫描的公平性限制。本工程通过把 500 ms 心跳
  放在 1 ms Data Link 任务之前避免饥饿；根治方案仍应在兼容版本中引入轮转、
  执行预算与绝对周期语义。
- 优先级 4 的控制 ISR 高于 FreeRTOS `max syscall` 优先级 5，禁止调用任何
  FreeRTOS API。与任务共享的数据应采用原子标量、邮箱或版本化快照，不能把
  RTOS 临界区当作控制 ISR 的互斥锁。

板上验证结果见 [stm32h753zi_nucleo/validation.md](stm32h753zi_nucleo/validation.md)，
全仓库分析见
[多线程与 RTOS 兼容性分析](../../../manual/multithreading_rtos_architecture_analysis_cn.md)。
