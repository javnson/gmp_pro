# GMP C2000 LaunchPad FreeRTOS 参考工程

本目录提供面向 GMP C2000 LaunchPad 系列的 TI FreeRTOS 参考工程。它与
`csp/c28x_syscfg/launchpad` 裸机工程相互独立，不修改裸机工程的执行模型。

## 支持的平台

一个 CCS 工程同时提供以下 16 个构建配置，用户只需切换活动配置即可切换目标板：

| Board 参数 | CCS 配置 | FreeRTOS CPU 时钟 | 构建验证 |
| --- | --- | ---: | --- |
| `F2800137C` | `F2800137C_Debug` / `F2800137C_Release` | 120 MHz | 通过 |
| `F280025C` | `F280025C_Debug` / `F280025C_Release` | 50 MHz | 通过 |
| `F280039C` | `F280039C_Debug` / `F280039C_Release` | 60 MHz | 通过 |
| `F280049C` | `F280049C_Debug` / `F280049C_Release` | 100 MHz | 通过，已实板验证 |
| `F28377S` | `F28377S_Debug` / `F28377S_Release` | 200 MHz | 通过 |
| `F28379D` | `F28379D_Debug` / `F28379D_Release` | 200 MHz | 通过 |
| `F28P55X` | `F28P55X_Debug` / `F28P55X_Release` | 150 MHz | 通过 |
| `F28P65X` | `F28P65X_Debug` / `F28P65X_Release` | 200 MHz | 通过 |

这里的“构建验证”表示 SysConfig 生成、C2000 编译和链接全部完成；当前只有已连接的
LAUNCHXL-F280049C 完成了烧录及运行时验收。其他板卡在正式发布前仍应分别执行实板
时钟、中断、控制回路和 Data Link 验收。

## 执行模型

FreeRTOS 拥有系统调度权，GMP 的任务管理仍是非阻塞的协作式函数调度器，作为
一个高优先级 RTOS 任务运行：

```text
main / CSP 初始化
        |
        +-- 屏蔽应用外设中断
        +-- FreeRTOS_init() -> 启动调度器
                  |
                  +-- gmpService，优先级 6，周期 1 ms
                  |     gmp_base_prepare()
                  |     gmp_base_activate()
                  |     开启应用中断
                  |     gmp_base_loop()
                  |
                  +-- userTask，优先级 1，周期 1 s
                        独立用户任务示例
```

- CPU Timer2/INT14 由 TI FreeRTOS 端口占用，系统 tick 为 1 kHz。
- ADC 控制 ISR 保持硬件中断形式，不进入 FreeRTOS 调度，也不调用 RTOS API。
- Scope 在 `ctl_dispatch()` 内随每次控制中断采样，参考配置为 20 kHz；需要降采样时使用 Scope 协议自身的 `sample_divider`，CSP 不做隐藏固定分频。
- SCI ISR 只搬运接收数据，DL 协议解析和响应在 `gmpService` 中完成。
- `gmp_csp_post_process()` 在 RTOS 模式下不全局开中断；应用中断在 GMP 初始化完成后开启。
- GMP 临界区在任务上下文映射到 FreeRTOS 临界区。不要在控制 ISR 中调用这些任务上下文接口。

每个平台的 SysConfig 都创建两个静态任务：`gmpService` 使用 1024 words 栈，
`userTask` 使用 256 words 栈。动态分配关闭，并在各芯片链接脚本中为 FreeRTOS
静态栈和堆显式分配片上 RAM。

## 工程边界

- 根目录下对应目标板的 `LAUNCHXL_*.syscfg` 是外设、FreeRTOS 配置和任务表的权威输入。
- 每个 CCS 配置只包含自己的 SysConfig 文件、`C2000Lib_<board>` 和 SDPE requirements；其他平台均被排除。
- `src/rtos/gmp_c28x_freertos.c` 是所有平台共用的 RTOS 所有权适配层和用户任务示例。
- `src/xplt` 实现板级中断、控制循环和 Data Link 传输。
- 各 `C2000Lib_<board>/build_support/*_gmp_flash_lnk.cmd` 维护对应芯片的 Flash/RAM 布局和 FreeRTOS 段。
- `<board>_Debug`、`<board>_Release` 及其中的 `syscfg` 内容均为生成物，不应提交。

## CCS 导入、切换和构建

已验证的工具组合是 CCS 12.8.1、C2000Ware 5.04.00.00、SysConfig 1.21.0 和
TI C2000 Compiler 22.6.1.LTS。

在 CCS 中选择 `File > Import > CCS Projects`，工程根目录指向本目录。导入后在
`Build Configurations > Set Active` 中选择表格中的目标配置，然后正常构建即可；
不需要复制工程或手工替换 SysConfig 文件。

命令行通过 `-Board` 快速切换平台，例如：

```powershell
& .\csp\c28x_syscfg\launchpad_rtos\tools\build.ps1 `
    -Board F28379D -Mode All -GenerateGmpSources
```

`-Board` 可取表格中的八个值；`-Mode` 可取 `Debug`、`Release` 或 `All`。默认目标仍为
`F280049C`，因此现有脚本调用保持兼容。

## F280049C 烧录和实板验证

目前仓库提供的自动烧录、串口验收和硬件状态采样脚本针对已连接的
LAUNCHXL-F280049C。默认 XDS110 Application UART 为 `COM5`、波特率 115200；
端口号变化时请显式覆盖：

```powershell
& .\csp\c28x_syscfg\launchpad_rtos\tools\flash_f280049c.ps1 -Mode Debug
& .\csp\c28x_syscfg\launchpad_rtos\tools\smoke_f280049c.ps1 `
    -Port COM5 -BaudRate 115200
```

硬件状态采样可在工程目录运行：

```powershell
& C:\ti\ccs1281\ccs\ccs_base\scripting\bin\dss.bat `
    .\tools\hardware_probe_f280049c.js
```

2026-09-12 的实板验收覆盖了 FreeRTOS 调度、GMP 轮询任务、独立用户任务、20 kHz
控制 ISR，以及 DL 信息查询、长帧回显、参数、内存和 400 x 2 float32 Scope 数据。
所有 DL CRC、FIFO、超时和 SCI overrun 计数均为 0。正常物理控制模式没有发布 PIL
facility，因此测试会按设备实际发布的能力跳过 PIL。

Scope 实时路径修正后的 5 秒板上采样得到 `control_isr_runs=99898`、
`dl_scope_control_steps=99898`，两者完全一致；Scope 发布采样率为 20000 Hz，运行时
分频为 0。这证明采样由控制 ISR 中的 `ctl_dispatch()` 逐次驱动，而不是由 RTOS
轮询任务或 CSP 固定分频驱动。

## 新增用户任务

优先在目标板的 `LAUNCHXL_*.syscfg` FreeRTOS 任务表中增加任务，并在普通源文件中
实现其入口函数。除非任务确有更严格的实时期限，优先级应低于 `gmpService`；阻塞式
通信或耗时计算不应放入 GMP 轮询函数或控制 ISR。跨任务共享 GMP 对象时，还需要在
业务层明确唯一所有者，或使用队列/互斥量保护，不能假设 GMP 内部对象天然支持并发访问。

## Data Link 注意事项

SCI 错误恢复必须先检查 `SCI_getRxStatus()`。TI DriverLib 在清除 `SCI_INT_RXERR`
时会软复位 SCI，所以正常 RX FIFO 中断只能清除 `SCI_INT_RXFF`；仅当确实检测到错误
时才执行 RXERR 恢复。发送端按 FIFO 容量分批写入并在等待期间让出 CPU，避免长帧
尾部丢失和低优先级任务饥饿。
