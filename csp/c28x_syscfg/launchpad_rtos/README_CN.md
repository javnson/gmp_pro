# GMP C2000 LaunchPad FreeRTOS 参考工程

本目录提供面向 `LAUNCHXL-F280049C` 的 GMP + TI FreeRTOS 参考工程。它与
`csp/c28x_syscfg/launchpad` 裸机工程相互独立，不修改裸机工程的执行模型。

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
- 20 kHz ADC 控制 ISR 保持硬件中断形式，不进入 FreeRTOS 调度，也不调用 RTOS API。
- SCI ISR 只搬运接收数据，DL 协议解析和响应在 `gmpService` 中完成。
- `gmp_csp_post_process()` 在 RTOS 模式下不全局开中断；应用中断在 GMP 初始化完成后开启。
- GMP 临界区在任务上下文映射到 FreeRTOS 临界区。不要在控制 ISR 中调用这些任务上下文接口。

当前 SysConfig 创建两个静态任务：`gmpService` 使用 1024 words 栈，`userTask`
使用 256 words 栈。动态分配关闭，并在链接脚本中为 FreeRTOS 静态栈和堆显式分配
片上 RAM。

## 工程边界

- `LAUNCHXL_F280049C.syscfg` 是外设、FreeRTOS 配置和任务表的权威输入。
- `src/rtos/gmp_c28x_freertos.c` 是 RTOS 所有权适配层和用户任务示例。
- `src/xplt` 实现板级中断、控制循环和 Data Link 传输。
- `C2000Lib_F280049C/build_support/28004x_gmp_flash_lnk.cmd` 维护 Flash/RAM 布局。
- `F280049C_Debug`、`F280049C_Release` 和其中的 `syscfg` 内容均为生成物，不应提交。

目前只把 F280049C 纳入此 RTOS 参考工程。移植到其他 C2000 型号时，需要同时确认
TI FreeRTOS 端口可用性、CPU Timer 归属、中断向量以及 FreeRTOS 栈/堆的链接区间，
不能仅复制 SysConfig 文件。

## CCS 导入和构建

已验证的工具组合是 CCS 12.8.1、C2000Ware 5.04.00.00、SysConfig 1.21.0 和
TI C2000 Compiler 22.6.1.LTS。

在 CCS 中选择 `File > Import > CCS Projects`，工程根目录指向本目录，然后选择
`F280049C_Debug` 或 `F280049C_Release` 配置即可构建。也可以在仓库根目录运行：

```powershell
& .\csp\c28x_syscfg\launchpad_rtos\tools\build.ps1 `
    -Mode All -GenerateGmpSources
```

## 烧录和实板验证

连接 LAUNCHXL-F280049C 后，默认 XDS110 Application UART 为 `COM5`、波特率
115200。端口号变化时请显式覆盖：

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

## 新增用户任务

优先在 `LAUNCHXL_F280049C.syscfg` 的 FreeRTOS 任务表中增加任务，并在普通源文件中
实现其入口函数。除非任务确有更严格的实时期限，优先级应低于 `gmpService`；阻塞式
通信或耗时计算不应放入 GMP 轮询函数或 20 kHz 控制 ISR。跨任务共享 GMP 对象时，
还需要在业务层明确唯一所有者，或使用队列/互斥量保护，不能假设 GMP 内部对象天然
支持并发访问。

## Data Link 注意事项

SCI 错误恢复必须先检查 `SCI_getRxStatus()`。TI DriverLib 在清除 `SCI_INT_RXERR`
时会软复位 SCI，所以正常 RX FIFO 中断只能清除 `SCI_INT_RXFF`；仅当确实检测到错误
时才执行 RXERR 恢复。发送端按 FIFO 容量分批写入并在等待期间让出 CPU，避免长帧
尾部丢失和低优先级任务饥饿。
