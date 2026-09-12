# STM32 Nucleo 工程 CMake、Keil 与 CubeIDE 兼容性报告

## 1. 结论

本次检查覆盖 `csp/stm32/Nucleo_32`、`Nucleo_64`、`Nucleo_144` 和
`Nucleo_144_RTOS` 下全部 11 个 IOC 工程。

- CMake/GNU Arm：11 个工程全部构建通过。
- Keil/ARMCC5：全部单核工程构建通过；STM32H755 的 CM4、CM7 两个目标均构建通过。
- STM32CubeIDE：除 STM32C092 的本机 IDE 版本兼容问题外，其余工程全部通过 Release 构建，均为 0 错误、0 警告；H755 的 CM4、CM7 均通过。
- STM32C092：CMake 和 Keil 已通过；CubeMX 6.17 能正确生成 CubeIDE 工程，但本机 CubeIDE 1.16.1 不包含 STM32C092 的器件数据库，构建器在编译前以 `Unknown target` 退出。因此该项需要在包含 STM32C092 器件支持的 CubeIDE 版本上补做最终实测，不能记作 1.16.1 下通过。
- STM32G431RB 原先只有 IOC 和规约文件，不是完整工程。本次已补齐 CMake、GMP 接入脚本、CubeIDE 和 Keil 生成/同步流程，并在三个工具链上通过。

## 2. 验证环境

本轮实际使用的主要工具如下：

- GNU Arm Embedded Toolchain 14.3.1；
- CMake + Ninja；
- Keil uVision/ARMCC5，编译器 V5.06 update 6 build 750；
- STM32CubeIDE 1.16.1；
- STM32CubeMX 6.17.0-RC5；
- NUCLEO-H753ZI 实板及 ST-Link。

H753 裸机和 H753 FreeRTOS 工程此前已在连接的 NUCLEO-H753ZI 上完成下载和运行验证；本轮在此基础上重新检查三种构建系统的一致性。

## 3. 构建矩阵

| 目录/工程 | CMake | Keil | CubeIDE | 备注 |
| --- | --- | --- | --- | --- |
| Nucleo_32/stm32g431kb_nucleo | 通过 | 通过，0/0 | 通过，0/0 | 单核 Cortex-M4 |
| Nucleo_64/stm32c092rc_nucleo | 通过 | 通过，0/0 | 需新版 IDE 实测 | 1.16.1 不认识 STM32C092RCTx |
| Nucleo_64/stm32f411re_nucleo | 通过 | 通过，0/0 | 通过，0/0 | 单核 Cortex-M4 |
| Nucleo_64/stm32g431rb_nucleo | 通过 | 通过，0/0 | 通过，0/0 | 本次补齐完整工程 |
| Nucleo_64/stm32g474re_nucleo | 通过 | 通过，0/0 | 通过，0/0 | 修正 Keil scatter 路径 |
| Nucleo_64/stm32g491re_nucleo | 通过 | 通过，0/0 | 通过，0/0 | 单核 Cortex-M4 |
| Nucleo_64/stm32h533re_nucleo | 通过 | 通过，0/0 | 通过，0/0 | 单核 Cortex-M33 |
| Nucleo_64/stm32u083rc_nucleo | 通过 | 通过，0/0 | 通过，0/0 | 单核 Cortex-M0+ |
| Nucleo_144/stm32h753zi_nucleo | 通过 | 通过，0/0 | 通过，0/0 | 已做 H753 实板验证 |
| Nucleo_144/stm32h755zi_nucleo CM4/CM7 | 双目标通过 | CM4、CM7 均通过，0/0 | CM4、CM7 均通过，0/0 | 修正双目标源组隔离 |
| Nucleo_144_RTOS/stm32h753zi_nucleo | 通过 | 通过，0/0 | 通过，0/0 | FreeRTOS，已做 H753 实板验证 |

表中 Keil/CubeIDE 的 `0/0` 表示 0 错误、0 警告。CMake 链接时部分工程仍会显示 newlib-nano 的 `_read/_write/_close/_lseek` stub 提示；它们不影响固件生成，但若以后启用标准输入输出，应统一实现 syscall 重定向。

## 4. 原问题分析

CubeMX 只了解 IOC 中的 STM32 外设和中间件源文件，并不了解 GMP 的以下构建输入：

- 家族级 `src/user` 和 `src/xplt`；
- SDPE 生成的目标配置；
- `gmp_src_mgr` 生成的头文件和源文件；
- RTOS 工程的 GMP/FreeRTOS 适配层；
- CMake 中额外设置的宏定义和包含目录。

因此，直接打开 CubeMX 原生生成的 CubeIDE 或 Keil 工程会出现源文件缺失、头文件搜索路径缺失或链接符号缺失。RTOS 工程还曾保留已不存在的 LwIP 链接资源；H755 双核 Keil 工程则会把 CM4/CM7 的 GMP 源组错误合并，导致重复符号。

## 5. 统一机制

本次把 CMake 的 `compile_commands.json` 定为三种构建系统的输入真值。通用工具完成以下工作：

1. 从编译数据库读取实际参与目标的 C/C++ 源文件、包含目录和宏定义；
2. 将它们同步到 CubeIDE `.project/.cproject`；
3. 将它们同步到 Keil `.uvprojx`，并把 FreeRTOS GCC portable 层映射为 RVDS portable 层；
4. 删除不存在或不属于当前目标的旧 IDE 链接资源；
5. 修正 CubeMX 生成的错误 startup/scatter 相对路径；
6. 对 H755 按 CM4/CM7 分别过滤编译数据库，并使用“同序镜像组”：当前核启用，另一核使用空的禁用占位组，防止 uVision 跨目标合并源文件；
7. 将 CubeMX 在另一核中标记为 `IncludeInBuild=0`、但实际属于当前 CMake 目标的 HAL 文件重新启用。

各板卡的 `build.ps1` 在 CMake 构建成功后调用统一同步入口，所以 CMake 目标变化后不再需要人工维护两份 IDE 源文件列表。

## 6. 推荐工作流

首次生成或 IOC 改动后，在仓库根目录执行：

```powershell
& csp/stm32/common/tools/generate_ide.ps1 `
  -BoardDir csp/stm32/Nucleo_144/stm32h753zi_nucleo `
  -Ide All

& csp/stm32/Nucleo_144/stm32h753zi_nucleo/build.ps1 `
  -Configuration Release
```

第一步让 CubeMX 同时生成 CubeIDE 和 Keil 元数据，并在结束后恢复原始 IOC；第二步执行 SDPE/GMP 源同步、CMake 构建，并以 CMake 编译数据库更新两个 IDE 工程。之后可以直接在 CubeIDE 或 Keil 中打开并构建。

只需要一个 IDE 时，可把 `-Ide All` 改为 `CubeIDE` 或 `Keil`。H753 FreeRTOS 工程也可直接使用其目录内的 `generate.ps1 -Ide All`。

## 7. 版本与维护约束

- CubeMX 和 CubeIDE 的器件数据库必须匹配目标芯片。尤其是 STM32C092，CubeIDE 1.16.1 不满足要求；升级后应重新生成并进行一次本机 Release 构建。
- 不应在 CubeIDE/Keil 中长期手工维护 GMP 源列表；应修改 CMake 后重新运行 `build.ps1`。
- H755 必须分别验证 CM4 和 CM7，不能以父工程成功导入代替两个子目标的实际链接。
- Keil 使用 ARMCC5 时，FreeRTOS 中供汇编器使用的中断优先级必须是汇编期常量，不能依赖 C 类型转换表达式。
- 实板验证优先使用 H753；其他芯片当前结论是交叉编译和链接通过，不等同于每一块板均已做外设运行验证。

## 8. 后续建议

1. 安装带 STM32C092 器件支持的 CubeIDE，补齐唯一未完成的原生 IDE 实测；
2. 在 CI 中保留 11 项 CMake 矩阵，并在具备许可证的 Windows runner 上增加 Keil 命令行矩阵；
3. 将 CubeIDE headless build 纳入周期性回归，至少覆盖 G431KB、F411、H753、H755 双核和 H753 FreeRTOS；
4. 后续新增 STM32 工程时，必须同时具备 IOC、CMake、SDPE requirement、GMP main patch 和三工具链同步入口。
