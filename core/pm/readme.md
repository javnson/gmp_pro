# Process and Task Management

`pm` 提供任务、调度和状态管理，不负责完整系统启动。

| 模块 | Facility 模块 ID | 文档 |
| --- | --- | --- |
| 协作式函数调度器 | `core|pm|function_scheduler` | [function_scheduler](function_scheduler/readme.md) |
| Duff 宏状态机 | `core|pm|duff_fsm` | [duff_fsm](duff_fsm/README.md) |
| 状态机设计占位 | 尚未注册 | [state_machine](state_machine/README.md) |
| 旧 timing manager | 尚未注册 | [timing](timing/README.md) |
| 旧 C++ workflow | 尚未注册 | [workflow](workflow/README.md) |

调度器通过 `gmp_base_get_system_tick()` 获得约 1 ms 粗时钟。任务回调仍是该
调度器的运行模型；不需要调度框架的组件不应依赖 `pm`。
