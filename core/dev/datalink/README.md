# Data Link

本目录包含数据链路核心以及 Tunable、Memory Perspective、Scope 和 PIL
设施。协议说明见 [English](readme_dl_protocol.md) 和
[简体中文](readme_dl_protocol_cn.md)。

核心 Facility 模块 ID 为 `core|dev|datalink|_internal`，并按功能选择
`pil`、`scope`、`mem_monitor`、`variable_monitor` 等子模块。数据链路依赖
`base` 的链表、时钟和 CRC 服务，但不应依赖完整 `rt`。

每个子模块以 `gmp_dl_facility_t` 作为第一个成员：其中链表节点必须是
第一个字段，设施类型必须是第二个字段。应用初始化子模块后，使用
`gmp_dev_dl_append_facility()` 显式串入 Data Link；收到完整报文后只需调用
`gmp_dev_dl_dispatch_rx()`。核心会检查链表、命令范围和冲突，自动路由命令，
并通过 `INFO (0x02)` 汇报已注册设施。整个 DL 任务的执行次数保存在
`gmp_datalink_t.service_run_count` 中。

应用应包含 `datalink.h`、`mem_presp.h` 等不带后缀的公共头文件。模块根据
当前平台最小寻址单元自动选择 u8 或 u16 后端，不应直接包含后端文件。
