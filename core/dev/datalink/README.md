# Data Link

本目录包含数据链路核心以及 tunable、memory perspective、scope 和 PIL
服务。协议说明见 [English](readme_dl_protocol.md) 和
[简体中文](readme_dl_protocol_cn.md)。

核心 Facility 模块 ID 为 `core|dev|datalink|_internal`，并按功能选择
`pil`、`scope`、`mem_monitor`、`variable_monitor` 等子模块。数据链路依赖
`base` 的时钟和 CRC 服务，但不应依赖完整 `rt`。



使用时可以直接使用datalink.h或者mem_presp.h等不带后缀的头文件，这个模块将会根据当前环境byte的大小自动选择应当使用u8版本还是u16版本。
