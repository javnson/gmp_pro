# Base

`base` 提供不要求完整 GMP 运行时的可移植基础能力。

| 内容 | Facility 模块 ID | 文档 |
| --- | --- | --- |
| `gmp_base.h` | `core|base|port` | 基础时钟、断言、打印、临界区和内存服务契约 |
| `checksum` | `core|base|checksum|crc16` | [CRC16](checksum/README.md) |
| `ds` | `core|base|ds|list`、`core|base|ds|ring_buffer` | [数据结构](ds/README.md) |
| `locale` | `core|base|locale|utf8_unicode_encoder` | [字符转换](locale/README.md) |
| `tick` | 尚未注册 | [系统时钟契约](tick/README.md) |

基础组件应包含 `<gmp_type.h>` 和自身头文件，不应包含 `<gmp_core.h>`。
