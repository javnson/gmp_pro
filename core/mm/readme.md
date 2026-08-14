# Memory Management

`mm` 是独立可选的内存管理层。当前 `block_mem.h` 和
`src/gmp_mm_block_memory.c` 提供静态块内存管理，Facility 模块 ID 为
`core|mm`（注册键为 `mm`）。

实现只依赖标准类型；只有定义 `SPECIFY_GMP_BLOCK_MEMORY_ENABLE` 时才启用。
实时路径不应隐式分配内存。
