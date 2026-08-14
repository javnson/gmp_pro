# Peripheral Drivers

`driver` 保存基于 `core/dev/peripheral_port.h` 的具体阻塞式外设驱动。
`driver.h` 是这些驱动的共同编译契约。驱动按器件用途分目录：application、
dac、display、encoder、excitation、gpio、meter、potentiometer、RTC 和 sensor。

Facility 模块 ID 使用 `core|dev|driver|<category>|<device>`。驱动不得直接
包含 `<gmp_core.h>`，也不得要求完整 GMP 运行时；CSP 负责实现其使用的
`gmp_hal_*` 接口。驱动同样不得反向依赖 CTL：例如 AS5048A 只维护并输出
14 位原始角度，偏置、极对数、圈数和标幺值换算由选择该驱动的控制模块装配。
