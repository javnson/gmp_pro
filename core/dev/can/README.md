# CAN Service

`can.h`、`can_hook.h` 和 `gmp_can_service.c` 提供静态异步 CAN 服务及 CSP
钩子契约，Facility 模块 ID 为 `core|dev|can`。队列存储由调用者静态提供，
临界区由 `core|base|port` 服务实现。
