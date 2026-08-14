# Runtime

`rt` 装配完整 GMP 运行框架。它包含：

- `gmp_runtime.h`：初始化、循环和入口流程；
- `csp_port.h`：CSP 生命周期与运行时钩子；
- `src/gmp_runtime.c`：默认打印、断言、弱入口和运行时服务；
- `gmp_logo.inl`：可选启动标识；
- `gmp_logo_lite.inl`：面向 RAM/Flash 紧张目标的单行标识；
- 仓库根目录 `gmp_core.h`/`gmp_core.hpp`：完整框架入口。

Facility 模块 ID 为 `core|rt`，依赖 `core|std`、`core|base|port` 和
`core|dev|_internal`。只使用独立 GMP 组件时不应选择本模块。

在用户或 CSP 配置中把 `SPECIFY_GMP_LOGO_MODE` 设为
`GMP_LOGO_MODE_LITE` 即可选择 Lite 版本；设为
`GMP_LOGO_MODE_DISABLED` 则完全去除 Logo。
