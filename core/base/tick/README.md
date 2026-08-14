# Tick Contract

GMP 的默认粗粒度系统时钟接口是：

```c
time_gt gmp_base_get_system_tick(void);
```

CSP 应提供约 1 ms 分辨率、允许自然回绕的单调计数。计数宽度由
`GMP_PORT_TIME_T`/`time_gt` 决定，可由 CSP 选择为 32 位或 64 位。这里暂不
提供实现，也不注册独立 Facility；相关辅助函数目前声明在
`core/base/gmp_base.h`。
