# Legacy Timing Manager

`timing_manager.h` 是旧设计草案，当前包含未完成的句柄和 tick 调用实现，
不注册为 Facility。新模块应直接使用 `core/base/gmp_base.h` 的粗时钟
契约，或在明确需要多时钟后重新设计该模块。
