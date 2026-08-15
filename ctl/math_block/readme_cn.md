# CTL 数学模块

[English](readme.md) | **简体中文**

`ctl/math_block/gmp_math.h` 是可复用 CTL C 算法的数值入口。它只装配选定的
`ctrl_gt` / `parameter_gt` 后端、转换、常量、轻量向量矩阵和坐标变换，不引入
CSP、设备层或 GMP runtime。

## 数值域

- `real_gt` 是可选的源数据/离线精度，嵌入式算法通常不保存该类型对象。
- `parameter_gt` 为 `float` 或 `double`，用于配置、整定、初始化和慢速分析；非线性运算使用 `param_*` 接口。
- `ctrl_gt` 用于实时状态和计算，可能是浮点或定点，因此乘除、饱和、非线性运算和转换必须使用 `ctl_*` 契约。

跨域转换统一使用 `real2param`、`real2ctrl`、`param2ctrl` 和 `ctrl2param`；`float2ctrl`、`ctrl2float` 已弃用。

## 角度契约

`ctl_sin`、`ctl_cos`、`ctl_tan` 使用周标幺角；其 `_rad` 版本使用弧度。
`param_sin`、`param_cos`、`param_tan` 使用弧度，`param_sin_pu`、
`param_cos_pu` 使用周标幺角。`ctl_atan2` 返回弧度，`ctl_atan2_pu` 返回周。
一周等于 `2π` 弧度。

## 当前内容与验证

当前 C 目录包括 `ctrl_gt`、`parameter_gt`、`const`、`coordinate`、
`vector_lite`、`matrix_lite`、`complex_lite` 和 `utilities`。公共函数名必须以这些
头文件为准；旧文档中的 `ctl_clarke_abc2ab0`、`ctl_park_ab2dq`、
`ctl_svpwm_calc` 已不是当前 API。

`gmp_math.hpp` 提供无堆分配的定长 C++ 向量和矩阵模板。`ctl/math_block/tests`
覆盖配置的 float/double、TI IQmath 契约和自定义数值类型 C++ 模板。更详细的后端与
审计规则见 [数值契约英文参考](README_EN.md)。
