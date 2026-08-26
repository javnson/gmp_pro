# CCTL 数值求解器

[English](readme.md) | **简体中文**

本目录提供 C++ 固定步长数值求解器，主要面向主机侧实时或离线仿真。

- `fixed_vector.hpp`：基于 `std::array` 的无堆分配固定维度向量，允许编译器展开和自动向量化短循环。
- `fixed_point.hpp`：带饱和运算的有符号 32 位 Q 格式标量；生成的 MNA
  定点模型使用 64 位中间量完成乘除运算。
- `explicit_euler.hpp`：显式欧拉单步求解器。
- `runge_kutta_4.hpp`：经典四阶 Runge–Kutta 单步求解器。

模型需要声明 `scalar_type`、`state_type`、`input_type`，并提供纯函数形式的：

```cpp
state_type derivative(scalar_type time,
                      const state_type& state,
                      const input_type& input) const;
```

求解器在一个步长内对输入采用零阶保持。RK4 会在四个中间状态上重新调用 `derivative()`，所以与状态相关的逆变器压降仍会在子阶段更新。

`fixed_point32<FractionalBits>` 只负责定点算术，并不承担量程分析。CCTL Studio
的 MNA 生成器负责选择 2 的幂次标幺基值，为不同矩阵族自动选择独立的系数
Q 格式并嵌入量化后的系数，同时使用 64 位混合 Q 点积累加器；外部
输入在电路入口转换为定点数，仅在公开输出边界恢复为浮点数。用于正式工程时仍应
明确给出输入满量程，并将定点结果与 Eigen 参考模型对比。
