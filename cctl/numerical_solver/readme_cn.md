# CCTL 数值求解器

[English](readme.md) | **简体中文**

本目录提供 C++ 固定步长数值求解器，主要面向主机侧实时或离线仿真。

- `fixed_vector.hpp`：基于 `std::array` 的无堆分配固定维度向量，允许编译器展开和自动向量化短循环。
- `explicit_euler.hpp`：显式欧拉单步求解器。
- `runge_kutta_4.hpp`：经典四阶 Runge–Kutta 单步求解器。

模型需要声明 `scalar_type`、`state_type`、`input_type`，并提供纯函数形式的：

```cpp
state_type derivative(scalar_type time,
                      const state_type& state,
                      const input_type& input) const;
```

求解器在一个步长内对输入采用零阶保持。RK4 会在四个中间状态上重新调用 `derivative()`，所以与状态相关的逆变器压降仍会在子阶段更新。
