# CCTL 数值求解器

[English](readme.md) | **简体中文**

本目录提供 C++ 数值求解器，主要面向内存资源相对充足的仿真或主机环境。求解器可以依赖由 C 语言实现的底层数学运算。

- `solver_base.hpp`：求解器公共基础接口。
- `runge_kutta_4.hpp`：四阶 Runge–Kutta 求解器。
