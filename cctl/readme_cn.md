# GMP CCTL——C++ 控制器模板库

[English](readme.md) | **简体中文**

`cctl` 保存实验性、可复用的 C++ 控制与电力电子对象。它是以 C 语言为主的 `ctl` 模块的补充，矩阵和数值计算可以采用 Eigen 提供的类型。

## 当前模块

| 目录 | 作用 |
| --- | --- |
| [`numerical_solver`](numerical_solver/readme_cn.md) | 数值方程与求解器实验 |
| [`power_electronics_objects`](power_electronics_objects/readme_cn.md) | 变换器和被控对象的 C++ 模型 |

当前硬件 suite 默认仍采用 `ctl`。正式嵌入式控制工程应优先从 `ctl/component` 和 `ctl/suite` 开始；只有在确实需要 C++ 类型或上位机数值模型时再使用 CCTL。
