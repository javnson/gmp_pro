# GMP CCTL——C++ 控制器模板库

[English](readme.md) | **简体中文**

`cctl` 保存实验性、可复用的 C++ 控制与电力电子对象。它是以 C 语言为主的 `ctl` 模块的补充，矩阵和数值计算可以采用 Eigen 提供的类型。

## 当前模块

| 目录 | 作用 |
| --- | --- |
| [`numerical_solver`](numerical_solver/readme_cn.md) | 数值方程与求解器实验 |
| [`power_electronics_objects`](power_electronics_objects/readme_cn.md) | 变换器和被控对象的 C++ 模型 |
| [`dsa`](dsa/README_CN.md) | 预分配 SPSC 无锁定长记录环等可复用数据流结构 |
| `peripheral_if` | TI DSP 风格的 ADC、中心对齐互补 ePWM、eQEP 与固定倍率分频模型 |
| `circuit_solver` | 历史 Python 改进节点分析实验 |
| `component` | 尚未由 `cctl.hpp` 导出的早期 C++ 接口实验 |

当前硬件 suite 默认仍采用 `ctl`。正式嵌入式控制工程应优先从 `ctl/component` 和 `ctl/suite` 开始；只有在确实需要 C++ 类型或上位机数值模型时再使用 CCTL。

## 已验证的 PMSM 平均值仿真核心

`cctl.hpp` 汇总了当前通过离线测试的固定维度向量、Euler/RK4 求解器、三相平均逆变器和 PMSM 平均模型。模型为纯 C++11 头文件，不依赖 Eigen，也不包含 UDP/TCP 或控制器算法耦合。

独立验证入口位于 `tb/pmsm_average_model_test`。它检查求解器收敛阶、锁轴 RL 解析响应、自由减速解析响应、abc/dq 功率不变性、转矩公式和死区平均压降。

`ctl/suite/mcs_pmsm_nt/project/cctl` 进一步把开关型 MNA 主电路、
`pmsm_cs` 电流源电机、上述外设和现有 PMSM 控制器直接链接在同一个进程中；
100 ns 被控对象通过固定 500 分频驱动 20 kHz 控制器，不需要网络通信。
该工程的三个仿真线程由 `csp/cctl` 管理，仿真与文件线程之间复用 `dsa` 的
32 MB 无锁记录环。
