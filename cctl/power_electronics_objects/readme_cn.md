# CCTL 电力电子对象

[English](readme.md) | **简体中文**

本目录提供实验性的 C++ 功率变换器、开关器件和电机模型，主要用于上位机分析与仿真。

当前稳定的离线仿真入口包括：

- `inverter/three_phase_average_inverter.hpp`：带导通电阻、二极管压降和平均死区误差的三相两电平平均逆变器。
- `motor_model/pmsm_average_model.hpp`：状态为 `id/iq/机械速度/机械角度` 的 PMSM 电磁与机械联合模型。

`buckboost_universal`、`dfig`、旧感应电机和开关级 MOSFET 目录仍属于历史实验代码。新 PMSM 仿真应从上述平均模型开始。这些对象不是嵌入式控制实现；正式控制代码仍应放在 `ctl/component` 或 suite 共用的 `src/` 目录中。
