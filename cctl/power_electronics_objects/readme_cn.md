# CCTL 电力电子对象

[English](readme.md) | **简体中文**

本目录提供实验性的 C++ 功率变换器、开关器件和电机模型，主要用于上位机分析与仿真。

当前包含通用 Buck-Boost、DFIG、PMSM、通用电机和 MOSFET 等模型。这些对象不是默认的嵌入式控制实现；正式控制代码通常应放在 `ctl/component` 或 suite 共用的 `src/` 目录中。
