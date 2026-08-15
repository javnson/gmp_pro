# GMP CTL 组件库

[English](readme.md) | **简体中文**

`ctl/component` 位于 `ctl/math_block` 之上、完整 `ctl/suite` 应用之下，提供可复用的 C 控制组件。当前目录按真实源码分为：

| 目录 | 当前内容 |
| --- | --- |
| `intrinsic` | basic、continuous、discrete、complex、advance、protection 控制与滤波组件 |
| `motor_control` | 电机接口、consultant、电流环、弱磁分配、机械环、轨迹、观测器和离线辨识 |
| `digital_power` | DCDC、三相逆变、单相逆变、MPPT 及基础接口 |
| `interface` | ADC、DAC、PWM、调制、偏置、增益和 SIL 接口抽象 |
| `dsa` | scope、Datalink scope、trigger、正弦分析和 TI DLOG |
| `hardware_preset` | 电机、逆变器、传感器和滤波器参数预设 |
| `kinematics` | 当前只有目录和注册占位，没有公开实现文件 |

## 使用边界

- 实时状态和运算使用 `ctrl_gt` 及 `ctl_*` 数学辅助函数；初始化和物理参数计算使用 `parameter_gt`。
- `ctl_step_*` 不做动态分配、阻塞 I/O、打印或芯片寄存器访问。
- 平台 ADC/PWM/编码器和通信映射放在 suite 目标的 `xplt` 中。
- 选择模块和源码时以 `tools/facilities_generator/src_mgr/gmp_framework_dic.json` 为准，并核对其直接依赖。
- `sys_compile`、`sys_sim`、`sys_hw` 分别表示编译、仿真/单元测试、实物验证，不能互相替代。

当前 `digital_power.h`、`motor_control.h` 和 `intrinsic.h` 聚合头仍包含已经不存在的历史 include，不能作为可靠的总入口。应包含具体模块头，或让工程的 `gmp_src_mgr` 选择并生成依赖。`dsa.h` 在本次校核时没有缺失 include。

模块清单与注册状态见 [组件模块表](COMPONENT_MODULE_CATALOG.md)，开发约束见 [组件开发规范](COMPONENT_DEVELOPMENT_SPEC.md)。具体函数签名必须以对应头文件为准。
