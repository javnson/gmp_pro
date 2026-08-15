# CTL 数字电源组件

[English](readme.md) | **简体中文**

当前实现按真实目录分为：

| 目录 | 已实现范围 |
| --- | --- |
| `basic` | `protectoion_strategy.h`、标准 SIL 数据接口、虚拟阻抗和 `ctl_dp_basic.c` |
| `dcdc` | Buck、Boost、CLLLC、FSBB 及公共 DCDC core |
| `inv` | GFL/GFM、PQ/下垂、切换、虚拟阻抗、VSM、谐波/负序/零序、电压环、SRF/DDSRF/DSOGI PLL 和 Vienna 整流器头 |
| `mppt` | INC 与 P&O 两种 MPPT 算法 |
| `sinv` | 单相逆变 core、外环、保护、重复控制、参考生成、SMS-PQ、SPFC 和两种 SOGI PLL |

应包含具体模块头，并由源管理器选择注册源码。`ctl/component/digital_power.h` 是历史聚合头，目前仍有 11 个 include 指向已删除的 `single_phase`、`three_phase` 等旧布局，不能作为可编译公共入口。

`inv/tests/host_sim` 只验证一组 GFM 主机用例，`sinv/tests/sinv_qr_contract_test.c` 只验证单相 QR 契约；它们不代表所有模块或硬件均已验证。各 suite 的 `BUILD_LEVEL`、信号方向和验证范围以对应 suite README 为准。
