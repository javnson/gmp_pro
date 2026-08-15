# CTL 电机控制组件

[English](readme.md) | **简体中文**

本目录提供可复用电机控制积木；完整控制应用位于 `ctl/suite`。

| 目录 | 当前实现 |
| --- | --- |
| `basic` | 解耦、编码器校准、电机保护、V/f 发生器、电压重构 |
| `consultant` | ACIM、PMSM、逆变器、机械、铭牌、标幺和单位计算 |
| `current_loop` | FOC、异步电机 FOC、DTC-SVM、PMSM 无差拍预测电流控制 |
| `distributor` | IPM、SPM 和 LUT 弱磁/电流分配 |
| `interface` | 编码器、编码器切换、通用电机接口、无感交接、启动激励和 SIL 结构 |
| `mechanical_loop` | 基础串级、LADRC 位置/速度、MIT 和 SMC 机械控制 |
| `motion` | 梯形、S 曲线、SOGI 和旋钮位置轨迹/控制 |
| `observer` | ACIM 磁链/位置/SMO、ATO PLL、BLDC Hall/ZCD、PMSM ESMO/磁链/HFI |
| `param_est` | PMSM 定子电阻 MRAS 头文件 |
| `pmsm_offline_id` | PMSM 离线辨识状态机及有感实现 |
| `suite_pmsm`、`suite_acim` | 仍位于 component 树内的完整控制器封装 |

`ctl/component/motor_control.h` 目前仍包含 42 个已删除布局或旧命名的 include，不能作为总入口。请包含 `current_loop/foc_core.h`、`observer/acim_fo.h` 等具体头，并通过 `gmp_framework_dic.json` 选择对应源码。

角度和单位属于具体模块契约。现代电机 suite 在明确说明处使用周标幺角（`1 pu = 2π rad`），但调用者仍须核对所选头文件和目标 `xplt`。`observer/tests/host_sim` 只覆盖一组观测器主机测试；硬件证据只对对应 suite、目标和 `BUILD_LEVEL` 有效。
