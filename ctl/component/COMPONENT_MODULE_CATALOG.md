# CTL Component 模块表

本文档记录 `ctl/component` 当前模块分层。后续开发应持续完善本表，包括注册名、文件位置、模块状态、验证 suite、任务状态和整改备注。

状态建议：

- `done`: 已实现并至少通过编译验证。
- `partial`: 有代码但注册、聚合头、验证或任务状态不完整。
- `stub`: 注册或目录存在，但基本没有可用实现。
- `preset`: 硬件参数预设，通常为纯头文件集合。
- `review`: 需要进一步确认归属或命名。

## 1. 顶层模块族

| 模块族 | 目录 | 注册前缀 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| interface | `ctl/component/interface` | `component|interface` | partial | 标准 ADC/DAC/PWM 接口和调制器。建议补顶层聚合头或明确只按子模块包含。 |
| intrinsic | `ctl/component/intrinsic` | `component|intrinsic` | partial | 基础控制算法组件，分 basic/continuous/discrete/advance/protection。 |
| motor_control | `ctl/component/motor_control` | `component|motor_control` | partial | 电机控制算法组件，含 current loop、observer、motion、consultant 等。 |
| digital_power | `ctl/component/digital_power` | `component|digital_power` | partial | 数字电源和逆变器相关组件。 |
| hardware_preset | `ctl/component/hardware_preset` | `component|hardware_preset` | preset | 纯参数预设集合，注册和目录命名需统一。 |
| dsa | `ctl/component/dsa` | `component|dsa` | partial | DSA scope/trigger 等，`sine_analyzer.h` 是否注册需复核。 |
| kinematics | `ctl/component/kinematics` | `component|kinematics` | stub | 目录和注册存在，但当前基本为空。 |

## 2. interface

| 子模块 | 目录/文件 | 注册项 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| internal | `interface/src/ctl_component_interface.c` 等 | `component|interface|_internal` | partial | 标准接口基础实现。 |
| adc | `adc_channel.h`, `adc_ptr_channel.h` | `component|interface|adc` | partial | ADC 通道抽象。 |
| dac | `dac_channel.h` | `component|interface|dac` | partial | DAC 通道抽象。 |
| pwm | `pwm_channel.h` | `component|interface|pwm` | partial | PWM 通道抽象。 |
| H bridge | `hpwm_modulator.h`, `src/ctl_if_pwm_h.c` | `component|interface|modulator|H_bridge` | partial | 半桥/H bridge 调制。 |
| npc | `npc_modulator.h`, `src/ctl_if_pwm_npc.c` | `component|interface|modulator|npc` | partial | NPC 调制。 |
| spwm | `spwm_modulator.h`, `spwm3d_modulator.h`, `src/ctl_if_pwm_spwm.c` | `component|interface|modulator|spwm` | partial | SPWM/三维 SPWM 调制。 |

## 3. intrinsic

### 3.1 basic

| 子模块 | 目录/文件 | 注册项 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| divider | `basic/divider.h` | `component|intrinsic|_internal` 覆盖 | review | 建议拆成独立注册项。 |
| hysteresis | `basic/hysteresis_controller.h` | `component|intrinsic|_internal` 覆盖 | review | 建议拆成独立注册项。 |
| saturation | `basic/saturation.h` | `component|intrinsic|_internal` 覆盖 | review | 建议拆成独立注册项。 |
| slope limiter | `basic/slope_limiter.h` | `component|intrinsic|_internal` 覆盖 | review | 建议拆成独立注册项。 |
| state sequencer | `basic/state_sequencer.h` | `component|intrinsic|_internal` 覆盖 | review | 建议拆成独立注册项。 |

### 3.2 continuous

| 子模块 | 目录/文件 | 注册项 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| PI | `continuous/continuous_pi.h` | `component|intrinsic|continuous|pi` | partial | 连续 PI。 |
| PID | `continuous/continuous_pid.h` | `component|intrinsic|continuous|pid` | partial | 连续 PID。 |
| PID anti-windup | `continuous/continuous_pid_aw.h` | `component|intrinsic|continuous|pid_aw` | partial | 带抗饱和 PID。 |
| LADRC1 | `continuous/ladrc1.h` | `component|intrinsic|continuous|ladrc1` | partial | 一阶 LADRC。 |
| LADRC2 | `continuous/ladrc2.h` | `component|intrinsic|continuous|ladrc2` | partial | 二阶 LADRC。 |
| SOGI | `continuous/sogi.h` | `component|intrinsic|continuous|sogi` | partial | 连续 SOGI。 |
| track PID | `continuous/track_pid.h` | `component|intrinsic|continuous|track_pid` | partial | 跟踪 PID。 |

### 3.3 discrete

| 子模块 | 目录/文件 | 注册项 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| biquad filter | `discrete/biquad_filter.h` | `component|intrinsic|discrete|biquad_filter` | partial | 二阶节滤波。 |
| direct form | `discrete/direct_form.h` | `component|intrinsic|discrete|direct_form` | partial | 直接型离散结构。 |
| discrete filter | `discrete/discrete_filter.h` | `component|intrinsic|discrete|discrete_filter` | partial | 离散滤波器。 |
| discrete PID | `discrete/discrete_pid.h` | `component|intrinsic|discrete|pid` | partial | 离散 PID。 |
| discrete SOGI | `discrete/discrete_sogi.h` | `component|intrinsic|discrete|sogi` | partial | 离散 SOGI。 |
| FIR | `discrete/fir_filter.h` | `component|intrinsic|discrete|fir_filter` | partial | FIR 滤波器。 |
| lead-lag | `discrete/lead_lag.h` | `component|intrinsic|discrete|lead_lag` | partial | 超前滞后环节。 |
| pole-zero | `discrete/pole_zero.h` | `component|intrinsic|discrete|pole_zero` | partial | 零极点补偿器。 |
| PR | `discrete/proportional_resonant.h` | `component|intrinsic|discrete|pr` | partial | 比例谐振控制器。 |
| signal generator | `discrete/signal_generator.h` | `component|intrinsic|discrete|signal_generator` | partial | 离散信号发生器。 |
| track PID | `discrete/track_discrete_pid.h` | `component|intrinsic|discrete|track_pid` | partial | 离散跟踪 PID。 |

### 3.4 advance

| 子模块 | 目录/文件 | 注册项 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| back stepping | `advance/back_stepping.h` | `component|intrinsic|advance|back_steppping` | partial | 注册拼写应修正。 |
| FLC | `advance/flc.h` | `component|intrinsic|advance|flc` | partial | 模糊控制。 |
| fuzzy PID | `advance/fuzzy_pid.h` | `component|intrinsic|advance|fuzzy_pid` | partial | 模糊 PID。 |
| ILC | `advance/ilc.h` | `component|intrinsic|advance|ilc` | partial | 迭代学习控制。 |
| IMC | `advance/imc.h` | `component|intrinsic|advance|imc` | partial | 内模控制。 |
| LMS | `advance/lms_filter.h` | `component|intrinsic|advance|lms_filter` | partial | LMS 自适应滤波。 |
| MRAC | `advance/mrac.h` | `component|intrinsic|advance|mrac` | partial | 模型参考自适应控制。 |
| paired LUT1D | `advance/paired_lut1d.h` | `component|intrinsic|advance|paired_lud1d` | partial | 注册拼写应修正。 |
| RC | `advance/repetitive_controller.h` | `component|intrinsic|advance|rc` | partial | 重复控制器。 |
| sinc interpolator | `advance/sinc_interpolator.h` | `component|intrinsic|advance|sinc_interpolator` | partial | Sinc 插值。 |
| SMC | `advance/smc.h` | `component|intrinsic|advance|smc` | partial | 滑模控制。 |
| surf search | `advance/surf_search.h` | `component|intrinsic|advance|surf_search` | partial | 表面搜索/查表。 |

### 3.5 protection

| 子模块 | 目录/文件 | 注册项 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| basic protection | `protection/protection.h`, `protection_slot.h` | `component|intrinsic|protection|basic` | partial | 通用保护。 |
| ITOC | `protection/itoc_protection.h` | `component|intrinsic|protection|itoc` | partial | 反时限过流保护。 |
| PT sensor | `protection/pt100x.h` | `component|intrinsic|protection|pt_sensor` | partial | 温度传感器。 |
| sag/swell | `protection/sag_swell.h` | `component|intrinsic|protection|sag_swell` | partial | 电压暂降暂升。 |

## 4. motor_control

| 子模块 | 目录/文件 | 注册项 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| basic decouple | `basic/decouple.h` | `component|motor_control|basic|decouple` | partial | 电机解耦。 |
| basic protection | `basic/mtr_protection.h` | `component|motor_control|basic|protection` | partial | 电机保护。 |
| VF generator | `basic/vf_generator.h` | `component|motor_control|basic|vf_generator` | partial | V/F 发生器。 |
| voltage calculator | `basic/voltage_calculator.h` | `component|motor_control|basic|voltage_calculator` | partial | 电压计算。 |
| consultant | `consultant/*.h` | `component|motor_control|consultant|...` | partial | PMSM/ACIM/PU/inverter/mechanical/nameplate 参数助手。 |
| current loop | `current_loop/*.h` | `component|motor_control|current_loop|...` | partial | FOC、IMFOC、DTC-SVM、PMSM DBPTC。 |
| distributor | `distributor/*.h` | `component|motor_control|distributor|...` | partial | SPM/IPM/LUT 弱磁分配。 |
| mechanical loop | `mechanical_loop/*.h` | `component|motor_control|mech_loop|...` | partial | 注册名与目录不一致。 |
| motion | `motion/*.h` | `component|motor_control|motion|...` | partial | 梯形、S 曲线、SOGI 轨迹。 |
| observer | `observer/*.h` | `component|motor_control|observer|...` | partial | PMSM/ACIM/BLDC 观测器和 ATO PLL。 |
| pmsm offline id | `pmsm_offline_id/pmsm_offline_id_sm.h` | `component|motor_control|motor_id|pmsm` | partial | 注册名与目录不一致。 |
| param_est | `param_est/pmsm_rs_est_mras.h` | `component|motor_control|param_est` | stub | 注册为空，需补模块定义。 |
| suite_pmsm/acim | `suite_pmsm/*`, `suite_acim/*` | `component|motor_control|pmsm_ctrl`, `acm_ctrl` | review | 更像 suite 级控制器，需明确归属。 |

## 5. digital_power

| 子模块 | 目录/文件 | 注册项 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| basic | `basic/*.h`, `basic/src/ctl_dp_basic.c` | `component|digital_power|_internal` | partial | 保护策略、SIL 接口、虚拟阻抗等基础设施。 |
| dcdc | `dcdc/*.h`, `dcdc/src/*.c` | `component|digital_power|dcdc` | partial | Buck、Boost、FSBB、DCDC core。 |
| inv GFL/PQ/PLL | `inv/*.h`, `inv/src/*.c` | `component|digital_power|inv|...` | partial | 三相逆变器、PLL、PQ、HCM、负序控制。 |
| mppt | `mppt/INC_algorithm.h`, `PnO_algorithm.h` | `component|digital_power|mppt|inc`, `po` | partial | INC 和 P&O MPPT。 |
| sinv | `sinv/*.h`, `sinv/src/*.c` | `component|digital_power|sinv|...` | partial | 单相逆变、RC、保护、参考生成、SPFC、SPLL、SMS PQ。`sinv_proctect` 注册拼写需修正。 |

## 6. hardware_preset

| 子模块 | 目录 | 注册项 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| acm motor | `hardware_preset/acm_motor` | `component|hardware_preset|acm_motor` | preset | 异步电机参数。 |
| pmsm motor | `hardware_preset/pmsm_motor` | `component|hardware_preset|pmsm_motor` | preset | PMSM 参数。 |
| pmasrm motor | `hardware_preset/pmasrm_motor` | 未清晰匹配 | review | 需确认是否补注册。 |
| current sensor | `hardware_preset/current_sensor` | `component|hardware_preset|sensor` | preset | 注册名建议改为 current_sensor。 |
| grid LC filter | `hardware_preset/grid_LC_filter` | `component|hardware_preset|grid_filter` | preset | 注册名建议对齐目录。 |
| inverter 3ph | `hardware_preset/inverter_3ph` | `component|hardware_preset|inverter_3ph` | preset | 三相逆变器预设。 |
| inverter HB | `hardware_preset/inverter_HB` | `component|hardware_preset|inverter_HB` | preset | 半桥逆变器预设。 |

## 7. dsa

| 子模块 | 目录/文件 | 注册项 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| dsa scope | `dsa/dsa_scope.h`, `dsa/src/dsa_scope.c` | `component|dsa|dsa_scope` | partial | 示波/数据采集。 |
| dsa trigger | `dsa/dsa_trigger.h`, `dsa/src/dsa_trigger.c` | `component|dsa|dsa_trigger` | partial | 触发采集。 |
| sine analyzer | `dsa/sine_analyzer.h` | 未清晰匹配 | review | 需确认是否漏注册。 |
| TI DLOG | `dsa/ti_dlog/*` | 未清晰匹配 | review | 第三方/辅助文件，需明确注册策略。 |

## 8. 后续维护约定

每次新增或调整模块，应同步更新本表：

- 模块名和注册项。
- 头文件和源文件路径。
- 依赖模块和验证 suite。
- `sys_compile`、`sys_sim`、`sys_hw` 状态。
- 命名、目录、聚合头、文档待办项。
