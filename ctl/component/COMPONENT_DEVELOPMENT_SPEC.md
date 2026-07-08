# CTL Component 模块开发规范与现状梳理

本文档基于 `ctl/component` 现有实现、`ctl/suite` 应用工程、`ctl/math_block` 数学类型，以及 `tools/facilities_generator/src_mgr/gmp_framework_dic.json` 中 `ctl.modules` 下的 `component|...` 注册项整理。所有头文件引用路径均以 GMP 工程根目录为根，例如 `#include <ctl/component/intrinsic/continuous/continuous_pid.h>`。

## 1. 模块定位

`ctl/component` 是控制算法组件库，定位在 `ctl/math_block` 之上、`ctl/suite` 之下：

- `ctl/math_block` 定义 `ctrl_gt`、`parameter_gt`、向量、矩阵、坐标变换和常量，组件模块必须优先使用这些基础类型。
- `ctl/component` 提供可复用控制模块，包括接口抽象、基础控制器、电机控制、数字电源、硬件预设和 DSA。
- `ctl/suite` 是组件的工程化集成和验证入口，完整应用应放在 suite 中，不应反向污染 component 的平台无关算法。
- `gmp_framework_dic.json` 是工程生成器和模块依赖关系的来源，新模块必须在其中注册。

## 2. 模块表

当前模块分层和后续维护表单独整理在 `ctl/component/COMPONENT_MODULE_CATALOG.md`。本规范只约定开发规则；模块表用于持续记录模块状态、注册名、文件位置、验证 suite 和待整改项。

## 3. 类型系统与实时计算约定

GMP 是面向控制的仓库。组件开发必须区分两个关键数值类型：`ctrl_gt` 和 `parameter_gt`。

### 3.1 `ctrl_gt`

`ctrl_gt` 是控制专用类型，用于实时控制路径中的状态、输入、输出、中间量和控制律计算。它可能是 `float`，也可能是定点数或平台优化类型，因此不能假设它一定支持普通 C 浮点语义。

要求：

- `ctl_step_*` 函数中参与控制律计算的变量必须使用 `ctrl_gt`。
- `ctl_step_*` 中的乘法、除法、饱和、绝对值、三角函数、平方根、非线性函数等必须使用 `ctrl_gt` 配套 API。
- 控制计算默认工作在标幺值系统中，除非函数文档明确说明单位。
- 角度若使用 `ctrl_gt` 标幺角，默认 `1.0 == 2*pi`，文档必须写清。
- 不允许在 `ctl_step_*` 中临时转换为 `float/double` 做控制计算，除非该模块明确只支持浮点后端且在模块名、文档、注册描述中标明。

`ctrl_gt` 的基础约定来自 `ctl/math_block/ctrl_gt`。其中 `float_macros.h` 是最易读的参考实现，定义了 `ctrl_gt` 支持的典型计算函数；同目录下其他后端文件说明了定点、CMSIS、IQmath、CLA 等情况下同一套 API 的实现方式。

典型写法：

```c
ctrl_gt err = ref - fdb;
ctrl_gt p = ctl_mul(obj->kp, err);
obj->integrator = ctl_sat(obj->integrator + ctl_mul(obj->ki, err), obj->upper, obj->lower);
obj->output = ctl_sat(p + obj->integrator, obj->upper, obj->lower);
```

不推荐写法：

```c
float p = obj->kp * err;
obj->output = (ctrl_gt)p;
```

### 3.2 `parameter_gt`

`parameter_gt` 是参数计算类型，至少为 `float` 能力，允许直接使用加减乘除和标准浮点计算。它适合用于初始化、整定、参数推导、物理单位换算和 consultant 类模块。

要求：

- `ctl_init_*`、`ctl_setup_*`、`ctl_auto_tuning_*`、consultant 模块中可使用 `parameter_gt` 做物理参数计算。
- `parameter_gt` 计算结果进入实时控制对象前，应转换或归一化为 `ctrl_gt`。
- 配置参数可以保留为 `parameter_gt`，但只要在 `ctl_step_*` 中参与实时控制律，就必须预先转换为 `ctrl_gt` 参数。
- `parameter_gt` 可以使用普通 `+ - * /`，但不得让 `ctl_step_*` 依赖大量浮点除法、标准库三角函数或动态计算整定参数。

推荐结构：

```c
typedef struct _example_init_t
{
    parameter_gt fs;
    parameter_gt bandwidth;
    parameter_gt upper_limit_pu;
    parameter_gt lower_limit_pu;
} example_init_t;

typedef struct _example_t
{
    ctrl_gt kp;
    ctrl_gt ki;
    ctrl_gt upper;
    ctrl_gt lower;
    ctrl_gt output;
} example_t;
```

### 3.3 标幺值约定

组件库默认采用 p.u. 标幺值系统：

- `ctl_step_*` 输入和输出原则上使用标幺值。
- 物理量到标幺值的换算在 interface、suite 或 init/setup 阶段完成。
- 模块内部不要混合物理单位和标幺值；如必须混合，应在字段名和注释中明确后缀，例如 `_pu`、`_hz`、`_radps`。
- 限幅、阈值、保护值应明确是 p.u. 还是物理单位。
- suite 工程负责将 ADC/PWM/传感器读数转换到组件期望的标幺域。

## 4. 编程风格

### 4.1 文件组织

一个普通组件应采用如下布局：

```text
ctl/component/<domain>/<category>/
├── <module>.h
└── src/
    └── ctl_<domain_prefix>_<module>.c
```

规则：

- 头文件放在模块目录下，非内联或较大函数放在 `src` 下的 `.c` 文件。
- 目录、头文件、源文件、注册名应一一对应；注册名使用 `component|...|module`，路径使用根目录相对路径。
- 聚合头文件位于上级目录，例如 `ctl/component/intrinsic.h`、`ctl/component/motor_control.h`、`ctl/component/digital_power.h`。
- suite 级完整控制器不应新增到 `ctl/component`；如果是完整工程，应放到 `ctl/suite`。

### 4.2 Include 规则

组件内引用头文件使用工程根目录路径：

```c
#include <gmp_core.h>
#include <ctl/math_block/gmp_math.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>
```

要求：

- 不使用相对路径 `../`。
- 跨模块依赖必须在注册表 `depends_on` 中声明。
- 控制数学优先包含 `ctl/math_block` 中已有类型和函数，不在 component 中重复定义数学基础设施。
- 平台、芯片、外设寄存器相关头文件不应出现在通用组件头文件中；这类适配应放在 `ctl/suite` 或 CSP/平台层。

### 4.3 类型与数据结构

现有风格以 C 结构体加 `ctl_` 前缀函数为主：

```c
typedef struct _example_controller_t
{
    ctrl_gt kp;
    ctrl_gt ki;
    ctrl_gt output;
} example_controller_t;
```

规则：

- 实时控制算法数值使用 `ctrl_gt`，参数整定和初始化计算使用 `parameter_gt`，布尔/状态/计数使用项目已有整数类型或标准整数类型。
- 向量、坐标变换使用 `ctl_vector2_t`、`ctl_vector3_t` 等 math_block 类型。
- 模块对象命名使用 `<module>_t`，初始化参数可使用 `<module>_init_t` 或 `<module>_param_t`。
- 结构体字段按“配置参数、运行状态、输出量、端口/回调”顺序组织。

### 4.4 API 命名

组件 API 统一使用：

```text
ctl_<action>_<module>[_via_<method>]()
```

常用动作：

| 动作 | 语义 |
| --- | --- |
| `ctl_init_*` | 初始化参数和状态，启动时调用一次 |
| `ctl_setup_*` | 配置可变参数 |
| `ctl_clear_*` | 清空运行状态但保留配置参数 |
| `ctl_step_*` | 执行一个控制周期，应可在 ISR 中调用 |
| `ctl_set_*` | 写入参考值、限幅、参数或端口 |
| `ctl_get_*` | 读取输出、状态或参数 |
| `ctl_attach_*` | 绑定接口、端口或回调 |
| `ctl_is_*` | 查询状态标志 |

要求：

- 快速周期函数尽量为 `GMP_STATIC_INLINE`，复杂初始化或较大函数放入 `.c` 并用 `GMP_NOINLINE` 或普通外部函数。
- `step` 函数不得做动态分配、阻塞等待、打印或硬件寄存器访问。
- `init` 函数必须使对象进入确定状态，不能依赖未初始化字段。

### 4.5 宏与保护

头文件应包含 include guard：

```c
#ifndef _CTL_EXAMPLE_CONTROLLER_H_
#define _CTL_EXAMPLE_CONTROLLER_H_

...

#endif // _CTL_EXAMPLE_CONTROLLER_H_
```

要求：

- 宏名与文件路径保持可读对应关系。
- 公共常量优先放入 `ctl/math_block/const` 或模块专属参数结构，不滥用全局宏。
- 如需条件编译，应以功能/平台抽象宏为条件，避免直接绑定具体工程。

### 4.6 错误处理与实时性

- 对外 API 应检查关键指针，使用项目已有断言宏，例如 `GMP_ASSERT_PTR`。
- `step` 路径应保持确定执行时间，并只使用 `ctrl_gt` 支持的实时计算 API。
- 组件不负责调度、通信、日志和命令行交互，这些属于 suite 或 core。
- 保护类模块只输出状态、标志或限幅结果，不直接关闭硬件。

### 4.7 Doxygen 注释规范

头文件和源文件中的注释应严格按照 Doxygen 风格完成。公开 API 应使用 Doxygen 注释：

```c
/**
 * @brief Execute one step of example controller.
 * @param[in,out] obj Controller object.
 * @param[in] ref Reference input in p.u.
 * @param[in] fdb Feedback input in p.u.
 * @return Controller output in p.u.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_example_controller(example_controller_t* obj, ctrl_gt ref, ctrl_gt fdb);
```

要求：

- 文件头使用 `@file`、`@brief`，必要时补 `@details`。
- 结构体、枚举、宏、公开函数都必须有 Doxygen 注释。
- 公开字段使用行尾 `/**< ... */` 或字段前 Doxygen 注释说明单位和含义。
- 函数必须包含 `@brief`、完整 `@param[in]`/`@param[out]`/`@param[in,out]` 和 `@return`。
- 说明输入输出单位，尤其是 p.u.、rad/s、Hz、A、V、机械角/电角。
- 说明调用频率和状态机约束。
- 如果依赖 `ctrl_gt` 标幺角度，必须写明 `1.0 == 2*pi`。
- 不使用普通大段注释代替 API 文档；实现内部复杂算法可用简短普通注释解释步骤。

文件头示例：

```c
/**
 * @file example_controller.h
 * @brief Example controller component.
 * @details This module runs in per-unit control domain. All ctl_step_* inputs
 * and outputs are ctrl_gt values.
 */
```

结构体示例：

```c
/**
 * @brief Example controller runtime object.
 */
typedef struct _example_t
{
    ctrl_gt kp;      /**< Proportional gain in p.u. domain. */
    ctrl_gt output;  /**< Last controller output in p.u. */
} example_t;
```

## 5. 注册规范

每个可独立选择的模块必须在 `gmp_framework_dic.json` 中注册：

```json
"component|intrinsic|continuous|pid": {
  "type": "module",
  "description": "Continuous PID controller",
  "src_patterns": [
    "${GMP_PRO_LOCATION}/ctl/component/intrinsic/continuous/src/ctl_continuous_pid.c"
  ],
  "inc_patterns": [
    "${GMP_PRO_LOCATION}/ctl/component/intrinsic/continuous/continuous_pid.h"
  ],
  "depends_on": [
    "ctl|math_block|..."
  ],
  "inc_dirs": [],
  "help_docs": [],
  "tasks": [
    {"id": "sys_compile", "summary": "Compilation Passed", "completed": false, "date": "", "reply": ""},
    {"id": "sys_sim", "summary": "Simulation / Principle Verified", "completed": false, "date": "", "reply": ""},
    {"id": "sys_hw", "summary": "Physical Hardware Verified", "completed": false, "date": "", "reply": ""}
  ]
}
```

规则：

- `src_patterns` 只登记需要编译的 `.c` 文件；纯头文件模块可为空，但必须有 `inc_patterns`。
- `inc_patterns` 登记模块公共头文件，不用模糊覆盖整个目录，除非该目录本身就是预设集合。
- `depends_on` 必须覆盖直接 include 的上游模块。
- `_internal` 仅用于分类节点，不作为可交付算法模块。
- `tasks` 是实现成熟度的唯一可机器读取标记：新增模块默认三项 false，通过 suite 或单元测试验证后再置 true。

## 6. Suite 验证要求

组件完成不等于工程可用。推荐验证层次：

1. `sys_compile`: 模块可被独立包含和编译，注册的 `src_patterns/inc_patterns` 均有效。
2. `sys_sim`: 在 PC、Simulink 或 suite 的 `simulate` 工程中完成原理验证。
3. `sys_hw`: 在目标硬件工程中完成实物验证。

关联 suite：

| 组件方向 | 主要验证工程 |
| --- | --- |
| PMSM/FOC/observer/motion | `ctl/suite/mcs_pmsm`、`mcs_pmsm_nt`、`mcs_pmsm_id` |
| ACM 控制 | `ctl/suite/mcs_acm` |
| FSBB/DCDC | `ctl/suite/dps_fsbb` |
| GFL inverter | `ctl/suite/pgs_inv_GFL_inverter` |
| 单相逆变 RC | `ctl/suite/pgs_sinv_rc` |

## 7. 当前不完整或不规范项

### 7.1 明显未正确实现或空壳模块

| 注册项/目录 | 问题 |
| --- | --- |
| `component|kinematics` / `ctl/component/kinematics` | 顶层已注册且目录存在，但当前没有可见头文件、源文件和任务状态，属于未实现模块。 |
| `component|motor_control|acm_ctrl` | 注册为空，无 `src_patterns`、`inc_patterns` 和任务状态；若是分类节点，应改为 `_internal`，若是模块则需补实现。 |
| `component|motor_control|param_est` | 注册为空；目录中仅见 `param_est/pmsm_rs_est_mras.h` 等文件，注册关系未闭合。 |
| `component|motor_control|pmsm_ctrl` | 注册为空；实际 PMSM suite 控制器位于 `motor_control/suite_pmsm`，需要明确它是 suite 级组件还是废弃聚合名。 |
| `component|motor_control|motor_id|_internal` | 注册使用 `motor_id`，实际目录为 `pmsm_offline_id`；命名层级不一致。 |

### 7.2 命名拼写或路径不一致

| 注册项 | 实际文件/目录 | 问题 |
| --- | --- | --- |
| `component|digital_power|sinv|sinv_proctect` | `sinv_protect.h`、`ctl_dp_sinv_protect.c` | `proctect` 拼写错误，应统一为 `protect`。 |
| `component|intrinsic|advance|back_steppping` | `back_stepping.h` | `steppping` 拼写错误，应统一为 `back_stepping`。 |
| `component|intrinsic|advance|paired_lud1d` | `paired_lut1d.h` | `lud` 拼写错误，应统一为 `lut`。 |
| `component|motor_control|mech_loop|...` | `motor_control/mechanical_loop` | 注册名 `mech_loop` 与目录 `mechanical_loop` 不一致。 |
| `component|hardware_preset|grid_filter` | `hardware_preset/grid_LC_filter` | 注册名未体现实际目录。 |
| `component|hardware_preset|sensor` | `hardware_preset/current_sensor` | 注册名过泛，实际只覆盖 current sensor。 |

### 7.3 聚合头和注册覆盖需复核

- `ctl/component` 顶层存在 `digital_power.h`、`dsa.h`、`intrinsic.h`、`motor_control.h`，但没有同级 `interface.h` 和 `hardware_preset.h`。
- `ctl/component/interface` 已作为注册模块族存在，应考虑增加聚合头，或明确只能按子模块头文件包含。
- `hardware_preset` 以预设集合为主，很多条目是纯头文件；需要统一说明其不需要 `.c` 和 `sys_compile` 任务的判定方式。
- `dsa/sine_analyzer.h` 在目录中存在，但注册表当前主要显示 `dsa_scope`、`dsa_trigger`，需要确认是否漏注册。

### 7.4 任务状态不完整

从注册表片段看，多个 component 模块的 `sys_compile/sys_sim/sys_hw` 仍为 false，尤其是 `dsa`、`digital_power/sinv` 后补模块，以及不少保护、观测器和高级控制模块。建议按下列优先级补齐：

1. 先补 `sys_compile`: 确保注册的 include/source 在最小工程中通过编译。
2. 再补 `sys_sim`: 与现有 suite 或最小仿真用例关联。
3. 最后补 `sys_hw`: 只有经过真实硬件工程验证后再置 true。

## 8. 新模块开发流程

1. 选择目录：放到现有模块族中；如果是完整控制器应用，放到 `ctl/suite`。
2. 定义头文件：使用根路径 include、include guard、`ctrl_gt` 和 Doxygen 注释。
3. 实现源文件：只放非内联或较复杂函数，文件名使用 `ctl_<domain>_<module>.c` 风格。
4. 更新聚合头：如果希望用户通过模块族入口包含，应同步更新对应 `*.h`。
5. 更新注册表：补 `src_patterns`、`inc_patterns`、`depends_on`、`tasks`。
6. 建立验证：至少提供最小编译验证；若模块用于电机/电源闭环，应在 suite 中给出仿真或工程用例。
7. 更新文档：在模块 readme 或本规范中补充接口、单位和调用频率。

## 9. 推荐的最小模板

```c
#ifndef _CTL_COMPONENT_EXAMPLE_H_
#define _CTL_COMPONENT_EXAMPLE_H_

#include <gmp_core.h>
#include <ctl/math_block/gmp_math.h>

/**
 * @brief Example controller runtime object.
 */
typedef struct _example_t
{
    ctrl_gt gain;    /**< Runtime gain in p.u. domain. */
    ctrl_gt output;  /**< Last output in p.u. */
} example_t;

/**
 * @brief Initialize example controller.
 * @param[out] obj Controller object.
 * @param[in] gain Runtime gain in p.u. domain.
 * @return None.
 */
GMP_STATIC_INLINE
void ctl_init_example(example_t* obj, ctrl_gt gain)
{
    GMP_ASSERT_PTR(obj);
    obj->gain = gain;
    obj->output = 0;
}

/**
 * @brief Execute one control step.
 * @param[in,out] obj Controller object.
 * @param[in] input Input value in p.u.
 * @return Output value in p.u.
 */
GMP_STATIC_INLINE
ctrl_gt ctl_step_example(example_t* obj, ctrl_gt input)
{
    GMP_ASSERT_PTR(obj);
    obj->output = ctl_mul(obj->gain, input);
    return obj->output;
}

/**
 * @brief Clear runtime states.
 * @param[in,out] obj Controller object.
 * @return None.
 */
GMP_STATIC_INLINE
void ctl_clear_example(example_t* obj)
{
    GMP_ASSERT_PTR(obj);
    obj->output = 0;
}

#endif // _CTL_COMPONENT_EXAMPLE_H_
```

## 10. 后续整理建议

- 对注册表执行一次自动审计，输出“注册项、源文件、头文件、聚合头、任务状态”的 CSV。
- 将拼写错误注册项迁移到规范名称，必要时保留兼容别名。
- 把 `kinematics`、`acm_ctrl`、`param_est`、`pmsm_ctrl` 这类空壳项标记为 `_internal`、补实现或删除。
- 为 `interface` 和 `hardware_preset` 补聚合头或在文档中明确不提供聚合入口。
- 将每个 suite 与其覆盖的 component 模块反向登记，避免“有工程但注册任务仍为 false”。
