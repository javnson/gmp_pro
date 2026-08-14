# SDPE v2 数据格式

本文档描述 SDPE v2 当前 CLI 支持的数据格式。

## 1. Schema

Schema 文件放在 `schemas/` 下。

必选字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `id` | string | schema 唯一 ID。 |
| `display_name` | string | 展示名称。 |

常用字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `description` | string | 说明。 |
| `category` | string | 分类。 |
| `tags` | array | 用于搜索和分类的标签。 |
| `output_subdir` | string | 生成头文件的子目录。 |
| `includes` | array | 当前类型生成头文件时需要额外包含的 GMP 头文件路径。 |
| `code_sections` | object | 生成头文件时注入的自定义代码段。 |
| `parameters` | array | 参数定义。 |
| `derived_macros` | array | 派生宏定义。 |
| `components` | object | 默认 Sub Components，格式与 Entity 的 `components` 一致。 |
| `exports` | object | 可被工程需求引用的逻辑导出量。 |

### Parameter

```json
{
  "name": "bias_v",
  "c_name": "BIAS_V",
  "unit": "V",
  "description": "Zero-current output bias.",
  "required": false,
  "default": 1.65,
  "value_format": "{}",
  "numeric_domain": "parameter"
}
```

`value_format` 使用 Python `str.format`，其中 `{}` 会被替换为 JSON 值。

`name` 面向用户显示，可以使用更自然的名称；`c_name` / Macro Name 是生成 C 宏和兼容旧 Entity 参数键的稳定标识。Entity 参数既可以使用 `name`，也可以使用 `c_name` 的小写形式，例如 `RANGE_A` 对应 `range_a`。

常用格式：

| 格式 | 说明 |
| --- | --- |
| `{}` | 原始值套入格式。 |
| `raw` | 完全原样输出，适合 C 表达式或已写好的宏。 |
| `({}f)` | 旧版 float 字面量格式；新配置不应使用。 |
| `({}U)` | unsigned 字面量。 |
| `"{}"` | C 字符串字面量。 |

数值参数还应通过 `numeric_domain` 明确目标域：

| `numeric_domain` | C 输出 | 用途 |
| --- | --- | --- |
| `raw` | 不包装 | 标识符、整数、外设宏或完整 C 表达式 |
| `real` | 原生无后缀表达式 | 仅在后续表达式中继续保持源数值语义 |
| `parameter` | `real2param(value)` | 物理量、配置值和初始化参数，推荐默认选择 |
| `ctrl` | `real2ctrl(value)` | 明确需要直接进入实时域的常量 |

不要给实数字面量添加 `f` 后缀；否则在 `parameter_gt=double` 时，精度会在进入转换宏之前丢失。工程 requirement 的 binding 也可使用 `{"real": ...}`、`{"parameter": ...}` 或 `{"ctrl": ...}`。UI 中对应显示为 `real_gt`、`parameter_gt` 和 `ctrl_gt`，Binding Value 始终按 real 字面量输入，再由生成器转换到目标域。生成的 MATLAB 初始化脚本提供同名四个转换函数句柄，并允许通过 `GMP_SDPE_*_TYPE` 和转换器句柄选择仿真数值类型。

`default` 可以是普通 JSON 值，也可以是绑定对象：

```json
{"literal": "(2048U)"}
{"macro": "EXISTING_C_MACRO"}
{"ref": "current_sensor.range"}
{"expr": "({current_sensor.internal_resistance_ohm} * 20.0f)"}
```

其中 `ref` 和 `expr` 以当前 entity 为根，引用 Sub Components 中的参数、derived macro 或 export。

### Derived Macro

```json
{
  "name": "SENSITIVITY_V_PER_A",
  "expr": "({PREFIX}_SENSITIVITY_MV_PER_A / 1000.0f)",
  "description": "Current sensor sensitivity converted to V/A.",
  "unit": "V/A"
}
```

`{PREFIX}` 会替换为 entity 的 `macro_prefix`。

### Default Sub Components

```json
"components": {
  "current_sensor": {
    "entity": "tmcs1133_b5a"
  },
  "voltage_sensor": {
    "entity": "lvfb_voltage_divider_150v"
  }
}
```

Template Definition 中的 Sub Components 表示实例化 Entity 时默认带出的子模块，不再表示必填槽位或类型约束。Entity 可以继承这些默认子模块，也可以在自己的 `components` 中用同名 slot 覆盖。

### Export

```json
{
  "macro": "{PREFIX}_SENSITIVITY_V_PER_A",
  "unit": "V/A",
  "description": "ADC scaling sensitivity."
}
```

工程绑定可以通过 `entity.export`、`entity.slot.export`、`entity.parameter` 或 `entity.slot.parameter` 引用。

## 2. Entity

Entity 文件放在 `entities/` 下。

```json
{
  "id": "tmcs1133_b5a",
  "schema": "current_sensor",
  "display_name": "TMCS1133 B5A Hall Current Sensor",
  "vendor": "Texas Instruments",
  "datasheet_url": "https://www.ti.com/lit/ds/symlink/tmcs1133.pdf",
  "document_url": "",
  "macro_prefix": "TMCS1133_B5A",
  "includes": [],
  "tags": ["current_sensor", "hall", "tmcs1133"],
  "parameters": {
    "chip": "TMCS1133B5A",
    "range_a": 10.3,
    "bias_v": 1.65,
    "sensitivity_mv_per_a": 150.0
  }
}
```

`includes` 也可以在 Entity 中声明，用于某个具体实例额外依赖的头文件。Schema 和 Entity 的 `includes` 会与 Sub Components 产生的 include 一起去重输出。

`code_sections` 用于在生成头文件中保留用户自定义代码。当前支持这些插入点：

| 字段 | 位置 |
| --- | --- |
| `after_includes` | include 列表之后，`extern "C"` 之前。 |
| `before_parameters` | 参数宏生成之前。 |
| `before_exports` | Logical exports 注释之前。 |
| `before_footer` | 文件结尾、include guard 关闭之前。 |

每个插入点可以是字符串，也可以是字符串数组。例如：

```json
"code_sections": {
  "before_footer": "#define TMCS1133_USE_EXTERNAL_CALIBRATION 1"
}
```

复合 entity 使用 `components`：

```json
"components": {
  "current_sensor": {"entity": "tmcs1133_b5a"}
}
```

引用完整 entity 时可以用 `overrides` 覆盖局部参数：

```json
"components": {
  "current_sensor": {
    "entity": "tmcs1133_b2a",
    "overrides": {
      "range_a": 25.0,
      "bias_v": 1.64
    }
  }
}
```

这表示当前元件仍然 include `tmcs1133_b2a.h`，但是会在父元件作用域下生成本地覆盖宏，例如：

```c
#define GMP_LVFB_HB_B_TUNED_CURRENT_SENSOR_RANGE_A (25.0f)
#define GMP_LVFB_HB_B_TUNED_CURRENT_SENSOR_SENSITIVITY_MV_PER_A TMCS1133_B2A_SENSITIVITY_MV_PER_A
```

因此用户可以把一个完整元件作为基础对象引入，只修改本工程、本板卡或当前装配中发生变化的参数。

Inline 子实体使用：

```json
"components": {
  "current_sensor": {
    "inline": {
      "id": "local_shunt",
      "schema": "current_sensor",
      "macro_prefix": "LOCAL_SHUNT",
      "parameters": {}
    }
  }
}
```

## 3. Project

Project 文件放在 `projects/` 下。

```json
{
  "id": "dps_fsbb_iris_node",
  "macro_prefix": "DPS_FSBB_IRIS",
  "display_name": "DPS FSBB on F280039C IRIS Node",
  "suite": "dps_fsbb",
  "output_header": "sdpe_dps_fsbb_iris_bindings.h",
  "common_requirements": [
    "../../../sdpe_general/sdpe_requirement.json",
    "%GMP_PRO_LOCATION%/extensions/site_common/sdpe_requirement.json"
  ],
  "hardware": [
    {"role": "mcu_board", "entity": "iris_f280039c_node"},
    {"role": "buck_half_bridge", "entity": "fsbb_inline_shunt_half_bridge"}
  ],
  "requirements": [
    {
      "role": "inductor_current_sensitivity",
      "macro": "CTRL_INDUCTOR_CURRENT_SENSITIVITY",
      "binding": {"export": "fsbb_inline_shunt_half_bridge.current_sensor.sensitivity"}
    }
  ],
  "peripheral_bindings": {
    "PHASE_BUCK_BASE": "IRIS_F280039C_EPWM1_BASE"
  },
  "global_macros": {
    "SDPE_ENABLE_DPS_FSBB_BINDINGS": "1"
  }
}
```

`macro_prefix` 是可选的 Project 元数据命名空间，可在 Project Requirement 的 Basic 页面编辑。设置后，生成器会输出
`DPS_FSBB_IRIS_SDPE_PROJECT_ID`、`DPS_FSBB_IRIS_SDPE_PROJECT_SUITE` 等元数据宏；未设置时继续输出旧的
`SDPE_PROJECT_*` 名称。这样同一个 C/C++ 工程可以同时包含多个 SDPE Project 头文件。

`common_requirements` 按顺序绑定零个或多个公共需求清单。每一项可以是相对于当前
Project JSON 的路径、绝对路径，或包含 `%VAR%`、`$VAR`、`${VAR}` 的环境变量路径。
该字段只保存在私有工程中；编辑器合并显示全部来源，但保存时仍分别写回原文件。
生成器只输出 Private 工程 `output_header` 指定的 C Header，并将所有 Common 内容直接
合并到该文件中；Private 内容在前，Common 内容作为 Weak fallback 在后。MATLAB Init
Script 仍按 Common 后 Private 的调用链生成，以兼容现有 Simulink 初始化流程。

合并视图中的 Common Requirement、Selection Macro 和 Option Macro 可以直接编辑，
保存时写回各自的 Common JSON。Requirements 右键菜单分别提供
`Create private requirement` 和 `Create Common requirement`。对 Common 项执行
`Cover with private requirement` 会创建同名的 Project Private 项；界面强制将
Project Private 项作为父节点，将所有同名 Common 项
作为子节点，并强制 Common 项启用 `weak`。生成 C 头文件时先输出 Project Private
定义，再 include Common 头文件；Common 中的同名定义使用 `#ifndef` 保护，因此可作为
安全的默认值。Project 与 Common 的 `enabled` 状态仍可分别设置。

`Ctrl+S` 和 Save All 会先提交当前活动的单元格编辑器，再分别保存所有受影响的
Private/Common JSON。Delete 默认删除所选行，焦点位于 Checkbox 等永久控件时行为一致；
进入文本编辑器后 Delete 只编辑文本。删除 Group 会删除完整子树，删除单独的 Private
Cover 项则会把 Common 子项移回所属 Group。

MATLAB Init Script 在完成赋值后会向控制台打印工程名称、ID、Suite、版本、硬件清单、
Common 清单、所有已使能变量的最终值以及被禁用的宏，便于启动仿真前核对配置。

Binding 支持三种形式：

```json
{"export": "entity.slot.export"}
{"macro": "EXISTING_C_MACRO"}
{"literal": "(3.3f)"}
```

`export` 路径也支持直接参数引用：

```json
{"export": "tmcs1133_b2a.range_a"}
{"export": "lvfb_half_bridge_phase_b_tuned.current_sensor.bias_v"}
```

这类路径会解析到对应作用域下的 C 宏。如果中间组件使用了 `overrides`，解析结果会指向父对象槽位作用域宏；否则解析到原始子元件宏。

Requirement 的 `role` 对应 GUI 中的 Name，是面向用户的说明名称，不是 C 标识符。
Name 可以包含空格，并统一采用每个单词首字母大写的显示风格；C 标识符仍由独立的
`macro` 字段保存。Project 合并视图中的 `Pri` 是由 Source 自动派生的只读状态，
不会写入 JSON。Tools 菜单中的重复修正工具可以在 Private 和多个 Common 之间为
每个重复宏选择唯一保留项。

## 4. 生成约定

- 生成头文件使用 Doxygen 风格文件头。
- 宏前缀来自 entity 的 `macro_prefix`。
- Schema 和 Entity 的 `includes` 会原样写入生成头文件。
- Schema 和 Entity 的 `code_sections` 会按插入点顺序写入生成头文件；同一插入点中 Schema 内容先于 Entity 内容。
- 非 inline 子实体生成独立头文件，并由父硬件 include。
- inline 子实体展开到父硬件头文件，不生成独立文件。
- 组件 `overrides` 使用父硬件的槽位作用域生成本地宏。
- include 默认以 GMP 根目录为逻辑根：`ctl/component/...`。
- 工程绑定头文件只做别名绑定，不在其中重新计算硬件参数。
