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
| `parameters` | array | 参数定义。 |
| `derived_macros` | array | 派生宏定义。 |
| `component_slots` | object | 可组合子硬件槽位。 |
| `required_components` | array | 必须存在的子硬件槽位。 |
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
  "value_format": "({}f)"
}
```

`value_format` 使用 Python `str.format`，其中 `{}` 会被替换为 JSON 值。

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

### Component Slot

```json
{
  "accepted_categories": ["current_sensor"],
  "required": true,
  "description": "Current measurement path."
}
```

`accepted_schemas` 用于指定某几个明确模板；`accepted_categories` 用于指定一类兼容模板。比如工程只要求电流传感器时，可以接受 `current_sensor` category 下的霍尔传感器模板，也可以接受分流器传感器模板。

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
  "tags": ["current_sensor", "hall", "tmcs1133"],
  "parameters": {
    "chip": "TMCS1133B5A",
    "range_a": 10.3,
    "bias_v": 1.65,
    "sensitivity_mv_per_a": 150.0
  }
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
  "display_name": "DPS FSBB on F280039C IRIS Node",
  "suite": "dps_fsbb",
  "output_header": "sdpe_dps_fsbb_iris_bindings.h",
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

## 4. 生成约定

- 生成头文件使用 Doxygen 风格文件头。
- 宏前缀来自 entity 的 `macro_prefix`。
- 非 inline 子实体生成独立头文件，并由父硬件 include。
- inline 子实体展开到父硬件头文件，不生成独立文件。
- 组件 `overrides` 使用父硬件的槽位作用域生成本地宏。
- include 默认以 GMP 根目录为逻辑根：`ctl/component/...`。
- 工程绑定头文件只做别名绑定，不在其中重新计算硬件参数。
