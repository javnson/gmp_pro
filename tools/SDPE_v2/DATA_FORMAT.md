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
  "accepted_schemas": ["current_sensor"],
  "required": true,
  "description": "Current measurement path."
}
```

### Export

```json
{
  "macro": "{PREFIX}_SENSITIVITY_V_PER_A",
  "unit": "V/A",
  "description": "ADC scaling sensitivity."
}
```

工程绑定可以通过 `entity.export` 或 `entity.slot.export` 引用。

## 2. Entity

Entity 文件放在 `entities/` 下。

```json
{
  "id": "tmcs1133_b5a",
  "schema": "current_sensor",
  "display_name": "TMCS1133 B5A Hall Current Sensor",
  "macro_prefix": "TMCS1133_B5A",
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

## 4. 生成约定

- 生成头文件使用 Doxygen 风格文件头。
- 宏前缀来自 entity 的 `macro_prefix`。
- 非 inline 子实体生成独立头文件，并由父硬件 include。
- inline 子实体展开到父硬件头文件，不生成独立文件。
- include 默认以 GMP 根目录为逻辑根：`ctl/component/...`。
- 工程绑定头文件只做别名绑定，不在其中重新计算硬件参数。
