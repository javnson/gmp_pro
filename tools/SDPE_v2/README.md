# SDPE v2 软件定义电力电子工具

SDPE 是 Software Defined Power Electronics 的缩写。它的目标是把 `ctrl_settings.h` 中冗长、易错、强耦合的硬件配置拆成可复用的数据模型，并根据工程需求自动生成硬件预设头文件和工程绑定头文件。

v2 当前先实现核心 CLI 和数据规范，不依赖 GUI，也不依赖第三方 Python 包。后续可以在此基础上接入 PyQt、Electron 或 Web UI。

## 1. 核心概念

### 1.1 Schema

Schema 定义一类硬件的规范，位于 `examples/schemas`。

它描述：

- 这类硬件有哪些参数。
- 哪些参数必填，哪些有默认值。
- 参数如何生成 C 宏。
- 这类硬件可以包含哪些子硬件。
- 这类硬件向工程需求导出哪些逻辑量。

例如 `current_sensor` 可以要求芯片型号、量程、输出范围、偏置、灵敏度、内阻等信息。

### 1.2 Entity

Entity 是 schema 的具体实例，位于 `examples/entities`。

例如：

- `tmcs1133_b5a` 是一个具体电流传感器。
- `bsc093n15ns5` 是一个具体功率 MOSFET。
- `lvfb_half_bridge_phase_a` 是一个复合半桥模块，包含功率器件、电流传感器、电压传感器。

Entity 可以引用另一个独立 entity，也可以使用 inline 子实体。Inline 子实体不会生成独立头文件，适合“某个分流器阻值只属于当前板卡”的场景。

### 1.3 Project

Project 定义某个 suite 工程的需求绑定，位于 `examples/projects`。

它描述：

- 当前工程引入了哪些硬件。
- 工程需要哪些宏。
- 每个需求宏绑定到哪个硬件导出量。
- 特殊板级外设名如何映射到 SysConfig 或 BSP 宏。

例如 `dps_fsbb_iris_node.json` 会把 FSBB 工程中的 `CTRL_INDUCTOR_CURRENT_SENSITIVITY` 绑定到半桥中电流传感器的灵敏度，并把 `PHASE_BUCK_BASE` 绑定到 IRIS 板卡的 `IRIS_EPWM1_BASE`。

## 2. 目录结构

```text
tools/SDPE_v2/
├── sdpe.py
├── sdpe_v2/
│   ├── generator.py
│   ├── library.py
│   ├── model.py
│   └── util.py
├── examples/
│   ├── schemas/
│   ├── entities/
│   └── projects/
└── tests/
```

## 3. 快速开始

在 `tools/SDPE_v2` 目录下运行：

```powershell
python .\sdpe.py --library .\examples validate
```

生成某个硬件对象及其依赖头文件：

```powershell
python .\sdpe.py --library .\examples generate-entity lvfb_half_bridge_phase_a --out .\build
```

生成某个工程的硬件头文件和工程绑定头文件：

```powershell
python .\sdpe.py --library .\examples generate-project .\examples\projects\dps_fsbb_iris_node.json --out .\build
```

默认 include 路径以 GMP 根目录为根，例如：

```c
#include <ctl/component/hardware_preset/half_bridge/lvfb_half_bridge_phase_a.h>
```

如果需要在独立工程中使用不同 include 根，可以通过 `--include-prefix` 调整：

```powershell
python .\sdpe.py --library .\examples generate-project .\examples\projects\dps_fsbb_iris_node.json --out .\build --include-prefix ctl/component
```

## 4. 生成结果

生成目录示例：

```text
build/
├── hardware_preset/
│   ├── current_sensor/
│   ├── half_bridge/
│   ├── mcu_board/
│   ├── power_switch/
│   └── voltage_sensor/
└── project/
    └── sdpe_dps_fsbb_iris_bindings.h
```

硬件 entity 头文件包含：

- Doxygen 文件头。
- include guard。
- 子硬件 include。
- 当前硬件参数宏。
- 派生宏。
- inline 子硬件宏。
- 子硬件 alias 宏。
- logical exports 注释。

工程绑定头文件包含：

- 引入的硬件头文件。
- 工程元信息。
- suite 需求宏绑定。
- 板级外设映射。
- 工程全局宏。

## 5. 数据模型示例

### 5.1 独立电流传感器

```json
{
  "id": "tmcs1133_b5a",
  "schema": "current_sensor",
  "macro_prefix": "TMCS1133_B5A",
  "parameters": {
    "chip": "TMCS1133B5A",
    "range_a": 10.3,
    "bias_v": 1.65,
    "sensitivity_mv_per_a": 150.0,
    "internal_resistance_ohm": 0.0018
  }
}
```

### 5.2 复合半桥

```json
{
  "id": "lvfb_half_bridge_phase_a",
  "schema": "half_bridge",
  "macro_prefix": "GMP_LVFB_HB_A",
  "parameters": {
    "board_name": "GMP LVFB 150V 2ph V2 Phase A",
    "phase_count": 1,
    "deadtime_ns": 523.0
  },
  "components": {
    "power_device": {"entity": "bsc093n15ns5"},
    "current_sensor": {"entity": "tmcs1133_b5a"},
    "voltage_sensor": {"entity": "lvfb_voltage_divider_150v"}
  }
}
```

### 5.3 Inline 分流器

```json
{
  "current_sensor": {
    "inline": {
      "id": "fsbb_5m_shunt",
      "schema": "current_sensor",
      "macro_prefix": "FSBB_5M_SHUNT",
      "parameters": {
        "chip": "5mR shunt + op amp",
        "range_a": 30.0,
        "bias_v": 1.65,
        "sensitivity_mv_per_a": 50.0,
        "internal_resistance_ohm": 0.005
      }
    }
  }
}
```

Inline 子实体会被展开到父硬件头文件中，不会生成 `fsbb_5m_shunt.h`。

## 6. 工程绑定

工程绑定把 suite 的需求宏映射到硬件参数。

```json
{
  "role": "inductor_current_sensitivity",
  "macro": "CTRL_INDUCTOR_CURRENT_SENSITIVITY",
  "binding": {
    "export": "fsbb_inline_shunt_half_bridge.current_sensor.sensitivity"
  }
}
```

这会生成：

```c
#define CTRL_INDUCTOR_CURRENT_SENSITIVITY FSBB_5M_SHUNT_SENSITIVITY_V_PER_A
```

板级外设映射用于 IRIS 这类硬件：

```json
"peripheral_bindings": {
  "PHASE_BUCK_BASE": "IRIS_F280039C_EPWM1_BASE",
  "FSBB_IL": "IRIS_F280039C_ADC_CH3"
}
```

这些值通常和 SysConfig 中的名字对应，例如 `F280039_Iris_node.syscfg` 中的 ADC、EPWM 实例名。

## 7. 和 Suite 的关系

SDPE v2 不直接替代 `xplt/ctrl_settings.h`，而是为它生成更小、更结构化的输入。

推荐落地方式：

1. 在 SDPE project 中定义工程需求。
2. 引入一组 hardware entity。
3. 生成硬件预设头文件和工程绑定头文件。
4. 在 `xplt/ctrl_settings.h` 中 include 生成的 project binding header。
5. 控制工程继续使用 `CTRL_*` 等既有宏初始化 ADC、PWM 和接口对象。

## 8. 测试

运行：

```powershell
python -m unittest discover -s .\tests
```

当前测试覆盖：

- 示例 schema/entity 校验。
- 复合硬件生成。
- inline 子实体不生成独立头文件。
- 生成 include 使用 GMP 根目录风格。
- 工程需求可以解析嵌套子硬件 export。
- 同一个子硬件被多个模块引用时不会重复输出。

## 9. 下一步

建议后续迭代：

- 增加 JSON Schema 文件，给 UI 和 CLI 共用。
- 增加 `ctrl_settings.h` patch/overlay 生成模式。
- 增加 `xplt.ctl_interface.h` 片段生成，用于 ADC/PWM 输入输出回调。
- 从 `*.syscfg` 自动提取 IRIS/C2000 外设名称。
- 增加 GUI：模板编辑、实体实例化、工程需求矩阵、实时预览。
- 将成熟的生成结果接入 `ctl/component/hardware_preset`。
