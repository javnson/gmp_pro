# SDPE v2 软件定义电力电子工具

[English](README.md) | **简体中文**

SDPE 是 Software Defined Power Electronics 的缩写。它本质上是一个数据管理与关联工具：Template 定义可导出的参数面，Entity 填写具体数据，Sub Components 引入其他硬件对象，Project Requirement 再把这些对象的参数绑定到控制工程需求。

一个硬件头文件应当主要导出 Template Parameters 中声明的宏。需要来自其他器件的数据时，通过 Sub Components include 对方头文件，并在当前 Parameters 中引用子组件的参数或导出量。例如半桥模块可以导出自己的测量电流范围宏，而这个宏的值来自 `current_sensor.range`。

v2 当前采用“核心 CLI + PyQt 图形化管理器”的结构。CLI 负责数据校验和头文件生成，PyQt 管理器负责以对象化表单维护模板、元件、工程需求和需求绑定。`gui/` 下保留了一个轻量 Web 原型，便于快速查看数据。

正式部署后，SDPE 使用 `%GMP_PRO_LOCATION%` 定位仓库：

- Template 原始文件：`%GMP_PRO_LOCATION%/ctl/hardware_preset/sdpe_schemas`
- Entity 原始文件：`%GMP_PRO_LOCATION%/ctl/hardware_preset/sdpe_src/<category>`
- 全局硬件头文件输出：`%GMP_PRO_LOCATION%/ctl/hardware_preset/<category>`
- 工程局部输出：工程的 `sdpe_mgr` 目录；工程设置头文件放在 `sdpe_mgr` 根目录，硬件头文件放在 `sdpe_mgr/hardware_preset`
- Suite 公共配置：`<suite>/sdpe_general`；公共 C 头文件输出到 `<suite>/src`，公共 MATLAB 初始化脚本保留在 `sdpe_general`
- 默认设置文件：`%GMP_PRO_LOCATION%/tools/SDPE_v2/sdpe_settings.json`

全局硬件头文件生成入口：

```bat
%GMP_PRO_LOCATION%\ctl\hardware_preset\gmp_sdpe_generate_all.bat
```

工程局部生成入口位于工程的 `sdpe_mgr` 文件夹，例如：

```bat
%GMP_PRO_LOCATION%\ctl\suite\pgs_sinv_rc\project\f280039c_Iris_node\sdpe_mgr\sdpe_generate.bat
```

两层工程可以直接双击任意 `sdpe_edit.bat`：从 `sdpe_general` 打开时会同时载入
该 suite 下所有 `project/**/sdpe_mgr`；从某个目标的 `sdpe_mgr` 打开时会同时载入
它自身和 `sdpe_general`。不带参数运行 `gmp_sdpe_deploy_project_mgr.bat` 会发现并更新
仓库中的全部 `sdpe_mgr` 和 `sdpe_general` 工具脚本。

## 1. 核心概念

### 1.1 Schema

Schema 定义一类硬件的规范，正式部署路径为 `ctl/hardware_preset/sdpe_schemas`。`tools/SDPE_v2/examples/schemas` 仅作为工具开发样例保留。

它描述：

- 这类硬件有哪些参数。
- 这些参数是否来自当前硬件本身，或者引用某个 Sub Component。
- 哪些参数必填，哪些有默认值。
- 参数如何生成 C 宏。
- 这类硬件可以包含哪些子硬件。
- 这类硬件向工程需求导出哪些逻辑量。
- 这类硬件有哪些 `tags`，用于搜索和分类。

Template Definition 中的 Sub Components 是默认子模块，格式与 Entity Instance 中的 Sub Components 一致。创建或编辑 Entity 时，这些默认子模块会自动带出；Entity 可以直接使用，也可以用同名 slot 覆盖。

例如 `current_sensor` 可以要求芯片型号、量程、输出范围、偏置、灵敏度、内阻等信息。

### 1.2 Entity

Entity 是 schema 的具体实例，正式部署路径为 `ctl/hardware_preset/sdpe_src/<category>`。`tools/SDPE_v2/examples/entities` 仅作为工具开发样例保留。

例如：

- `tmcs1133_b5a` 是一个具体电流传感器。
- `bsc093n15ns5` 是一个具体功率 MOSFET。
- `lvfb_half_bridge_phase_a` 是一个复合半桥模块，包含功率器件、电流传感器、电压传感器。

Entity 可以引用另一个独立 entity，也可以使用 inline 子实体。Inline 子实体不会生成独立头文件，适合“某个分流器阻值只属于当前板卡”的场景。

Entity 支持 `tags`，也支持对引用进来的完整子对象做局部覆盖：

```json
"current_sensor": {
  "entity": "tmcs1133_b2a",
  "overrides": {
    "range_a": 25.0,
    "bias_v": 1.64
  }
}
```

这种写法仍然 include `tmcs1133_b2a.h`，但父对象会额外生成 `PARENT_CURRENT_SENSOR_*` 这一组本地宏。未覆盖参数会自动别名到原始元件，覆盖参数则使用父对象本地值。

Entity 的参数值支持四类写法：

```json
{"literal": "(2048U)"}
{"macro": "EXISTING_C_MACRO"}
{"ref": "current_sensor.range"}
{"expr": "({current_sensor.internal_resistance_ohm} * 20.0f)"}
```

`ref` 和 `expr` 用于从 Sub Components 中引用参数或导出量。`expr` 中用 `{slot.parameter}` 或 `{slot.export}` 写占位符。

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
├── gui_pyqt/
│   └── sdpe_gui.py
├── gui/
│   ├── server.py
│   └── static/
└── tests/
```

## 3. 快速开始

在 `tools/SDPE_v2` 目录下运行：

```powershell
python .\sdpe.py --library .\examples validate
```

Windows 用户也可以直接使用批处理入口：

```powershell
.\gmp_sdpe_generate_all.bat
.\gmp_sdpe_gui.bat
.\gmp_sdpe_project_gui.bat
```

`gmp_sdpe_generate_all.bat` 的第一个参数是库目录，第二个参数是输出目录：

```powershell
.\gmp_sdpe_generate_all.bat .\examples .\build
```

`gmp_sdpe_gui.bat` 的第一个参数是库目录：
第二个参数是工作模式，可选 `all`、`library`、`project`：

```powershell
.\gmp_sdpe_gui.bat .\examples
.\gmp_sdpe_gui.bat .\examples library
```

`gmp_sdpe_project_gui.bat` 用于只管理某个工程的需求和绑定。第一个参数是硬件库目录，第二个参数是 project requirement 文件或目录：

```powershell
.\gmp_sdpe_project_gui.bat .\examples .\examples\projects
```

启动 PyQt 图形化管理工具：

```powershell
python .\gui_pyqt\sdpe_gui.py --library .\examples
python .\gui_pyqt\sdpe_gui.py --library .\examples --mode library
python .\gui_pyqt\sdpe_gui.py --library .\examples --mode project --projects .\examples\projects
```

PyQt 管理器包含 4 个工作页：

| 页面 | 对应需求 |
| --- | --- |
| Template Definition | 定义元件模板、参数、组件槽位和 tag。 |
| Entity Instance | 实例化模板、引入子元件、编辑 overrides，并生成单个元件头文件。 |
| Project Requirement | 为 suite 创建工程需求、引入硬件、维护需求宏和外设映射。 |
| Requirement Binding | 将需求宏绑定到硬件导出量、直接参数、已有宏或手工 literal，并生成工程绑定头文件。 |
| Settings | 设置模板路径、元件路径、工程需求路径和默认头文件生成路径。 |

每个页面采用可开关的横向面板布局：

| 面板 | 用途 |
| --- | --- |
| Items | 当前对象列表和搜索。 |
| Basic | 图形化对象编辑区。 |
| Professional | JSON 专业视图，用于检查底层数据。 |
| Code | 生成头文件预览。Template Definition 没有最终头文件，因此不显示 Code。 |

默认只打开 `Items` 和 `Basic`，把主要空间留给图形化编辑。用户需要检查 JSON 或最终生成头文件时，再打开 `Professional` 或 `Code`。

`Template Definition` 和 `Entity Instance` 面向硬件库，适合固定保存在 GMP 仓库或扩展包中。`Project Requirement` 和 `Requirement Binding` 面向具体工程，建议通过 `--projects` 指向 suite 工程内的需求文件或目录。

Project Requirement 的 Basic 页面提供 `Macro Prefix`。为它设置独立前缀后，Project 元数据宏会生成为
`<PREFIX>_SDPE_PROJECT_ID`、`<PREFIX>_SDPE_PROJECT_SUITE`、`<PREFIX>_SDPE_PROJECT_VERSION` 和
`<PREFIX>_SDPE_PROJECT_UPDATED_AT`，因此一个软件工程可以安全包含多个 SDPE Project 生成头文件。未填写时保持旧命名以兼容现有工程。

启动轻量 Web 原型：

```powershell
python .\gui\server.py --library .\examples --host 127.0.0.1 --port 8765
```

然后打开：

```text
http://127.0.0.1:8765/
```

Web GUI 当前包含 5 个工作页：

| 页面 | 对应需求 |
| --- | --- |
| 模板定义 | 定义元件的模板和规范。 |
| 元件实例 | 实例化模板，构建独立元件或复合元件。 |
| 头文件生成 | 生成单个元件、全部元件或工程绑定头文件。 |
| 工程需求 | 为 suite 创建工程需求，引入硬件和需求宏。 |
| 需求绑定 | 将工程需求宏绑定到具体硬件导出参数。 |

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

### 项目侧 sdpe_mgr 部署

在实际 suite 工程中，推荐像 `gmp_src_mgr` 一样建立项目本地的 `sdpe_mgr` 文件夹。主程序仍保存在：

```text
%GMP_PRO_LOCATION%\tools\SDPE_v2
```

在目标工程目录下部署：

```bat
%GMP_PRO_LOCATION%\tools\SDPE_v2\gmp_sdpe_deploy_project_mgr.bat .
```

也可以显式指定工程目录：

```bat
%GMP_PRO_LOCATION%\tools\SDPE_v2\gmp_sdpe_deploy_project_mgr.bat E:\lib\gmp_pro\ctl\suite\mcs_pmsm_nt\project\f280039c_Iris_node
```

生成的 `sdpe_mgr` 包含：

- `sdpe_requirement.json`：当前工程需求和硬件绑定。
- `sdpe_settings.bat`：由统一模板分发的运行路径设置。
- `sdpe_edit.bat`：启动 Project Requirement GUI。
- `sdpe_generate.bat`：生成工程 SDPE 头文件。
- `sdpe_validate.bat`：检查 SDPE 库和当前需求文件是否可读取。

默认生成输出目录为：

```text
sdpe_mgr\
  <project_settings_header>.h
  hardware_preset\
```

工程差异应保存在 `sdpe_requirement.json` 或其他独立配置文件中，不要直接
修改会在安装时被覆盖的 BAT。若所有工程都需要新的启动行为，应修改
`tools\SDPE_v2\sdpe_mgr` 中的规范模板。

### 全仓库工具链分发

工程侧的以下四个批处理属于发行工具链，不再作为各工程的独立源码维护：

- `sdpe_edit.bat`
- `sdpe_generate.bat`
- `sdpe_settings.bat`
- `sdpe_validate.bat`

唯一模板位于 `tools\SDPE_v2\sdpe_mgr`。安装程序会运行：

```bat
python tools\SDPE_v2\distribute_sdpe_mgr.py
```

该程序会全仓库搜索带有 `sdpe_requirement.json` 的 `sdpe_mgr`，使用 Git
自身解释 `.gitignore` 并排除 Debug 等忽略目录，然后原子覆盖上述四个
BAT。它不会修改工程的 requirement、README、生成头文件、Matlab 初始化
文件或 `hardware_preset`。只查看分发范围而不写文件时使用：

```bat
python tools\SDPE_v2\distribute_sdpe_mgr.py --dry-run
```

工程内的四个 BAT 已加入 `.gitignore`。维护工具链时只修改规范模板，不要
修改工程副本；新建工程只需准备 `sdpe_mgr\sdpe_requirement.json`，下一次
安装或手动分发会补齐最新启动脚本。

## 4. 生成结果

在 GMP 主仓库中，生成的工程头文件和 `*_matlab_init.m` 与 SDPE requirement
及库数据重复，因此可以不提交。工程复制到 GMP 仓库之外并作为独立仓库维护时，
必须提交这些生成结果以及工程侧生成工具，使独立工程无需父仓库也能直接编译，
同时保留重新生成能力。

生成目录示例：

```text
sdpe_mgr/
├── hardware_preset/
│   ├── current_sensor/
│   ├── half_bridge/
│   ├── mcu_board/
│   ├── power_switch/
│   └── voltage_sensor/
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

工程绑定也可以直接引用参数路径：

```json
{
  "macro": "CTRL_REFERENCE_CURRENT_RANGE_A",
  "binding": {
    "export": "tmcs1133_b2a.range_a"
  }
}
```

如果需求值不属于任何元件，可以使用手工输入：

```json
{
  "macro": "CTRL_VIN_ADC_OFFSET_MANUAL",
  "binding": {
    "literal": "(2048U)"
  }
}
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
- 工程需求可以解析 `entity.parameter` 直接参数路径。
- 组件 overrides 会生成父对象槽位作用域下的本地宏。
- 同一个子硬件被多个模块引用时不会重复输出。

## 9. 下一步

建议后续迭代：

- 增加 JSON Schema 文件，给 UI 和 CLI 共用。
- 增加 `ctrl_settings.h` patch/overlay 生成模式。
- 增加 `xplt.ctl_interface.h` 片段生成，用于 ADC/PWM 输入输出回调。
- 从 `*.syscfg` 自动提取 IRIS/C2000 外设名称。
- 继续增强 PyQt GUI：对象复制、批量 tag、参数绑定矩阵、差异预览。
- 完善工程 `ctrl_settings.h` 对 `sdpe_mgr/<project_settings_header>.h` 的集成方式。
