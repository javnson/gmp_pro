# SDPE (Software Defined Power Electronics) 开发者指南

## 1. 系统愿景与目标

**SDPE** 的核心目标是实现**电力电子控制逻辑与底层硬件的完全解耦**。

通过将硬件特性“参数化”和生成逻辑“模板化”，SDPE 允许工程师在不改动核心控制算法的前提下，通过修改配置文件即可适配不同的传感器、驱动板和 MCU 平台。它最终为用户构建出一个**标幺化 (Per-unit System)** 的控制环境。

------

## 2. 系统架构概述

系统的核心由三部分组成：

- **资源库 (Resources)**：定义“是什么”。由 `paradigms`（范式）、`templates`（代码逻辑）和 `inst`（具体器件数据库）组成。
- **描述文件 (Definitions)**：定义“怎么接”。包括 `Topology`（逻辑需求）和 `Project Binding`（物理连接）。
- **工具链 (Toolchain)**：定义“怎么做”。负责解析上述文件并生成可编译的 C 代码。

------

## 3. 元件定义规范 (Paradigms)

`paradigms` 文件夹中的 JSON 文件是系统的“基因”。它定义了一类硬件的通用属性。

### 3.1 核心字段说明

每一个范式 JSON 必须遵循以下结构：

| **字段名**           | **必选/可选** | **说明**                                                     |
| -------------------- | ------------- | ------------------------------------------------------------ |
| `paradigm_name`      | **必选**      | 范式的唯一识别名称。                                         |
| `output_path`        | **必选**      | 生成的 `.h` 预设文件的存放路径。基于 `$GMP_PRO_LOCATION`。   |
| `template_ref`       | **必选**      | 关联的 Jinja2 模板文件名。路径搜索基准：`$GMP_PRO_LOCATION/tools/SDPE/templates`。 |
| `parameters`         | **必选**      | 定义该类元件拥有的物理参数（如量程、精度等）。               |
| `instance_databases` | 可选          | 关联的具体器件型号库文件名。路径搜索基准：`$GMP_PRO_LOCATION/tools/SDPE/inst`。 |

### 3.2 路径规范示例

JSON

```
{
    "paradigm_name": "current_hall",
    "output_path": "ctl/hardware_presets/current_sensors", 
    "template_ref": "current_hall.j2",
    "instance_databases": [
        "db_hall_sensors.json",
        "db_my_custom_halls.json"
    ],
    "parameters": {
        "range_amp": {"unit": "A", "default": 100.0, "desc": "Full scale range"},
        "v_out_max": {"unit": "V", "default": 3.0, "desc": "Output voltage at full range"}
    }
}
```

------

## 4. 模板与代码生成逻辑 (Templates)

模板文件（`.j2`）决定了参数如何转化为 C 语言代码。

- **输入**：来自 `parameters` 中的物理参数。
- **输出**：通常包含两个部分：
  1. **宏定义 (Macros)**：计算传感器变比、增益等。
  2. **初始化函数 (Init Function)**：配置外设寄存器或软件对象。

------

## 5. 实例数据库 (Instances)

位于 `inst/` 目录下。它不含逻辑，只记录具体零件的参数值。

- **文件名**：需在 Paradigm 的 `instance_databases` 中引用。
- **格式**：一个以型号为 Key 的字典。

JSON

```
{
    "ACS724_50B": {
        "range_amp": 50.0,
        "v_out_max": 4.5
    }
}
```

------

## 6. 工程与需求定义

### 6.1 工程需求 (Project Topology)

工程定义文件（如 `c280049c_solution.json`）是具体项目的“施工蓝图”，它定义了软件如何与物理硬件连接。

### **核心字段详解**

| **字段**                 | **说明**                                                     |
| ------------------------ | ------------------------------------------------------------ |
| **`project_name`**       | 工程名称，用于生成文件头注释。                               |
| **`topology_ref`**       | **拓扑引用**。指明该项目遵循哪种控制架构（如 FOC, PFC 等）。 |
| **`global_environment`** | **全局环境**。定义系统级的物理常量（如 ADC 精度、基准电压/电流、时钟频率）。这些参数会注入所有组件。 |
| **`hardware_pool`**      | **硬件池**。列出本项目涉及的所有物理硬件节点及其对应的范式（Paradigm）和型号（Instance）。 |
| **`role_mapping`**       | **角色绑定**。将拓扑要求的“逻辑角色”映射到“硬件池”中的具体实例或其子模块。 |
| **`overrides`**          | **参数覆盖**。允许在工程层面对数据库中的默认值进行微调（如特定电机的相电阻或编码器偏置）。 |
| **`custom_macros`**      | **自定义宏注入**。这是最关键的部分，用于注入平台相关的底层驱动函数（如 TI C2000 的 `ADC_readResult`），实现硬件动作的闭环。 |

### **设计哲学**

- **解耦**：通过 `role_mapping`，如果你从霍尔传感器更换为分流电阻采样，只需修改此 JSON 的绑定关系，无需触动任何控制算法代码。
- **透明**：所有硬件细节都被抽象为 `SDPE_` 前缀的宏，算法开发者只需关注标幺化（PU）后的变量。

### 6.2 物理绑定 (Project Binding)

工程绑定文件的本质是回答：**“在这个特定的项目中，拓扑所要求的每个角色具体由哪个硬件实例承担，以及这些硬件如何与 MCU 交互？”**

### 1. 文件结构分解

一个标准的绑定文件包含以下五个关键区块：

#### A. 基础信息 (`project_name` & `topology_ref`)

- **`project_name`**: 定义工程的唯一标识，会出现在生成文件的注释首行。
- **`topology_ref`**: 极其重要。它必须对应 `proj_topology/` 目录下的一个 JSON 文件名。它决定了本项目必须分配哪些 `required_roles`。

#### B. 全局环境 (`global_environment`)

定义整个工程共享的电气和算力背景。

- 通常包含：ADC 位数（`adc_res`）、Q 格式位数（`adc_iqn`）、标幺化基准（`voltage_base`, `current_base`）以及外设时钟。
- **作用**：这些参数会作为“隐形背景”注入到所有硬件组件的计算公式中（例如计算 ADC 采样增益）。

#### C. 硬件池 (`hardware_pool`)

列出本项目物理上存在的所有硬件模块。

- 每个节点需要指定 `paradigm`（范式类型）和 `instance`（具体型号）。
- 如果是复合板卡（如包含电流采样和驱动的逆变板），需标注 `"is_composite": true`。

#### D. 角色映射 (`role_mapping`)

**这是文件的灵魂**。它将拓扑中的逻辑角色映射到硬件池。

- **语法**：`"逻辑角色名": "硬件池节点名"` 或 `"逻辑角色名": "硬件池节点名.子模块名"`。
- *示例*：`"phase_current": "main_inv_board.phase_current"` 表示三相电流角色由逆变板内部的电流采样逻辑实现。

#### E. 自定义驱动宏 (`custom_macros`)

解决“最后 1 厘米”的硬件调用问题。

- **作用**：将特定芯片的库函数（如 TI C2000 的 `ADC_readResult`）注入到组件的 `input` 或 `output` 动作中。
- **格式**：`"硬件节点名.参数名": "代码段"`。

------

### 2. 撰写步骤与检查清单

1. **对齐拓扑**：打开 `topology` 文件，确认所有的 `required_roles` 在 `role_mapping` 中都有对应的指向。
2. **实例化硬件**：在 `hardware_pool` 中填入你实际焊在板子上的器件型号。
3. **覆盖默认值**（可选）：如果在 `overrides` 中定义了参数，它会覆盖数据库 (`inst/*.json`) 中的原始值。适用于每块板子微小的校准偏差。
4. **注入驱动**：在 `custom_macros` 中，填写实际能跑通的 MCU 底层驱动代码。

------

### 3. 典型示例说明

JSON

```
{
    "project_name": "My_C2000_Servo_Drive",
    "topology_ref": "pmsm_sensored_foc",

    "global_environment": {
        "adc_res": 12, 
        "current_base": 10.0  // 算法看到的 1.0 对应物理 10A
    },

    "hardware_pool": {
        "sys_config": {
            "paradigm": "mcu_global_settings",
            "instance": "global_f280049c_100mhz_20khz"
        },
        "main_inv_board": {
            "paradigm": "inverter_board",
            "instance": "brd_lvhb_v1_sic",
            "is_composite": true
        }
    },

    "role_mapping": {
        "mcu_core": "sys_config",
        "phase_current": "main_inv_board.phase_current" 
    },

    "custom_macros": {
        // 将具体的 TI ADC 读取函数绑定到硬件节点
        "main_inv_board.curr_raw_data_source": "ADC_readResult(ADCA_BASE, ADC_CH_0)"
    }
}
```

### 4. 撰写建议

- **命名规范**：硬件池的 Key（如 `main_inv_board`）建议使用小写加下划线，因为它会决定生成代码中宏的前缀。
- **路径隔离**：尽量不要在绑定文件中写复杂的逻辑，只写**数据绑定**。复杂的算法逻辑应留在模板（`.j2`）中。
- **利用覆盖**：如果是量产项目，每台机器的校准系数可以通过 `overrides` 字段配合自动化脚本批量生成，而无需修改基础数据库。

------

## 7. 工具链说明 (SDPE Tools)

系统目前提供四种核心自动化工具：

- ## 工具 1: 组件调试编译器 (Debug Paradigms)

  ### 描述

  该工具用于在不构建整个工程的情况下，预览单个硬件组件（或整个数据库中的组件）生成的 C 代码。它会模拟外部环境（如 ADC 引用、PWM 对象名），并将结果输出到 `test/` 目录供人工审查。

  ### 位置

  ```
  $GMP_PRO_LOCATION/tools/SDPE/bin_py/debug_paradigms.py
  ```

  ### 核心参数

  - `-p, --paradigm`: **(必填)** 指定要测试的范式名称（如 `current_hall`）。
  - `-i, --instance`: (可选) 指定具体的器件型号。如果不指定，则批量生成该范式关联数据库中的**所有**型号。
  - `-m, --mode`: (可选) 覆盖默认模式（如强制切换到 `high_speed` 采样模式）。

  ### 示例指令

  **1. 调试特定型号的霍尔传感器：**

  Bash

  ```
  python bin_py/debug_paradigms.py -p current_hall -i inst_hall_acs724_20a
  ```

  - **动作**：查找 `current_hall` 范式，实例化 `ACS724_50B` 的参数。
  - **输出**：`test/ACS724_50B_debug.c`。

  **2. 批量测试某一类电阻分压器的所有配置：**

  Bash

  ```
  python bin_py/debug_paradigms.py -p current_hall
  ```

  - **动作**：遍历 `voltage_divider` 关联的所有数据库，为每个型号生成预览代码。
  - **用途**：用于检查模板逻辑（J2）是否适配所有量程。

  **3. 测试复合板卡的输出逻辑：**

  Bash

  ```
  python bin_py/debug_paradigms.py -p inverter_board -m sic_driven_mode
  ```

  ### 检查要点

  生成的预览文件中会包含 `/* FROM_OUT: ... */` 标签。这代表该变量在实际工程中将由 `Topology` 或其他硬件模块提供，在调试阶段仅做占位处理。

- ## 工具 2: 全库批量预览工具 (Batch Preview All)

  ### 描述

  该工具是 `debug_paradigms` 的自动化升级版。它会自动扫描 `paradigms/` 目录下的所有范式文件，并针对每个范式关联的数据库，生成库中**所有**器件型号的渲染预览。

  ### 位置

  ```
  $GMP_PRO_LOCATION/tools/SDPE/bin_py/debug_preview_all.py
  ```

  ### 使用场景

  - **模板回归测试**：当你修改了 `templates/common_logic.j2` 等通用底层模板时，运行此工具可以一次性检查是否导致某些特定型号（如三相传感器或复合板卡）的生成代码报错。
  - **库同步校验**：检查是否有新增的实例 JSON 缺少了 Paradigm 要求的必要参数。

  ### 示例指令

  Bash

  ```
  python bin_py/debug_preview_all.py
  ```

  ### 运行结果

  生成的代码将统一存放在 `SDPE/test/` 目录下，文件名格式为 `{instance_name}_preview.c`。

- ## 工具 3: 正式元件库生成器 (Library Builder)

  ### 描述

  该工具将范式与数据库结合，生成立即可供 C 语言工程包含 (`#include`) 的正式头文件。它会自动处理目录结构，并具备智能写入功能，保护文件修改时间戳以加速工程编译。

  ### 位置

  ```
  $GMP_PRO_LOCATION/tools/SDPE/bin_py/sdpe_build_library.py
  ```

  ### 核心特性

  - **自动归档**：根据范式 JSON 中的 `output_path` 自动决定存放位置。
  - **智能写入**：若生成的代码内容与磁盘现有文件完全一致，则跳过写入操作，避免触发 IDE 的增量编译逻辑。

  ### 示例指令

  **1. 生成全部传感器库到默认路径：**

  Bash

  ```
  python bin_py/sdpe_build_library.py -p current_hall
  ```

  - **结果**：文件将根据 `output_path` 放入 `ctl/hardware_presets/current_sensors/`。

  **2. 强制将特定型号生成到临时测试位置：**

  Bash

  ```
  python bin_py/sdpe_build_library.py -p current_hall -i ACS724_50B -o tmp_debug/sensors
  ```

- ## 工具 4: 全库自动化构建工具 (Full Library Builder)

  ### 1. 描述

  `sdpe_build_all.py` 是 SDPE 元件生产线的“总调度员”。它会自动遍历 `paradigms/` 文件夹及其所有子文件夹，识别每一个范式定义，并调用 **Tool 3** 为该范式关联的所有硬件型号生成正式的、纯净的参数头文件（`.h`）。

  该工具确保了底层数据库（`inst/`）与工程正式引用库（`ctl/hardware_presets/`）之间的数据一致性。

  ### 2. 工作原理

  1. **扫描 (Scan)**：递归搜索 `paradigms/` 下的所有 `.json` 文件。
  2. **调度 (Dispatch)**：针对每个范式，提取其关联的 `instance_databases`。
  3. **构建 (Build)**：利用 `sdpe_build_library` 模块进行渲染，将物理参数换算为 C 语言宏。
  4. **智能同步 (Smart Sync)**：仅当计算出的内容发生变化时才会覆盖磁盘文件，保护工程编译时间。

  ### 3. 位置

  ```
  $GMP_PRO_LOCATION/tools/SDPE/bin_py/sdpe_build_all.py
  ```

  ### 4. 使用场景

  - **初次部署**：在新的开发环境下生成所有标准硬件预设。
  - **大规模更新**：修改了某个范式的计算公式（`.j2` 模板）后，刷新全量库。
  - **型号扩充**：在 `inst/` 数据库中录入了数十个新器件后，一键生成对应的头文件。

  ### 5. 示例指令

  在 `tools/SDPE` 目录下执行：

  Bash

  ```
  python bin_py/sdpe_build_all.py
  ```

  ### 6. 输出结构示例

  运行后，它会根据各范式定义的 `output_path` 自动填充目标目录：

  Plaintext

  ```
  $GMP_PRO_LOCATION/
  └── ctl/
      └── hardware_presets/
          ├── current_sensors/      <-- 来自 current_hall.json
          │   ├── ACS724_20A.h
          │   └── LEM_HASS_50_S.h
          └── voltage_sensors/      <-- 来自 voltage_divider.json
              ├── RES_DIV_100V.h
              └── RES_DIV_600V.h
  ```

  ### 7. 开发者注意事项

  - **环境依赖**：运行前请确保已设置系统环境变量 `GMP_PRO_LOCATION`，否则工具可能无法正确定位工程根目录。
  - **纯净性保证**：此工具生成的头文件**不包含**任何 Handle 声明或初始化逻辑。如果你在生成的文件中发现了 `adc_channel_t` 等对象定义，请检查你的 `.j2` 模板是否错误地将这些内容放进了 `config_macros` 块中。
  - **只读属性**：虽然生成的是 `.h` 文件，但请将其视为“只读”资源。任何对参数的修改都应在 `inst/*.json` 中进行，然后重新运行此工具。

  ## 工具5：工程构建引擎 (`sdpe_gen_xplt_src.py`)

  ### **描述**

  `sdpe_gen_xplt_src.py` 是 SDPE 系统的“总装配台”。它的任务是将**静态硬件库**（由 Tool 3 生成）与**工程特定需求**（Project JSON）进行缝合，生成最终的 C 语言接口文件。

  ### **核心功能**

  - **多源路径搜索**：智能搜索工程文件和拓扑定义，支持绝对路径、PWD（当前目录）、`$GMP_PRO_LOCATION` 以及工具内置路径。
  - **API 抽象映射**：将拓扑中的逻辑角色（如 `phase_current`）映射到具体的硬件物理宏，使算法层实现“硬件无关化”。
  - **代码保护（User Code Preservation）**：通过识别 `/* USER CODE BEGIN */` 标签，确保在重新生成接口时，用户手写的底层驱动或自定义逻辑不会丢失。
  - **动作流缝合**：将分布在各组件中的 `input`（数据采集）和 `output`（调制输出）代码块聚合，生成统一的系统回调函数。

  ### **使用指令**

  PowerShell

  ```
  # 格式：python sdpe_gen_xplt_src.py <工程JSON路径> [输出目录]
  python bin_py/sdpe_gen_xplt_src.py proj_topology/c280049c_solution.json ./gen
  ```

------

## 8. 开发者工作流 (Workflow)

1. **定义范式**：在 `paradigms/` 建立物理模型。
2. **编写模板**：在 `templates/` 编写 C 代码映射规则。
3. **录入器件**：在 `inst/` 录入具体的型号参数。
4. **配置工程**：编写 Topology 指明需求。
5. **一键生成**：运行工具链，得到立即可用的标幺化控制环境。