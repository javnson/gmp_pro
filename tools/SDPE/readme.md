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

说明这个控制算法需要哪些角色（Roles）。

- 例如：一个三相逆变器算法需要 `phase_a_curr`, `phase_b_curr`, `bus_voltage` 三个角色。

### 6.2 物理绑定 (Project Binding)

将逻辑角色映射到具体的硬件实例。

- `phase_a_curr` -> `ACS724_50B` (位于某个特定采样通道)。

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

### 工具 3：全库批量生成器 (Library Builder)

- **功能**：扫描所有的 `paradigms`，根据 `instance_databases` 里的所有型号，一次性生成全部的预设头文件。
- **用途**：用于维护和更新全局元件预设库。

### 工具 4：工程构建引擎 (Project Engine)

- **功能**：基于工程需求和连接关系构建文件。
- **核心特性**：**保留用户内容**。利用 `/* USER CODE BEGIN */` 保护区，确保用户手写的硬件强相关代码（如特定 DMA 配置、SPI 读位置信息）在重新生成时不会丢失。

------

## 8. 开发者工作流 (Workflow)

1. **定义范式**：在 `paradigms/` 建立物理模型。
2. **编写模板**：在 `templates/` 编写 C 代码映射规则。
3. **录入器件**：在 `inst/` 录入具体的型号参数。
4. **配置工程**：编写 Topology 指明需求。
5. **一键生成**：运行工具链，得到立即可用的标幺化控制环境。