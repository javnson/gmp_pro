# canopen_eds_cc

`canopen_eds_cc.py` 用于在 **EDS ↔ JSON ↔ 生成代码** 之间转换 CANopen 对象字典，
并保持向后兼容现有 `EDS -> C` 的调用方式。

## 使用方式

## GUI模式（GMP启动器）

在 GMP 环境下直接运行（优先）：

```powershell
gmp_canopen_eds_cc_gui.bat [path\to\project.json]
```

- 不带参数启动：打开空白项目编辑窗口；
- 带参数启动：直接加载指定项目 JSON；
- 启动器会先检查 `GMP_PRO_LOCATION`，并调用 `tools\gmp_installer\ensure_gmp_environment.bat`。

GUI 支持（PySide6）：

- 统一的对象条目树表页（列表+树状结构合并），上部是 **Entities**，下部是 **Imports**；
- 相同 Index 的条目会自动聚合到同一展开/折叠的索引节点；
- 索引节点标题会显示友好信息，例如 `0x2000 [0x00..0x02] (3 entries: ActualPos (+2))`；
- 导入并合并一个或多个 `.eds` 文件；
- 记录 `imports` 引用信息；
- 一键生成 `.h`/`.c`，并可按需导出合并 EDS。
- 交互风格参考 SDPE_v2：
  - 参数树作为主编辑面板，支持右键菜单与快捷键；
  - 数据类型以具体枚举名展示（如 `UNSIGNED16`）并自动联动 Size；
  - 双击单元格直接编辑字段（默认不再弹出编辑窗口）；
  - `PDO` 列使用勾选框，直接开关映射属性；
  - `Access` 列改为下拉 + `Const` 复选的编辑器，减少手工输错，显示短标签（`R` / `W` / `R/W` / `C`），并保持固定宽度、居中对齐；
  - Access 属性拆成 `Read` / `Write` / `Const` 复选项；
  - Storage 为 variable 时自动生成 `gmp_canopen_od_XXXX_YY` 风格变量名（可手动覆盖）；
  - 导入管理（新增/编辑/删除/上下移动/替换重载）；
  - 预设模板：`tools/protocol_mgr/canopen_eds_cc/presets` 支持保存/加载/删除。
- 快捷键：
  - `Ctrl+N` 新建工程
  - `Ctrl+O` 打开工程
  - `Ctrl+S` 保存
  - `Ctrl+Shift+S` 另存为
  - `Insert` 新增条目
  - `Enter` 编辑条目
  - `Delete` 删除条目
  - `Ctrl+I` 从 EDS 导入条目
  - `Ctrl+G` 生成
  - `F5` 刷新预览
  - `Ctrl+Q` 退出

- 左侧还提供「当前打开文件」停靠窗，可在 **View** 菜单里显示/隐藏，默认加载：
  - `core/protocol/canopen/cia301/cia301_project.json`
  - `core/protocol/canopen/cia401/cia401_project.json`
  - `core/protocol/canopen/cia402/cia402_project.json`

这份列表来自 `canopen_eds_cc_gui_config.json`，位于
`tools/protocol_mgr/canopen_eds_cc/`；你可以修改 `quick_open_projects` 指定更多/其他工程路径（支持相对 GMP 根路径或绝对路径），也可以调整标题、默认可见性、宽度等展示参数。

### 1）兼容模式

```powershell
python canopen_eds_cc.py device.eds --output-dir generated --name device_od --storage pointer --node-id 7
```

字段支持：

- `ParameterName`
- `DataType`
- `AccessType`
- `DefaultValue` / `ParameterValue`
- `PDOMapping`
- `DataLength`（可变长度类型）

`[1234]` 或 `[1234sub5]` 分区可被解析；`GMPStorage=pointer|value` 会覆盖命令行默认存储。

### 2）项目模式（JSON）

通过 JSON 形成完整工程描述：

- 多源绑定：`imports`
- 树形展示：`tree`、`group_path`
- 存储模式：
  - `pointer`：工具生成存储符号
  - `value`：固定值方式
  - `variable`：绑定既有应用变量（`variable_name`）
- 头/源代码片段插桩（前缀和后缀）
- 重复对象冲突策略：`error|first|last`
- `--emit-eds` 可额外导出整合后的 EDS

```powershell
python canopen_eds_cc.py --project dictionary.json --output-dir generated --node-id 7 --emit-eds generated/combo.eds
```

### 2.1）GMP 现有方案配置文件

每个 CiA 档位的定制配置均放在对应目录内，和 `.eds` 同级：

- `core/protocol/canopen/cia301/cia301_project.json`
- `core/protocol/canopen/cia401/cia401_project.json`
- `core/protocol/canopen/cia402/cia402_project.json`

这些项目文件使用相对路径记录 `imports` 与 `output_dir`，可直接原位重新生成：

```powershell
python tools/protocol_mgr/canopen_eds_cc/canopen_eds_cc.py --project core/protocol/canopen/cia301/cia301_project.json --node-id 1
python tools/protocol_mgr/canopen_eds_cc/canopen_eds_cc.py --project core/protocol/canopen/cia401/cia401_project.json --node-id 1
python tools/protocol_mgr/canopen_eds_cc/canopen_eds_cc.py --project core/protocol/canopen/cia402/cia402_project.json --node-id 1
```

GUI 启动方式（推荐）：

```powershell
gmp_canopen_eds_cc_gui.bat core/protocol/canopen/cia301/cia301_project.json
```

### 3）EDS 转 JSON

```powershell
python canopen_eds_cc.py --to-json combo.json --import-eds cia301.eds cia402.eds --name gmp_combo_od --node-id 7
```

### 结果约定

指针模式会生成头文件 `extern` 声明和源文件实际对象；
字符串/字节数组类对象会用 `byte_gt` 数组表达，兼容 8bit/ C28x 平台。
`<name>_init()` 需要在系统初始化时执行一次即可完成字典注入。
