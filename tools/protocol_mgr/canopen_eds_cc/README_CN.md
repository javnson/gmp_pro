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

GUI 支持：

- 表格式编辑对象字典条目；
- 导入并合并一个或多个 `.eds` 文件；
- 记录 `imports` 引用信息；
- 一键生成 `.h`/`.c`，并可按需导出合并 EDS。
- 交互风格参考 SDPE_v2：
  - 双视图编辑参数：**列表** 与 **参数树**；
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

### 3）EDS 转 JSON

```powershell
python canopen_eds_cc.py --to-json combo.json --import-eds cia301.eds cia402.eds --name gmp_combo_od --node-id 7
```

### 结果约定

指针模式会生成头文件 `extern` 声明和源文件实际对象；
字符串/字节数组类对象会用 `byte_gt` 数组表达，兼容 8bit/ C28x 平台。
`<name>_init()` 需要在系统初始化时执行一次即可完成字典注入。
