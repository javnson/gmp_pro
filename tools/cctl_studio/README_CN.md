# GMP CCTL Studio

[English](README.md) | **简体中文**

已审定的目标数据结构、`mcs_pmsm_nt` 自动生成边界与分阶段验收门槛见
[架构与执行计划](ARCHITECTURE_CN.md)。

本目录已经包含可运行的 Qt 离线桌面图形编辑器，以及两条数据驱动链路：新建模拟
电路可直接导出 `MNA_Solver` 网表，旧 schema-v1 工程继续使用 Xyce 网表生成器。
其架构借鉴 TI
SysConfig 工具中值得采用的分层方式：元件属性和工程连线属于数据，通用引擎只负责
校验数据并生成求解器输入。这里不包含 TI 源码或素材，也不修改 GMP 的 UDP/TCP
通信模块，当前阶段尚未把 CCTL 电机模型接入 Xyce。

## 已实现内容

- 基于 PyQt5/Qt 5 的桌面编辑器，使用 GMP 私有 Python 环境中固定的 Qt 运行时；
- 两级编辑架构：主层使用数值信号模块框图，双击复合模块进入其专用子层；
- 主层提供电气主拓扑、数字模块、电机模型和信号适配器，所有主层端口均为
  `numeric`；
- 新建模拟电路子层的元件目录与 `mna_solver.py::parse_netlist()` 对齐，覆盖
  R/L/C、独立源、理想运放、IdOpamp、E/G/F/H 受控源、D/M/S 和电流表；
- 元件库提供可按 `GND` 或 `GMD` 搜索的接地符号，两者导出时均规范化为 MNA 节点 `0`；
- MOSFET 和压控开关是内置理想 PWM 驱动的复合元件，只暴露 D/S 或功率端；导出器按
  图中顺序展开为 `MTn/SWn + VPWMn`，MNA 生成器再把 `VPWMn` 识别为 `PWMn` 控制端；
- 电路子层使用电气符号和可编辑多段正交导线；单击端口开始布线，单击空白处放置
  拐点，再单击目标端口结束；在导线上结束布线或双击导线会建立电气 junction；
- 数字子层使用输入/输出、AND/OR/NOT、延时和单稳态符号及类型化有向逻辑连线；
- 工具栏返回、面包屑导航和设计树均可在主层与子层之间切换；
- 可搜索元件面板、拖放/双击添加、元件拖动、框选和多选；
- 右侧属性检查器以元件参数为中心；执行顺序仅显示在主系统层，不显示在模拟/数字子层；
- 元件支持 90° 旋转和水平镜像，移动或变换元件时相邻导线段随引脚正交伸缩；空格
  旋转后继续保持元件选中；
- 所有连接端点与网格对齐；滚轮缩放，中键或在空白画布上右键拖动平移；
- 撤销/重做、复制实例、删除以及 JSON 工程打开/保存；
- 新建模拟电路可导出 MNA `.cir`，并由现有 MNA 解析器直接读取；
- 元件由 JSON 定义：端口、参数、校验类型和 Xyce 网表模板；
- 工程由 JSON 定义：元件实例、网络连接、瞬态分析和观测量；
- 生成保守、便于移植的 SPICE/Xyce `.cir` 网表；
- 支持附加元件库，增加合法 JSON 元件无需修改生成器代码；
- 按 `--xyce`、`XYCE_EXECUTABLE`、`PATH` 的顺序查找 Xyce；
- 提供 RC 低通示例和自动测试。

`.cir` 只是常用扩展名，内容仍是文本 SPICE 网表：首行为标题，随后是元件语句，
分析和输出命令以 `.` 开始，以 `.END` 结束。Xyce 支持这种网表，但并不保证与所有
厂商的 PSpice/HSPICE/Spectre 方言完全兼容，厂商模型后续需要增加方言检查或转换层。

## 快速验证

先运行 `tools\gmp_installer\activate_env.bat` 激活已安装的 GMP 私有环境，再在仓库根目录执行：

```powershell
tools\cctl_studio\run_cctl_studio.bat
tools\cctl_studio\run_cctl_studio.bat tools\cctl_studio\cctl_core\examples\rc_low_pass\project.json
```

也可以直接运行 `python tools/cctl_studio/cctl_core/qt_studio.py [project.json]`。启动后首先显示
`System [system]` 主层，双击带有 “Double-click to open” 的复合模块进入子层。编辑器
快捷操作包括：

- 单击元件端口圆点开始导线，单击画布放置任意多个正交拐点，单击目标端口完成；
  布线过程中按空格可在两种正交拐角方向之间切换；
- 直接拖动导线段或选择导线后拖动蓝色路径点；双击导线建立 junction；右键或 `Esc`
  取消正在绘制的导线；
- 中键拖动或在空白画布上右键拖动可平移画布，滚轮缩放，`F6` 适合窗口；
- 框选或按住 Shift 多选；空格或 `R` 顺时针旋转，`Shift+R` 逆时针旋转，`M` 镜像；
- `Ctrl+Z/Y` 撤销/重做，`Ctrl+D` 复制实例，Delete 删除；
- `Alt+Left` 或工具栏 Back 返回父层，面包屑按钮可直接跳回任一祖先层。

命令行生成器仍可独立使用：

```powershell
python tools/cctl_studio/cctl_core/cctl_studio.py list-components
python tools/cctl_studio/cctl_core/cctl_studio.py validate tools/cctl_studio/cctl_core/examples/rc_low_pass/project.json
$output = Join-Path $env:GMP_PRO_LOCATION 'tmp\cctl_studio\rc_low_pass\cctl_studio_rc.cir'
python tools/cctl_studio/cctl_core/cctl_studio.py generate tools/cctl_studio/cctl_core/examples/rc_low_pass/project.json -o $output
```

安装 Xyce 后可直接求解：

```powershell
$runOutput = Join-Path $env:GMP_PRO_LOCATION 'tmp\cctl_studio\rc_low_pass\xyce'
python tools/cctl_studio/cctl_core/cctl_studio.py run tools/cctl_studio/cctl_core/examples/rc_low_pass/project.json -o $runOutput --xyce C:/path/to/Xyce.exe
```

求解波形由 `.PRINT TRAN FORMAT=CSV` 写入运行目录。

## 无代码增加元件

参照 `cctl_core/components/` 增加一个 JSON 文件，在工程的 `libraries` 数组中引用其相对路径，
或通过可重复的 `--library` 参数传入文件/目录。模板当前被限制为一行，允许使用：

- `$instance`：实例名；
- `$port_<端口名>`：连接网络；
- `$param_<参数名>`：实例参数或默认值。

例如电流源实例可以由模板
`$instance $port_p $port_n DC $param_current` 生成，通用程序无需知道“电流源”这一类型。

## 数据兼容边界

当前编辑器读取和保存 schema v1。布局、层级、视图和执行顺序存放在可选的 `editor`
对象中，现有 Xyce 生成器会忽略这部分，因此旧工程可以直接打开，保存后的工程仍能生成
相同网表。该元数据是 UI 框架的兼容层，不代替规划中的 schema v2 类型化显式连接。

`editor.hierarchy` 明确保存每层的 `kind`、节点、连接、视图和复合节点的 `child_layer`。
旧工程的 `project.instances` 自动映射为默认 `Main Topology` 的兼容电路子层，并继续走
原有 Xyce 生成器。新建模拟电路采用显式连接，能够导出当前 MNA Solver 可解析的网表；
数字子层和跨层系统图在 schema v2 生成器完成前不会被静默加入 CCTL 代码生成输入。

所有 Studio Python 源码、内置元件、示例和测试集中在 `cctl_core/`；外层入口由
`run_cctl_studio.bat` 和 `run_tests.bat` 提供。

## 后续实施边界

1. 增加正式 JSON Schema，并将现有 UI 框架切换到 schema-v2 规范化模型；
2. 将编辑器私有层级节点规范化为可生成的层次模块、`.SUBCKT` 和厂商模型文件清单；
3. 定义带采样周期的控制/数字端口，建立确定性的 CCTL-Xyce 协同仿真桥；
4. 增加 PWM、ADC、编码器和记录探针适配器；
5. 在工程格式上层增加 OpenDSS 工况/工作点导入。

核心原则是让 JSON 成为稳定接口：未来的 SysConfig 风格界面只读写工程数据，不把
具体元件逻辑固化在 UI 程序中。
