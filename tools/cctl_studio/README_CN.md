# CCTL Studio 验证原型

[English](README.md) | **简体中文**

本目录用于离线验证“数据驱动的电路编辑 + Xyce 网表生成”路线。其架构借鉴 TI
SysConfig 工具中值得采用的分层方式：元件属性和工程连线属于数据，通用引擎只负责
校验数据并生成求解器输入。这里不包含 TI 源码或素材，也不修改 GMP 的 UDP/TCP
通信模块，当前阶段尚未把 CCTL 电机模型接入 Xyce。

## 已实现内容

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

在仓库根目录执行：

```powershell
python tools/cctl_studio/cctl_studio.py list-components
python tools/cctl_studio/cctl_studio.py validate tools/cctl_studio/examples/rc_low_pass/project.json
python tools/cctl_studio/cctl_studio.py generate tools/cctl_studio/examples/rc_low_pass/project.json -o $env:TEMP/cctl_studio_rc.cir
```

安装 Xyce 后可直接求解：

```powershell
python tools/cctl_studio/cctl_studio.py run tools/cctl_studio/examples/rc_low_pass/project.json -o tools/cctl_studio/build/rc --xyce C:/path/to/Xyce.exe
```

求解波形由 `.PRINT TRAN FORMAT=CSV` 写入运行目录。

## 无代码增加元件

参照 `components/` 增加一个 JSON 文件，在工程的 `libraries` 数组中引用其相对路径，
或通过可重复的 `--library` 参数传入文件/目录。模板当前被限制为一行，允许使用：

- `$instance`：实例名；
- `$port_<端口名>`：连接网络；
- `$param_<参数名>`：实例参数或默认值。

例如电流源实例可以由模板
`$instance $port_p $port_n DC $param_current` 生成，通用程序无需知道“电流源”这一类型。

## 后续实施边界

1. 增加正式 JSON Schema，并让图形界面直接渲染相同元数据；
2. 支持层次模块、`.SUBCKT` 和厂商模型文件清单；
3. 定义带采样周期的控制/数字端口，建立确定性的 CCTL-Xyce 协同仿真桥；
4. 增加 PWM、ADC、编码器和记录探针适配器；
5. 在工程格式上层增加 OpenDSS 工况/工作点导入。

核心原则是让 JSON 成为稳定接口：未来的 SysConfig 风格界面只读写工程数据，不把
具体元件逻辑固化在 UI 程序中。
