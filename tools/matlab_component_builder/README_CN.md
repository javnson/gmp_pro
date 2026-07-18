# GMP MATLAB 元件构建器

[English](README.md) | **简体中文**

`matlab_component_builder` 用于把真实的 GMP C 控制元件转换成独立安装、带 Mask 的 Simulink Block。它不依赖、也不修改现有的 GMP `slib`。

当前已接入 basic 的滞环、饱和与斜率限制，continuous 的 PI/PID、LADRC1/LADRC2、SOGI，PR 控制器族，lead/lag、biquad、一级离散滤波器、1P1Z/2P2Z/3P3Z，以及 advance 的 RC/FDRC。完整接入队列见 [元件接入清单](docs/COMPONENT_ROLLOUT_CN.md)。

## 源文件与生成物边界

| 路径 | 作用 |
| --- | --- |
| `components/*.json` | 用户维护的权威元件定义。 |
| `schemas/` | 配置格式说明。 |
| `templates/` | 代码生成模板。 |
| `python/gmp_mcb/` | 校验、生成器和 PyQt 编辑器。 |
| `matlab/+gmp_mcb/` | MATLAB 编译、建库、Mask、模型和测量程序。 |
| `build/` | 生成的 C++ 源码和注册表，不提交。 |
| `install/<Release>/` | 生成的 MEX 和 Simulink Library，不提交。 |
| `cache/` | 频响测量结果，不提交。 |

JSON 中的 GMP 头文件和源文件位置必须相对 `GMP_PRO_LOCATION`。本地生成物可以包含解析后的绝对路径，但绝对路径不会成为权威输入。

## 快速使用

首先安装或激活 GMP，确保已经定义 `GMP_PRO_LOCATION`。

启动编辑器：

```bat
tools\matlab_component_builder\run_builder.bat
```

也可以使用命令行：

```bat
cd /d %GMP_PRO_LOCATION%\tools\matlab_component_builder
python matlab_component_builder.py validate
python matlab_component_builder.py generate
```

为当前 MATLAB Release 编译并注册：

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), 'tools', ...
    'matlab_component_builder', 'matlab', ...
    'install_gmp_matlab_components.m'));
```

MATLAB 安装脚本会自动调用 GMP 环境守卫：如果存在完整的
`bin/gmp_virtual_env_installed.flag`，使用 `bin/python/python.exe`；否则使用
`install_gmp.bat` 配置的 system Python。无需从已经激活的命令行启动 MATLAB。
如果所选环境缺少 Jinja2，安装器会报告实际 Python 路径，并要求通过对应的
GMP 安装入口修复环境，而不会在 MATLAB 中临时执行 `pip install`。

编辑器左侧按分类显示全部元件，右侧可表格化维护多输入/多输出端口、参数分组、外部输入资格，并预览生成的 C++ S-Function。刷新 MATLAB 后可以在 Simulink Library Browser 中找到 **GMP MATLAB Components**。

每个 Block 的 Mask 分为 **Parameters** 和 **Simulation Analysis** 两页。可外部化的参数旁有 Check Box：关闭时使用 Mask 固定值，开启时固定值编辑框禁用，并按参数表顺序增加输入端口。PID 的 Kp/Ki/Kd、输出限幅和积分限幅均支持这种增益调度接口；初始化频率 `fs` 保持为初始化参数。

移除注册路径：

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), 'tools', ...
    'matlab_component_builder', 'matlab', ...
    'uninstall_gmp_matlab_components.m'));
```

卸载只移除 MATLAB 路径，不删除生成物。如需完全重建，可手动删除被忽略的 `build/`、`install/` 和 `cache/`。

## 调度频率与 `fs`

生成的 S-Function 使用继承采样时间。Mask 中的 `fs` 只传递给 GMP 初始化函数，不强制 Simulink 的调度周期。因此用户可以把控制器放进 Triggered Subsystem 或 Function-Call Subsystem，使控制器以正常控制频率运行，而外部对象使用更精细的仿真步长。

**Simulation Analysis** 页另有独立的执行频率、扫频范围、点数、激励幅值和测量周期。理论离散模型和测量 testbench 用分析执行频率把离散拍数映射为真实时间；该值不传入控制器，也不改变 Block 的调度。如果实际触发频率与初始化参数 `fs` 不同，应在这里填写真实执行频率。

## PID 模型定义

并联初始化对应的当前实际差分频响为：

```text
Kp + (Ki/fs_parameter)/(1-z^-1) + (Kd*fs_parameter)(1-z^-1)
```

其中：

```text
z^-1 = exp(-j*2*pi*f/fs_execution)
```

时间常数模式严格跟随当前 `ctl_step_pid_ser` 实现。当前代码的微分项没有再次乘以 `Kp`，分析器会忠实显示这一行为，而不会用理想串联 PID 掩盖它。

所有 SISO 元件都保留 **Measure and plot simulated frequency response**。测量按钮会创建临时离散 Simulink testbench，复制当前 Masked Block，并运行相干逐频正弦激励。有解析模型时，图中叠加连续理论、离散实现和 MEX 实测曲线；没有解析模型时，只绘制实际测得的幅频和相频曲线。MAT 结果保存到 `cache/`。

对于饱和、滞环等非线性模块，测得的频响取决于激励幅值、偏置和稳态条件，不应解释为唯一的线性传递函数。测量图标题和缓存会记录激励幅值、执行频率与稳态/测量周期。

对于 LADRC 等多输入/多输出时域模块，**Simulation Analysis** 页会提供激励输入端口、观测输出端口和每个输入的稳态工作值。测量程序只在所选输入上叠加正弦激励，其余输入保持指定工作值，并绘制所选输入到所选输出的局部幅相响应。

RC/FDRC 在整个扫频过程中保持固定的 **Locked frequency**。每个频点先持续施加当前误差正弦，按锁定频率计算并运行指定数量的预学习周期，随后进入独立的相干测量窗口。扫频激励频率不会反向修改控制器的锁定频率。

## 当前范围

- SISO 和标量多输入/多输出；Simulink 端口为 `double`，GMP 内部使用 `ctrl_gt`。
- PID 模板（并联参数和时间常数初始化）。
- R、PR、QR、QPR 谐振模板（QR/QPR 支持标准与预扭曲 Tustin 初始化）。
- 参数分组，以及固定 Mask 值/动态输入端口选择。
- 动态工作区 current/pending 双缓冲，在 `mdlTerminate` 中统一释放。
- 继承 Simulink 调度周期。
- 当前 MATLAB 平台上的 Host MEX 仿真。
- 连续理想/离散实现绘图与逐频正弦测量，支持 RC 固定锁频预学习。

MIMO、通用状态观察端口、定点 MEX、Simulink Coder 部署和独立边界 testbench 尚未实现。Host MEX 通过不等同于硬件验证通过。

## 验证

Python 测试：

```bat
python -m unittest discover -s tests -v
```

MATLAB 验证需要已经配置 MEX 编译器。当前框架已在 MATLAB R2024b、Microsoft Visual C++ 2022 下完成 22 个 MEX 的编译、Library/双页 Mask 生成、PID 首拍与外部参数端口检查，以及 QPR、RC 和 biquad 的实测频响检查。
