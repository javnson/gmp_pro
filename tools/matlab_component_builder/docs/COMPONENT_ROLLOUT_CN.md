# GMP 元件接入清单

本清单记录 `matlab_component_builder` 对 GMP 控制元件的接入状态。`已编译` 表示当前 MATLAB Host MEX 已通过编译；它不代表硬件验证。

## 适配类别

| 类别 | 处理方式 |
| --- | --- |
| 静态/普通状态 SISO | `generic_stateful_v1`，继承采样时间，current/pending 状态提交。 |
| 多输入或多输出 | 通用标量多端口 S-Function；Mask 根据配置生成端口标签。 |
| 需要稳态的元件 | 分析配置记录 settling/measurement 周期；后续 MIMO FSRA 还需配置工作点和注入端口。 |
| 外部工作区 | current/pending 双缓冲；只在 `mdlUpdate` 交换，`mdlTerminate` 释放。 |
| 硬件接口对象 | 必须先区分“纯数值变换”和“指针/外设绑定”；后者不能伪装成普通 Host SISO 模块。 |

## 当前状态

| 目录 | 元件 | 状态 | 备注 |
| --- | --- | --- | --- |
| basic | hysteresis | 已编译 | 非线性、带锁存状态。 |
| basic | saturation | 已编译 | 支持上下限外部端口。 |
| basic | slope limiter | 已编译 | `fs` 只用于把物理斜率换算为逐拍增量。 |
| continuous | PI | 已编译 | 并联/时间常数初始化。 |
| continuous | PID | 已编译、已测量 | 并联/时间常数初始化。 |
| continuous | LADRC1 | 已编译 | 2 输入、2 输出，需要工作点分析。 |
| continuous | LADRC2 | 已编译 | 4 输入、3 输出，需要工作点分析。 |
| continuous | SOGI | 已编译 | 1 输入、direct/quadrature 双输出。 |
| discrete | PR/R/QR/QPR | 已编译、QPR 已测量 | 已有理论连续/离散模型。 |
| discrete | lead / lag | 已编译 | lead 支持三种初始化形式，设计参数可外引。 |
| discrete | biquad | 已编译、已测量 | LPF/HPF/BPF/notch/peaking/low-shelf/high-shelf。头文件声明但源码未实现的 all-pass 未注册。 |
| discrete | LP / IIR1 filter | 已编译 | 轻量 LP 与 LPF/HPF/APF/lag 四种 IIR1 初始化。 |
| discrete | 1P1Z / 2P2Z / 3P3Z | 已编译 | 覆盖实数根与 GMP 已提供的复共轭初始化形式。 |
| advance | repetitive controller | 已编译、已测量 | 双缓冲延迟线；固定锁频，先学习后测量，并与差分方程频响对照。 |
| advance | FDRC | 已编译 | 双缓冲延迟线和内部 biquad Q 滤波器，采用相同固定锁频测量流程。 |

## 后续接入队列

| 批次 | 元件 |
| --- | --- |
| Discrete A | DF11/DF22/DF13/DF23、discrete PID。 |
| Discrete B | discrete SOGI/SOGI-DC。 |
| Advanced | back stepping、MRAC、SMC。 |
| Interface 数值层 | ADC/DAC 数值换算、PWM channel、调制器。 |
| Interface 绑定层 | ptr ADC、advanced ADC/PWM、标准接口对象；需要明确 Host 端口代替硬件指针的契约。 |

`ctl_adv_rc.c` 同时定义 `ctl_init_rc` 和 `ctl_init_fdrc`，因此单独链接 RC 时也必须带入 biquad 实现。Builder 配置已经显式记录该依赖；不要把生成目录中的链接结果当成源码修复。
