# CCTL Studio MNA 电路求解器

[English](README.md)

这是 CCTL Studio 线性电路求解器的第一版 Python 实现，处理流程为：

```text
网表 -> 精确符号 MNA -> 数值描述符 MNA
     -> 代数量消元 -> 状态/输出方程
     -> 离散化 -> 时域仿真或频率响应

TINA D/M 网表 -> 4 个主开关拓扑 + 2 个体二极管拓扑
              -> 根据上一采样端电压选模 -> 开关仿真
```

MNA 首先写成描述符系统：

```text
E z_dot = A z + B u
```

`z` 包含所有非接地节点电压和 MNA 引入的支路电流。程序采用 SVD
秩分解并对 index-1 系统执行代数消元，得到：
$$
\dot{\bf x} = \bf A \bf x +\bf B \bf u \\
\bf y = \bf C \bf \bf u + F \dot{\bf u}
$$
通常 `F=0`。保留 `F` 是为了正确表示一个特殊情况：当观测电容电流，且
电容电压被理想输入源直接约束时，输出会依赖输入导数。程序会显式保留该项，
不会悄悄给出错误的 `C/D` 方程。

## 依赖

- NumPy：数值降阶、离散迭代和频率响应；
- SymEngine：精确符号 MNA 矩阵；
- Eigen3：生成的固定维 C++ 矩阵计算。

Python 包由 `tools/gmp_installer/requirements-gmp.txt` 统一固定。Eigen3
由本目录的 `vcpkg.json` 声明，并由 GMP 安装器恢复到共享 vcpkg 安装树；
不再使用已经弃用的 `third_party/eigen`。

## 支持的网表范围

| 元件 | 语法 | 含义 |
| --- | --- | --- |
| `R`、`L`、`C` | `R1 n+ n- value` | 无源元件 |
| `V`、`I` | `V1 n+ n- value` | 独立输入源 |
| `O` | `O1 n+ n- nout` | 理想运放 |
| `E`、`G` | `E1 n+ n- nc+ nc- gain` | VCVS / VCCS |
| `F`、`H` | `F1 n+ n- Vctrl gain` | CCCS / CCVS |
| TINA 电流箭头 | `VAM1 n+ n- ; Current Arrow` | 零压降电流探针 |
| `D` | `D1 anode cathode model` | 开关求解器中的分段线性二极管 |
| `M` | `M1 drain gate source bulk model` | 开关求解器中的分段线性 NMOS |
| TINA 理想运放 | `X1 n+ n- nout IdOpamp` | 三引脚理想运放 |

节点可以是数字或文本；`0` 和 `GND` 表示地。支持 `K`、`MEG`、`M`、
`U`、`N` 等 SPICE 数值后缀。`Symbolic` 或其他非数值参数在 SymEngine
中用元件名表示；进入数值降阶前用 `--param` 赋值。

程序忽略 `.LIB`、`.TEMP`、`.AC`、`.TRAN` 及其他无关分析指令；解析
`.PROBE`、`.PRINT`、`.SAVE` 中的 `V(node)`、`V(node+,node-)` 和
`I(element)`。网表未注册观测量时，默认输出全部非接地节点电压。

## 命令行用法

以下命令均从 GMP 仓库根目录运行。

显示精确符号 MNA 矩阵以及数值状态/输出矩阵：

```bat
python tools\cctl_studio\mna_solver\mna_solver.py analyze ^
  tools/cctl_studio/mna_solver/tb/basic/0_divider.CIR
```

为符号元件参数赋值：

```bat
python tools\cctl_studio\mna_solver\mna_solver.py analyze ^
  tools\cctl_studio\mna_solver\tb\basic\example5.cir ^
  --param R1=1K --param R2=10K
```

列出前向欧拉离散矩阵以及逐项 `x[k+1]`、`y[k]` 迭代表达式：

```bat
python tools\cctl_studio\mna_solver\mna_solver.py discretize ^
  tools\cctl_studio\mna_solver\tb\basic\example6.cir --dt 1N
```

使用前向欧拉法仿真并输出 CSV。独立源的网表数值是默认输入；源为符号值时
必须使用 `--input`：

```bat
python tools\cctl_studio\mna_solver\mna_solver.py simulate ^
  tools\cctl_studio\mna_solver\tb\basic\example6.cir ^
  --dt 1N --duration 10U --input Vin=1 ^
  --output tools\cctl_studio\mna_solver\tb\basic\example6_result.csv
```

计算每一组输入/输出之间的幅值、dB 幅值和相位：

```bat
python tools\cctl_studio\mna_solver\mna_solver.py frequency ^
  tools\cctl_studio\mna_solver\tb\basic\example6.cir ^
  --start 10 --stop 1MEG --points 200 ^
  --output tools\cctl_studio\mna_solver\tb\basic\example6_frequency.csv
```

Python API 的完整示例见 [README.md](README.md)。离散化接口已经保留
`method` 参数；当前实现 `forward_euler` 和适合刚性开关电路的
`backward_euler`，后续可以在不改变解析器、状态空间调用方的前提下加入
龙格-库塔等方法。

## 二极管/MOSFET 分段线性仿真

`switched_solver.py` 可以读取 TINA 续行和 `.MODEL` 参数。对于当前
`tb\buck\buck.CIR`。D1 的 `Vf/Ron` 来自 `VJ/RS`，固定结电容使用
`CJO/VJ/M/FC` 的 SPICE 耗尽层电容公式在指定偏压处线性化（默认偏压为 0，
因此等于 CJO）。T1 的 `Ron` 使用
`RD+RS+1/(KP*(W/L)*(Vdrive-VTO))`，`Roff=RDS`，`Coss=CBD+CGDO`，
体二极管采用 `Vf=PB`、`Ron=RD+RS`。MOS 门极电压源从电气 MNA 中移除，
转换成外部 `uint32_t PWM` 命令。

用户要求的 4 个主方程是：

```text
D1 关 + MOS 沟道关       D1 关 + MOS 沟道开
D1 开 + MOS 沟道关       D1 开 + MOS 沟道开
```

严格保留体二极管时，门极关断的 MOS 还存在源极到漏极的体二极管路径，
因此程序额外缓存 D1 开/关对应的 2 个体二极管方程。每个采样点根据上一状态的
`V(D1.A)-V(D1.K)` 和 `V(T1.S)-V(T1.D)` 选择下一拓扑。默认输出包含
`V(T1.D)`、`V(T1.S)`、`V(D1.A)` 和 `V(D1.K)`。

列出全部拓扑矩阵和逐项离散方程：

```bat
python tools\cctl_studio\mna_solver\switched_solver.py analyze ^
  tools\cctl_studio\mna_solver\tb\buck\buck.CIR --dt 1N
```

使用外部 10 kHz、50% 占空比方波仿真：

```bat
python tools\cctl_studio\mna_solver\switched_solver.py simulate ^
  tools\cctl_studio\mna_solver\tb\buck\buck.CIR ^
  --dt 50N --transition-substep 500P --duration 50M ^
  --pwm-frequency 10K --duty 0.5 --output-stride 100 ^
  --output buck_10khz.csv
```

可以用重复的 `--device-param` 覆盖近似值，例如 `D1.Vf=0.7`、
`T1.Ron=50M`、`T1.Coss=1N`。开关仿真默认使用后向欧拉法，因为毫欧级
导通电阻和皮法级结电容构成刚性系统。
过渡子步只在两个二极管路径都未导通时使用，用于抑制宏步长造成的开关节点
过冲，因此 50 ms 长时间仿真仍然可以保持实用速度。

50 ms 参数模型参考仿真覆盖 5 个 `L/(R1+R2)` 时间常数和 500 个 PWM
周期。`R1+R2` 只有 0.1 Ω，而从模型提取的 T1 导通电阻约 0.1979 Ω，
因此 `V(VF1)` 末段平均值约 0.78865 V。这是重载下的模型导通损耗，不是
数值发散。使用 `T1.Ron=1M T1.BodyRon=2M` 的理想开关覆盖值时，参考测试
仍会稳定在约 2.25 V，即接近 50% 占空比的理想半电源值。

当前网表中 T1 的 `D=4`、`S=3`，节点 3 是 +5 V 电源。因此门极关断后，
固有体二极管会从 S 向 D 向开关节点供电，参考仿真中 D1 始终未接管续流。
如果原意是由接地 D1 完成标准 Buck 续流，应再检查 MOS 漏源方向；求解器
不会为了得到预期波形而擅自反转器件。

## 电路数据文件与 Eigen C++ 代码生成

`circuit_data.py` 把 CIR/MNA 结果固化成带版本的 JSON。文件包含外部端口、
探针、原始模型参数和提取依据、状态/信号名称、连续矩阵、正常与短步长离散
矩阵，以及上一采样端电压选模所需索引。单二极管加单 MOS 电路包含 6 个
拓扑；FSBB 的四个 MOS 每个具有关断、沟道和体二极管三条路径，因此包含
`3^4=81` 个拓扑；三相逆变器的六个 MOS 则包含 `3^6=729` 个拓扑。JSON
可以由 `CircuitDataSimulator` 直接仿真，也是 C++
代码生成器唯一的输入。

独立二极管和 TINA `VSWITCH` 均按通断两条路径展开，因此整流器包含
`2^(4+1)=32` 个拓扑。`VSWITCH` 的控制电压源会从 MNA 中移除，并转换为
外部 `uint32_t` 控制端口。

单相逆变器案例使用 30 V 直流母线、10 kHz 中心对齐双极性 SPWM、50 Hz
正弦参考、0.8 调制度和 1 us 死区。100 ms C++ 参考仿真中，差分负载电压
`V(2,1)` 的基波峰值约 22.031 V、有效值约 15.610 V，50 Hz 电流基波峰值
约 2.203 A。

可切除预充电电阻的桥式整流案例使用 32 Vrms、50 Hz 输入。保留 10 Ω 充电
电阻时，`V(VF1)` 平均约 14.490 V；60 ms 时闭合 `SW2` 后，稳态全波输出
平均约 31.088 V，峰值约 44.133 V。

三相逆变器案例使用 5 V 直流母线、10 kHz 中心对齐 SPWM、相差 120° 的
50 Hz 三相正弦参考、0.8 调制度和 1 us 死区。三相电压基波峰值为
1.8566--1.8571 V，三相电流基波峰值为 0.18568--0.18573 A；三组相电压
相量两两夹角余弦与 `-0.5` 的偏差小于 0.00025，符合 120° 相位关系。

NPC(I) Buck 案例使用两个 30 V 电源、10 kHz 载波和 1 us 死区。0.2 pu
参考只在 N/O 两个电平间切换，稳态输出约 11.050 V；0.8 pu 参考只在 O/P
两个电平间切换，稳态输出约 45.453 V。它们是带载开环结果，包含模型提取的
半导体导通损耗、50 mΩ 电源串联电阻和死区损耗，因此低于理想 12/48 V。

混合器件电路按各器件路径的笛卡尔积展开。NPC Buck 含四个三路径 MOSFET
和两个两路径钳位二极管，因此导出 `3^4*2^2=324` 组兼容状态/输出矩阵；
二极管 A/K 与 MOSFET D/S 电压保留为内部选模信号。

### 测试用例目录约定

全部测试电路位于 `tb` 下。不含开关的基础用例集中在 `tb\basic`，以下脚本
会编译 Python 求解器，依次构造全部基础网表的符号/数值模型并执行暂态仿真，
同时检查分压器和运放低通的已知结果：

```bat
tools\cctl_studio\mna_solver\tb\basic\run_tests.bat
```

已经验证的开关案例统一采用以下结构：

```text
tb\buck\
├── buck.CIR
├── generate_code.bat
├── build_test.bat
├── generated\              自动生成的电路 JSON 和计算类头文件
└── test\cpp\               手写 testbench、CMakeLists.txt 和 vcpkg.json
```

`boost`、`fsbb`、`sinv`、`rectifier`、`inv` 和 `buck_npc` 遵守同一约定。

所有 BAT 入口都通过 `GMP_PRO_LOCATION` 找到求解器和安装环境，不依赖当前
工作目录或仓库所在盘符。用户直接运行时成功和失败都会暂停；自动化调用可
传入 `--no-pause`。

每个开关案例把 JSON 和计算类都写入 `generated`。JSON、CSV、本地构建目录
和 IDE 缓存由 `tb\.gitignore` 忽略，因为它们可重复生成，且 JSON 可能很大。

### 生成和构建

只生成 Buck 数据和计算类：

```bat
tools\cctl_studio\mna_solver\tb\buck\generate_code.bat
```

`generate_code.bat` 开头定义 `NETLIST_FILE`，所以不带参数运行时会使用案例
默认 CIR；也可以把其他 CIR 作为第一个参数传入。生成器只写计算类，不会
覆盖手写 testbench。生成的 `BuckCircuit` 使用 Eigen 固定维矩阵，提供
`step_short(PWM, VS1)`、`step_normal(PWM, VS1)`、`run` 和 `operator()`；
探针既可通过 `circuit.output.VF1`，也可通过 `circuit["V(VF1)"]` 读取。

生成、编译并运行已验证案例的手写 C++ 测试：

```bat
repair_gmp_vcpkg.bat
tools\cctl_studio\mna_solver\tb\buck\build_test.bat
tools\cctl_studio\mna_solver\tb\boost\build_test.bat
tools\cctl_studio\mna_solver\tb\fsbb\build_test.bat
tools\cctl_studio\mna_solver\tb\sinv\build_test.bat
tools\cctl_studio\mna_solver\tb\rectifier\build_test.bat
tools\cctl_studio\mna_solver\tb\inv\build_test.bat
tools\cctl_studio\mna_solver\tb\buck_npc\build_test.bat
```

SINV testbench 令 `PWM1=PWM3`、`PWM2=PWM4`，作为两组双极性 SPWM 对角
开关信号，并检查同一桥臂不重叠以及每个载波周期确实包含 1 us 双关断区。
TINA 的图形化电压表只保存在二进制 TSC 中，CIR 并未导出，因此可移植网表
显式注册 `V(2,1)` 作为负载两端电势差。直流母线的两个 100 uF 电容会在
MNA 中自然相加；回归测试进一步逐项验证了全部 81 个拓扑的描述矩阵与单个
200 uF 电容完全相同。

INV testbench 使用相差 120° 的三路正弦参考驱动三个互补桥臂，并检查上下管
互锁、1 us 死区、三相电压与电流基波平衡、相位关系以及
`V(3,1)+V(4,1)+V(5,1)` 是否抵消。除并联在三个 Y 接滤波电容上的 1 MΩ
泄放电阻 R8--R10 外，新网表还通过 1 MΩ 的 R12 和 R11 分别把负载星点 1、
电容星点 2 显式参考到地。六管全关断拓扑的代数块因此满秩，全部 729 个
MNA 拓扑都能直接降阶，不需要求解器隐式添加参考支路。

整流器把 TINA 控制源 `VSWGPIO1` 映射为公开的 `uint32_t SWGPIO1` 端口。
`SWGPIO1=0` 时保留 10 Ω 充电电阻，`SWGPIO1=1` 时通过等效 1 mΩ 路径闭合
`SW2`。生成类根据 D1--D4 上一采样的 A--K 电压选择两组桥臂导通组合；手写
testbench 输入 32 Vrms、50 Hz 正弦波，并在 60 ms 后切除充电电阻。

四个二极管提取出的 460 pF 结电容都接到了理想时变电压源 `VS1` 的端子。
保留它们会引入输入导数 `u_dot` 和 index-2 描述系统，无法纳入当前普通
`x_dot=Ax+Bu` 数据边界。JSON 会保留提取值和抑制原因，但本案例的状态矩阵
暂不加入这些结电容；以后增加描述系统/输入导数后端后可以恢复。

NPC Buck testbench 令 `VS1=VS2=30 V`，因此完整 60 V 母线定义为 1 pu。
0.2 pu 时保持 PWM3 导通，由 PWM2/PWM4 换流选择 N/O；0.8 pu 时保持 PWM2
导通，由 PWM1/PWM3 换流选择 O/P；两组互补信号均检查 1 us 死区。该电路的
Coss 系统刚性较强，案例使用 1 ns 正常步长和 100 ps 启动步长，并分别输出
0.2/0.8 pu CSV。新版网表中的 50 mΩ `R3`、`R4` 显式解除理想电源割集，
全部 324 个拓扑均可直接降阶，不需要求解器暗中添加正则化支路。该串联供电
中点并非由理想源对地刚性钳位，因此两个钳位二极管提取出的 460 pF 结电容
均保留在七状态模型中。

`build_test.bat` 首先刷新计算头文件，再从 `test\cpp` 配置 CMake。构建脚本
使用 GMP 安装的 Eigen/vcpkg；私有环境下关闭单项目 manifest 自动恢复，
避免测试案例意外修改共享安装树。

## `1_OPAMP.CIR` 验证结果

修正后的 `XIOP1 4 1 VF1 IdOpamp` 可正常降为一阶反相低通：直流增益
`-1`，极点为 `1000 rad/s`（截止频率约 159.155 Hz）。自动测试同时验证了
DC 传递函数和状态矩阵 `A=-1000`。

## 当前边界

- 输入电路必须是线性时不变电路，或已经在外部完成线性化；
- 描述符降阶支持正则的 index-1 MNA 系统；浮空节点、冲突理想源和更高阶
  描述符拓扑会报告奇异错误；
- 初值目前使用降阶后的状态坐标；把物理电容电压、励磁电流映射为初值属于
  后续工作；
- 已支持不含独立二极管的多 MOS 电路，拓扑数量按 `3^N` 增长；纯二极管/
  VSWITCH 网络按 `2^N` 增长；同时含 MOSFET 与多个独立二极管或 VSWITCH
  的通用组合展开仍待实现；
- 将同一数据文件后端扩展为 Verilog/定点矩阵实现属于下一阶段。

验证命令：

```bat
tools\cctl_studio\mna_solver\tests\run_tests.bat
```

该脚本只使用 `cmd.exe` 语法，成功和失败都会暂停，并执行全部单元测试及
Basic 验收、命令行冒烟测试及 Buck/Boost/FSBB/SINV/Rectifier C++ 测试；自动化环境可使用
`tests\run_tests.bat --no-pause`。
