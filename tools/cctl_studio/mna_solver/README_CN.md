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

```text
x_dot = A x + B u
y     = C x + D u + F u_dot
```

通常 `F=0`。保留 `F` 是为了正确表示一个特殊情况：当观测电容电流，且
电容电压被理想输入源直接约束时，输出会依赖输入导数。程序会显式保留该项，
不会悄悄给出错误的 `C/D` 方程。

## 依赖

- NumPy：数值降阶、离散迭代和频率响应；
- SymEngine：精确符号 MNA 矩阵。

两者都已经包含在 GMP Python 环境中，版本由
`tools/gmp_installer/requirements-gmp.txt` 统一固定，无需另建运行环境。

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
  tools/cctl_studio/mna_solver/tb/0_divider.CIR
```

为符号元件参数赋值：

```bat
python tools\cctl_studio\mna_solver\mna_solver.py analyze ^
  tools\cctl_studio\mna_solver\tb\example5.cir ^
  --param R1=1K --param R2=10K
```

列出前向欧拉离散矩阵以及逐项 `x[k+1]`、`y[k]` 迭代表达式：

```bat
python tools\cctl_studio\mna_solver\mna_solver.py discretize ^
  tools\cctl_studio\mna_solver\tb\example6.cir --dt 1N
```

使用前向欧拉法仿真并输出 CSV。独立源的网表数值是默认输入；源为符号值时
必须使用 `--input`：

```bat
python tools\cctl_studio\mna_solver\mna_solver.py simulate ^
  tools\cctl_studio\mna_solver\tb\example6.cir ^
  --dt 1N --duration 10U --input Vin=1 ^
  --output tools\cctl_studio\mna_solver\tb\example6_result.csv
```

计算每一组输入/输出之间的幅值、dB 幅值和相位：

```bat
python tools\cctl_studio\mna_solver\mna_solver.py frequency ^
  tools\cctl_studio\mna_solver\tb\example6.cir ^
  --start 10 --stop 1MEG --points 200 ^
  --output tools\cctl_studio\mna_solver\tb\example6_frequency.csv
```

Python API 的完整示例见 [README.md](README.md)。离散化接口已经保留
`method` 参数；当前实现 `forward_euler` 和适合刚性开关电路的
`backward_euler`，后续可以在不改变解析器、状态空间调用方的前提下加入
龙格-库塔等方法。

## 二极管/MOSFET 分段线性仿真

`switched_solver.py` 可以读取 TINA 续行和 `.MODEL` 参数。对于当前
`2_buck.CIR`，D1 的串联电阻、结电容来自 `RS/CJO`，T1 则近似为
`Ron/Roff/Coss`；MOS 门极上的电压源从电气 MNA 中移除，转换成外部布尔
PWM 命令。

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
  tools\cctl_studio\mna_solver\tb\2_buck.CIR --dt 1N
```

使用外部 10 kHz、50% 占空比方波仿真：

```bat
python tools\cctl_studio\mna_solver\switched_solver.py simulate ^
  tools\cctl_studio\mna_solver\tb\2_buck.CIR ^
  --dt 50N --transition-substep 500P --duration 50M ^
  --pwm-frequency 10K --duty 0.5 --output-stride 100 ^
  --output buck_10khz.csv
```

可以用重复的 `--device-param` 覆盖近似值，例如 `D1.Vf=0.7`、
`T1.Ron=50M`、`T1.Coss=1N`。开关仿真默认使用后向欧拉法，因为毫欧级
导通电阻和皮法级结电容构成刚性系统。
过渡子步只在两个二极管路径都未导通时使用，用于抑制宏步长造成的开关节点
过冲，因此 50 ms 长时间仿真仍然可以保持实用速度。

50 ms 参考仿真覆盖 5 个 `L/(R1+R2)` 时间常数和 500 个 PWM 周期；
`V(VF1)` 最终值及末段平均值均约为 2.25 V，已经进入理想半电源 2.5 V
附近。两者的差异来自模型中明确保留的体二极管压降和有限导通电阻。

当前网表中 T1 的 `D=4`、`S=3`，节点 3 是 +5 V 电源。因此门极关断后，
固有体二极管会从 S 向 D 向开关节点供电，参考仿真中 D1 始终未接管续流。
如果原意是由接地 D1 完成标准 Buck 续流，应再检查 MOS 漏源方向；求解器
不会为了得到预期波形而擅自反转器件。

## `1_OPAMP.CIR` 诊断结果

解析器已经把 `XIOP1 3 1 VF1 IdOpamp` 识别为常规 `(+、-、输出)` 顺序的
TINA 三引脚理想运放。但该网表的描述符矩阵束在所有频率下均为奇异：节点 3
通过 R2 接地，理想运放又强制节点 3 与节点 1 等电位，而 VG1 独立固定节点 1。
因此当前导出的网表本身没有可求解的低通传递函数。程序会明确报告奇异拓扑，
不会猜测引脚顺序或静默加入寄生参数；需要先修正原理图/网表连接。

## 当前边界

- 输入电路必须是线性时不变电路，或已经在外部完成线性化；
- 描述符降阶支持正则的 index-1 MNA 系统；浮空节点、冲突理想源和更高阶
  描述符拓扑会报告奇异错误；
- 初值目前使用降阶后的状态坐标；把物理电容电压、励磁电流映射为初值属于
  后续工作；
- C/C++ 迭代表达式与 Verilog 生成是下一层功能，本初版尚不输出代码。

验证命令：

```bat
tools\cctl_studio\mna_solver\run_tests.bat
```

该脚本只使用 `cmd.exe` 语法，成功和失败都会暂停，并执行全部单元测试及
命令行冒烟测试；自动化环境可使用 `run_tests.bat --no-pause`。
