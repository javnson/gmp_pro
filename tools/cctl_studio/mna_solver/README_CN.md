# CCTL Studio MNA 电路求解器

[English](README.md)

这是 CCTL Studio 线性电路求解器的第一版 Python 实现，处理流程为：

```text
网表 -> 精确符号 MNA -> 数值描述符 MNA
     -> 代数量消元 -> 状态/输出方程
     -> 离散化 -> 时域仿真或频率响应
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
`method` 参数；当前实现 `forward_euler`，后续可以在不改变解析器、状态空间
调用方的前提下加入龙格-库塔等方法。

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
