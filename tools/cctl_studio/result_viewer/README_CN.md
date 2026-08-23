# GMP CCTL Simulation Viewer Manager

此工具用于查看 CCTL 仿真生成的超大 CSV/TSV 数据文件，避免先把几千万行数据载入 Excel 再绘图。

## 启动

先使用 GMP 私有环境安装程序完成安装并配置 `GMP_PRO_LOCATION`，然后双击
`run_result_viewer.bat`。该脚本会通过环境守卫检查安装状态，并显式使用
`%GMP_PRO_LOCATION%\bin\python\python.exe`，不会意外调用系统 Python。也可把结果文件作为参数传入：

```bat
run_result_viewer.bat E:\path\to\mcs_pmsm_nt_cctl.csv
```

也可以一次加载多个不同采样率的文件，并立即进入 20 Hz 动态刷新：

```bat
run_result_viewer.bat --live E:\result\drive_circuit.csv E:\result\drive_control.csv
```

Y 数据使用树形结构整理：每个 CSV/TSV 文件是一个可展开的顶层节点，文件中的
信号是子节点，界面只显示清晰的原始列名，不再平铺 `文件名 :: 列名`。内部仍以
文件和列的组合键区分不同文件中的同名信号。每条曲线始终绑定所属文件自己的
`time_s`，因此高速电路采样和控制中断采样可画在同一图中并按仿真时间对齐；
不会按行号拼接，也不会隐式插值。树中的叶节点支持跨文件多选和双击添加，文件
节点本身不会被误当成曲线。

打开文件后选择 X 轴和一到多列 Y 数据，再点击 `Add selected curves`。Viewer
采用多页面结构：第一页 `Configuration` 只管理显示和仿真参数，其余
`Waveforms` 页面用于绘图。`Data and curves` 面板只在波形页面左侧显示，因此
选择数据、添加或删除曲线时不必离开图像。点开菜单栏的 `Layout` 后会显示一个
4 列×6 行的方格选择器；移动鼠标会高亮从左上角到当前位置的目标区域，点击即可
选择从 1×1 到 6×4 的任意布局。选择后会立即创建对应数量的图，而不只是改变容量；
所有图采用 MATLAB tiled-layout 风格等分并铺满波形页，单页最多 24 张图。
缩小布局时优先保留左上方的图及曲线，超出新布局的图会被移除，并在状态栏报告
被移除的曲线数量。`Waveforms` 菜单用于增删页面和图表。每张图分别保存自己的
曲线，蓝色边框表示当前图。X/Y 轴链接只作用于同一波形页，并且两个方向可以
独立启用。

也可以直接双击 Y 数据列，把该曲线加入当前蓝框图。左侧 `Curves in active plot`
只显示当前图已有的曲线，选中后按 `Delete` 或点击删除按钮即可移除。修改页面的
行列布局即可统一调整图表尺寸；双击图内标题可直接重命名。

顶部工具栏提供四种 MATLAB 风格交互：`Pan` 双轴平移，`Horizontal zoom` 只改变 X 范围，`Vertical zoom` 只改变 Y 范围，`Magnifier` 框选同时放大两轴。`Fit active` 复位当前图，`Fit all` 复位全部图。X、Y 链接完全独立：仅勾选 `Link X` 时各图只共享横轴，纵轴仍分别缩放；仅勾选 `Link Y` 时行为相反。

程序不会把整个超大文件保存在内存中。`Memory time window` 决定每个文件最多保留
多少秒的数据，默认 1 s。勾选 `Load newest time segment` 时读取文件的最新时间段；
取消后可填写 `Fixed segment start`，再用 `Reload selected time segment` 加载指定
起点的一段数据。文件仍以流式方式扫描，因此文件大小只影响扫描时间，不会让
驻留内存随历史长度持续增加。超过显示点数上限时使用分桶最小值/最大值降采样，
可保留开关尖峰。

仿真仍在写文件时也可以直接打开结果。勾选 `Dynamic refresh (20 Hz)` 后，查看器每 50 ms 只读取文件新增的完整行，不会反复扫描整个大文件。如果写入线程暂时只写出了半行，该行会延迟到后续写完整后再解析；若文件末尾确实存在格式异常的最后一行，则忽略该行并正常显示此前所有完整数据。文件被截断并重新开始写入时，动态读取器会自动从新表头之后重新加载。

Viewer 默认启用 `Rolling X window`，只显示最新 0.1 s，并随着新数据向后滚动，
形成示波器式显示。该功能与 `Link X` 兼容；取消勾选会恢复当前内存时间段的
自适应显示。命令行也可使用 `--rolling-window 0.1`。滚动窗口仅决定可见范围，
内存窗口决定驻留数据量，两者互不替代，也都不会删除 CSV 历史数据。

`Auto-fit visible data` 默认启用。Viewer 会直接从缓存中筛选当前可见 X 时间窗
内的采样，计算所有已选曲线的有限 Y 最小值/最大值并增加 5% 边距；动态刷新、
滚动或手工改变 X 范围时都会重新计算。启用 `Link Y` 时，同一页采用所有图的
联合范围。关闭 Auto-fit 后会严格保留用户手动设置的纵轴范围。

推荐的 PMSM 重复仿真流程也可以直接给仿真器传入 `--viewer`；CCTL CSP 会启动
GMP 私有 Python 的 Viewer，加载全部输出 CSV 并勾选动态刷新。手动流程仍可用：
先启动 Viewer 并打开上一次生成的 CSV，选择曲线并勾选动态刷新，再重新运行
仿真。文件被截断重写后，Viewer 会清除旧采样并从新表头继续刷新。

当前面向纯数值 CSV、TSV 或分号分隔文本；`.xlsx` 并不适合作为数千万采样点的仿真交换格式。

`Configuration` 页可以设置原生 CCTL 仿真器、输出基路径、目标时长和显示参数。
开始、暂停、继续、停止按钮、总进度和运行指标固定在整个窗口最下面的一行，切换
波形页时仍然可见。Viewer 使用 `QProcess` 管理子进程，控制与状态通过 JSON
交换；用户程序及 GMP 的普通输出继续写到启动 Viewer 的控制台，不占用绘图区。
仿真器公布的多个输出文件会自动载入并以 20 Hz 刷新。直接双击或无参数启动
CCTL 仿真程序时，程序会先打开该 Manager，再由 Manager 以受管模式重新启动
仿真器。Viewer 初始显示 `Stop`，不会自动开始计算；用户点击一次 `Start` 后才
启动受管仿真。为了让用户在启动前配置图表，Viewer 会先把受管仿真器初始化到
等待状态；此时 CSP 已经创建所有 CSV 并写入表头，`Data and curves` 会立即列出
可选通道，但 CSV 中还没有任何数值数据行。纯命令行运行可传 `--headless`。

## PIL Server 与在线调试

`PIL Server` 页直接复用 GMP Data Link Debugger 的协议引擎和功能页，包括 Raw、
Echo、PIL Simulation、Simulink-PIL Bridge、Tunable、Memory、Chronos 和 Data Link
Scope。它不打开物理串口：Debugger 生成的标准 Data Link 原始帧经受管 JSON 通道
Base64 编码后送入 CCTL CSP 的虚拟通信外设；目标程序的 `user_main.c` 仍通过
`gmp_dev_dl_loop_cb()`、设施分派器和标准 CRC/转义实现处理请求，回复再沿同一路径
返回。暂停状态也可执行设施发现、参数读写和 PIL 单步，不需要先启动数值仿真。

Viewer 只显示目标实际注册的设施。当前 `mcs_pmsm_nt/project/cctl` 注册 Echo、PIL、
Tunable 和白名单 Memory；未注册的 Chronos/Scope 页会按标准协议报告不可用。PIL
命令基址来自工程 SDPE 的 `GMP_PIL_DL_BASE_COMMAND`（当前为 `0x10`），而
`ENABLE_GMP_DL_PIL_SERVER` 只开放在线服务，不会启用硬件工程所用的 PIL-only
控制模式，也不会改变正常 CCTL 闭环的 ADC ISR/PWM 路径。
