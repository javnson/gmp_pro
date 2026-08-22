# GMP CCTL Result Viewer

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

多文件模式用 `文件名 :: 列名` 区分同名列。每条曲线始终绑定所属文件自己的
`time_s`，因此高速电路采样和控制中断采样可画在同一图中并按仿真时间对齐；
不会按行号拼接，也不会隐式插值。

打开文件后选择 X 轴和一到多列 Y 数据，再点击 `Add selected curves`。可创建多张图；蓝色边框表示当前图，每张图分别保存自己的曲线。`Link X zoom` 用于时间轴对齐缩放，`Link Y zoom` 同时统一纵轴缩放。

也可以直接双击 Y 数据列，把该曲线加入当前蓝框图。左侧 `Curves in active plot` 只显示当前图已有的曲线，选中后按 `Delete` 或点击删除按钮即可移除。图与图之间的分隔条可以上下拖动，以独立调整每张图的高度；双击图内标题可直接重命名，不再需要额外的标题文本框。

顶部工具栏提供四种 MATLAB 风格交互：`Pan` 双轴平移，`Horizontal zoom` 只改变 X 范围，`Vertical zoom` 只改变 Y 范围，`Magnifier` 框选同时放大两轴。`Fit active` 复位当前图，`Fit all` 复位全部图。X、Y 链接完全独立：仅勾选 `Link X` 时各图只共享横轴，纵轴仍分别缩放；仅勾选 `Link Y` 时行为相反。

程序只在后台读取被选择的列，并缓存已加载列。数据量超过显示上限时使用分桶最小值/最大值降采样，因此开关尖峰不会因简单抽点而消失。修改 `Maximum display points / curve` 后重新添加曲线即可改变显示精度。

仿真仍在写文件时也可以直接打开结果。勾选 `Dynamic refresh (20 Hz)` 后，查看器每 50 ms 只读取文件新增的完整行，不会反复扫描整个大文件。如果写入线程暂时只写出了半行，该行会延迟到后续写完整后再解析；若文件末尾确实存在格式异常的最后一行，则忽略该行并正常显示此前所有完整数据。文件被截断并重新开始写入时，动态读取器会自动从新表头之后重新加载。

勾选 `Rolling X window` 并设置秒数后，每张图的水平轴固定为从“最新完整采样
时刻减去窗口长度”到“最新完整采样时刻”，形成示波器式滚动显示。该功能与
`Link X` 兼容；取消勾选会恢复完整数据自适应显示。命令行也可使用
`--rolling-window 0.1`。滚动只改变显示范围，不会删除 CSV 历史数据。

推荐的 PMSM 重复仿真流程也可以直接给仿真器传入 `--viewer`；CCTL CSP 会启动
GMP 私有 Python 的 Viewer，加载全部输出 CSV 并勾选动态刷新。手动流程仍可用：
先启动 Viewer 并打开上一次生成的 CSV，选择曲线并勾选动态刷新，再重新运行
仿真。文件被截断重写后，Viewer 会清除旧采样并从新表头继续刷新。

当前面向纯数值 CSV、TSV 或分号分隔文本；`.xlsx` 并不适合作为数千万采样点的仿真交换格式。
