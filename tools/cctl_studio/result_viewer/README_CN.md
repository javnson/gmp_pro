# GMP CCTL Result Viewer

此工具用于查看 CCTL 仿真生成的超大 CSV/TSV 数据文件，避免先把几千万行数据载入 Excel 再绘图。

## 启动

先使用 GMP 安装程序配置环境变量，然后双击 `run_result_viewer.bat`，也可把结果文件作为参数传入：

```bat
run_result_viewer.bat E:\path\to\mcs_pmsm_nt_cctl.csv
```

打开文件后选择 X 轴和一到多列 Y 数据，再点击 `Add selected curves`。可创建多张图；蓝色边框表示当前图，每张图分别保存自己的曲线。`Link X zoom` 用于时间轴对齐缩放，`Link Y zoom` 同时统一纵轴缩放。

程序只在后台读取被选择的列，并缓存已加载列。数据量超过显示上限时使用分桶最小值/最大值降采样，因此开关尖峰不会因简单抽点而消失。修改 `Maximum display points / curve` 后重新添加曲线即可改变显示精度。

当前面向纯数值 CSV、TSV 或分号分隔文本；`.xlsx` 并不适合作为数千万采样点的仿真交换格式。

