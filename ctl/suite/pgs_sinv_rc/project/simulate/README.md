# PGS SINV RC UDP/SIL 联合仿真

本目录用于单相双向整流/逆变控制器与 Simulink 功率级的 UDP 联合仿真。当前配置已完成 BUILD_LEVEL 1–5 的闭环回归，并保留重复控制器（FDRC）开关对比所需的监测量和自动报告。

## 配置结构

SDPE 使用公共层与项目层两级结构：

- 公共控制参数：`../../sdpe_general/sdpe_requirement.json`
- 公共生成文件：`../../src/sdpe_pgs_sinv_rc_common_settings.h` 和对应 MATLAB 初始化脚本
- 仿真平台参数：`sdpe_mgr/sdpe_requirement.json`
- 仿真生成文件：`sdpe_mgr/sdpe_pgs_sinv_rc_simulate_settings.h` 和对应 MATLAB 初始化脚本

公共层定义基值、QPR/FDRC、PLL、功率环和直流母线环等跨平台参数；项目层定义 BUILD_LEVEL、UDP/SIL ADC、PWM、传感器和三个仿真功率级的参数。平台生成头文件直接包含公共头文件，不再依赖 `ctrl_settings.h`。

修改参数后依次执行：

```bat
cd ..\..\sdpe_general
sdpe_generate.bat
cd ..\project\simulate\sdpe_mgr
sdpe_generate.bat
```

在 MATLAB 当前目录切换到本目录后运行：

```matlab
configure_sinv_models
```

该脚本把两级生成的 MATLAB 初始化脚本同时写入三个模型的 `PreLoadFcn`/`InitFcn`，并把功率级 Mask 按 PWM、开关器件、交流滤波器、直流母线、ADC、电压传感器和电流传感器分组。模型中出现的 disabled library link 提示来自功率级 Mask 的项目级定制，模型更新检查已通过。

## 模型与 BUILD_LEVEL

| BUILD_LEVEL | 模型 | 验证目标 |
| --- | --- | --- |
| 1 | `PGS_STD_SINV_MODEL_RLOAD.slx` | 电阻负载，正弦电压开环；仅验证采样、调制和功率级 |
| 2 | `PGS_STD_SINV_MODEL_RLOAD.slx` | 电阻负载，正弦电流闭环、输出电压锁相和 FDRC |
| 3 | `PGS_STD_SINV_MODEL_Grid.slx` | 并网电流闭环；有符号 P/Q 指令，正 P 为逆变送电、负 P 为整流吸收 |
| 4 | `PGS_STD_SINV_MODEL_Grid.slx` | 并网有功功率外环，经电流内环向电网注入指定功率 |
| 5 | `PGS_STD_SINV_MODEL_Rectifier.slx` | 整流运行，直流母线电压外环产生负有功指令 |

仿真目标启动后会自动给 CiA402 状态机发送 `ENABLE_OPERATION`，完整经历等待、ADC 校准、锁相/预充条件检查后才使能 PWM。FDRC 在进入运行态 300 ms 后投入，避免学习启动暂态。

BL1/2 的交流侧电阻为 `SINV_RLOAD_OHM=12 Ω`。BL5 的直流侧负载独立使用 `SINV_RECTIFIER_RLOAD_OHM=30 Ω`：60 V 时为 120 W，处于当前 0.9 pu 峰值限流范围；若误用 12 Ω，则负载需要 300 W/12.5 Arms，控制器必然限幅，不能作为电压环整定依据。

## 接线与数据映射

功率级 ADC 总线固定顺序为：

| ADC 索引 | 模型信号 | 控制器用途 |
| --- | --- | --- |
| 0 | `IL` | 交流电感/并网电流 `adc_i_ac` |
| 1 | `IDC` | 直流侧电流，当前保留 |
| 2 | `VDC` | 直流母线电压 `adc_v_bus` |
| 3 | `IC` | 滤波电容电流，当前保留 |
| 4 | `VC` | 输出/电网电压 `adc_v_grid` |
| 5 | `IG` | 网侧支路电流，当前保留 |

PWM 总线通道 1 驱动全桥 L 桥臂，通道 2 驱动 N 桥臂；`output_enable` 同时控制两个桥臂。采样和控制均以 PWM 周期触发。UDP 数据端口为 12500/12501，命令端口为 12502/12503；控制器和 Simulink 必须使用相反的收发方向。

控制器向 Simulink 返回 16 个监测量，顺序为：`Vac`、`Iac`、`Vdc`、`Iref`、调制波、PLL 频率、P、Q、电流误差、QPR 输出、FDRC 输出、FDRC 使能、CiA402 状态、CiA402 命令、活动故障和控制器发散诊断值。

## 构建与运行

若修改了 `gmp_framework_config.json`，先执行：

```bat
gmp_src_mgr\gmp_generate_src.bat
gmp_src_mgr\gmp_generate_inc.bat
```

然后使用 Visual Studio 构建 `Debug|x64`。输出程序为：

```text
x64\Debug\Motor_Control_Suite_SIL_Env.exe
```

`run_sinv_cosim(level, stop_time)` 用于交互仿真；`run_sinv_validation(level, stop_time, label)` 会校验可执行文件中的 BUILD_LEVEL、自动启动控制器、记录波形、计算稳态值和电流 THD，并写入 `validation/`。例如：

```matlab
run_sinv_validation(5, 5.0, 'build_level_5')
```

切换 BUILD_LEVEL 后必须重新生成项目层 SDPE 文件并重新编译，否则脚本会拒绝运行，避免模型与控制器级别不一致。

## 已完成的仿真结果

| 级别 | 关键结果 | 电流 THD |
| --- | --- | --- |
| BL1 | 13.629 Vrms，1.181 Arms，PWM 正常，无活动故障 | 3.83% |
| BL2，FDRC 关闭 | 2.000 Arms，指令 1.999 Arms，PLL 49.875 Hz | 0.8868% |
| BL2，FDRC 开启 | 2.000 Arms，指令 1.999 Arms，PLL 49.875 Hz | 0.8766% |
| BL3 | P=0.1000 pu，Q=0.0001 pu，PLL 50.000 Hz | 1.89% |
| BL4 | P=0.1499 pu（目标 0.15 pu），Q=0.0001 pu | 1.59% |
| BL5 | Vdc=60.008 V（目标 60 V），5.167 Arms，P=-0.2555 pu | 3.02% |

全部正式结果均为 `output_enable=1`、CiA402 `OPERATION_ENABLED`、活动故障为 0。BL2 使用相同 3 s 仿真窗口进行 RC 开/关公平对比；在线性电阻负载、QPR 基准 THD 已低于 0.9% 的情况下，FDRC 提供了小幅进一步改善。非线性负载或含周期性死区畸变的硬件上，RC 的作用通常会更明显，但需要重新核定学习增益、Q 滤波器截止频率和相位超前步数。

波形记录：

- [BL1 波形](validation/build_level_1_waveforms.png)
- [BL2 FDRC 关闭](validation/build_level_2_rc_off_final_waveforms.png)
- [BL2 FDRC 开启](validation/build_level_2_rc_on_final_waveforms.png)
- [BL3 波形](validation/build_level_3_waveforms.png)
- [BL4 波形](validation/build_level_4_waveforms.png)
- [BL5 波形](validation/build_level_5_waveforms.png)

每张图对应的同名 `*_metrics.json` 保存了可机器读取的完整指标。
