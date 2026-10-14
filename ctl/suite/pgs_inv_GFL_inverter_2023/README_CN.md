# 三相并网跟网型变流器（GFL）

[English](README.md) | **简体中文**

本工程实现三相两电平跟网型并网变流器控制，控制主体位于 `src`，目前包含 F280039C IRIS、LaunchXL-F280049C、PC 仿真和 STM32G431 四个平台。IRIS 与 F280049C 是已完成硬件调通的平台；PC 仿真用于软件验证，STM32G431 仍应按增量调试流程完成实机验证。

## 两层 SDPE 配置

工程已弃用各平台原有的 `xplt/ctrl_settings.h`，参数分成公共层和平台层：

- `sdpe_general/sdpe_requirement.json`：网压/频率、PLL、P/Q 外环、调试给定、ADC 校准器、控制算法开关等平台无关参数。
- `project/<platform>/sdpe_mgr/sdpe_requirement.json`：`BUILD_LEVEL`、控制频率、PWM 时基、标幺基值、传感器标定、板级资源映射以及硬件实体。
- `src/ctl_settings_defaults.h`：控制源码唯一的公共设置入口。
- `src/sdpe_pgs_inv_gfl_common_settings.h` 和各平台 `sdpe_mgr/sdpe_pgs_inv_gfl_*_settings.h`：SDPE 生成文件，不应手工修改。

公共工程带有前缀 `PGS_INV_GFL_COMMON`，各平台也使用独立前缀，因此多个 SDPE 模块包含在同一编译单元时不会发生项目元数据宏冲突，也不再需要通过 `#undef SDPE_PROJECT_*` 清理公共层元数据。

从 `sdpe_general` 目录运行：

```bat
sdpe_edit.bat
sdpe_validate.bat
sdpe_generate.bat
```

`sdpe_edit.bat` 会同时打开公共配置和 `project` 下全部四个平台配置。平台目录中的 `sdpe_edit.bat`、`sdpe_validate.bat` 和 `sdpe_generate.bat` 可用于单独维护某个平台。运行脚本前需设置 `GMP_PRO_LOCATION` 指向仓库根目录。

## 硬件配置

IRIS 和 LaunchXL-F280049C 平台均在 SDPE 中选择以下功率硬件：

- `gmp_helios_3phganinv_lv`：三相 GaN 逆变器及其功率器件、电流传感器定义。
- `gmp_harmonia_3ph_lc_filter`：并网 LC 滤波器及网侧电流、电压采样定义。

平台层保留了原工程已经调通的 80 V 直流母线基值、10 A 电流基值和传感器标定值。硬件实体中尚标记为待确认的 BOM 参数不会自动覆盖这些实机标定值。IRIS 的 PWM 负逻辑也只在 IRIS 平台层启用。

当前平台默认值如下：

| 平台 | 默认 BUILD_LEVEL | 控制频率 | 状态 |
|---|---:|---:|---|
| F280039C IRIS Node | 3 | 20 kHz | 已调通，保持原默认值 |
| LaunchXL-F280049C | 1 | 10 kHz | 已调通，保持原默认值 |
| PC simulate | 2 | 20 kHz | 可编译仿真 |
| STM32G431 | 1 | 10 kHz | 待实机增量验证 |

## 增量调试级别

`BUILD_LEVEL` 现在由各平台 SDPE 工程选择：

1. 电压开环，验证采样、PWM 极性、死区和门极驱动。
2. 离网电流闭环，验证 dq 电流调节器。
3. PLL 并网与正/负序电流闭环。
4. 在级别 3 的基础上启用解耦、主动阻尼和超前补偿。
5. 完整功率闭环：P/Q 外环生成 dq 电流给定，内层电流环继续按 PWM 频率执行。

严禁在未完成前一级保护与波形检查时直接切换到更高等级。启用 `SPECIFY_ENABLE_ADC_CALIBRATE` 时，必须保证被校准的功率输入处于已知零状态；并网带电输入下不应进行零偏校准。

## 功率闭环

本次检查确认原有 `gfl_pq_ctrl` 虽然被初始化和调用，但没有给定、没有启用，并且电流矢量限幅公式会错误地对分量平方，因此原程序并未形成可用的功率闭环。现在 `BUILD_LEVEL == 5` 实现完整级联结构：

```text
P*/Q* -> P/Q PI（默认 1 kHz）-> 圆形 dq 电流限幅 -> dq 电流环（10/20 kHz）-> PWM
                 ^                              |
                 |------ vdq、idq 测量 ---------|
```

功率定义为：

```text
P = vd*id + vq*iq
Q = vq*id - vd*iq
```

在 PLL 锁定且 `vq ≈ 0` 时，正有功对应正 `id`，正无功对应负 `iq`。Q 控制器已按这一约定修正反馈方向。P/Q 外环使用独立分频器运行，电流给定采用圆形幅值限制，并对 PI 积分器执行限幅回算。默认给定、增益、外环频率和电流上限均由公共 SDPE 页面管理，也可通过 Datalink 在线观察或修改 `pq_ctrl.pq_set`、`pq_ctrl.pq_meas`。

## 编译与生成顺序

推荐修改参数后的顺序：

1. 在 `sdpe_general` 运行 `sdpe_validate.bat`。
2. 修改公共参数后运行公共 `sdpe_generate.bat`。
3. 在目标平台 `sdpe_mgr` 运行平台 `sdpe_generate.bat`。
4. 再由 CCS、Keil 或 Visual Studio 编译目标工程。

PC 仿真工程已用 Visual Studio 2022 的 `Debug|x64` 配置完成编译验证。构建时若不需要自动恢复 vcpkg，可传入 `VcpkgEnableManifest=false`。

## 目录结构

```text
pgs_inv_GFL_inverter/
├─ src/                         公共控制实现与公共生成头文件
├─ sdpe_general/                公共 SDPE requirement 与管理脚本
└─ project/
   ├─ f280039c_Iris_node/
   ├─ f280049c/
   ├─ simulate/
   └─ stm32g431/
      ├─ sdpe_mgr/              平台 SDPE requirement 与生成头文件
      └─ xplt/                  平台外设适配层
```
