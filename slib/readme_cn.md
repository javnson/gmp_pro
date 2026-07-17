# GMP MATLAB/Simulink 库（`slib`）

[English](readme.md) | **简体中文**

`slib` 是 GMP MATLAB/Simulink 集成的源码和发行构建目录，包含可复用的功率级与虚拟外设模型、与 GMP 数值语义一致的控制辅助模块、UDP 软件在环（SIL）桥接、标准模型以及由 SDPE 参数驱动的 MATLAB 初始化工具。

## 依赖和支持边界

- `upgrade_gmp_simulink_lib.m` 要求 MATLAB/Simulink R2022b 或更新版本。
- 现有功率级模型依赖 Simscape Electrical Specialized Power Systems 及其 `powerlib`。
- 当前安装器会拒绝 R2026a 及更新版本，因为这些版本不再包含 Specialized Power Systems。现有模型应使用 R2025b 或更早版本。
- Linux SIL 属于正式支持方向，但当前实现尚不完整：仓库已有 Linux x64 `GMP_SIL_Core`，目前还没有 Linux UDP-helper 二进制文件。在完成后续适配与验证前保持现有 Linux 代码不变。
- SLX 与 MEX 文件必须与 MATLAB Release 和操作系统兼容，不能默认复用其他 Release 的生成目录。

安装器会实际尝试加载 `powerlib`。仅仅在 MATLAB 产品信息中看到相关产品，并不能证明文件和许可证都可用。

## 目录结构

| 路径 | 作用 | 维护规则 |
| --- | --- | --- |
| `simulink_lib_src/*.slx` | Simulink 库的权威源码模型 | 修改这些模型。 |
| `simulink_lib_src/src/` | MATLAB 辅助函数和已编译 MEX 文件 | 在此修改和测试脚本；MEX 应从对应原生工程重新编译。 |
| `simulink_lib_src/tests/` | MATLAB 回归测试 | 为辅助函数和路径无关初始化补充测试。 |
| `simulink_lib_src/icon/` | Library Browser 图标 | 模型中保持相对路径引用。 |
| `simulink_lib_src/avatars/` | STEP/SolidWorks 可视化资源 | 作为源资源维护。 |
| `install_path/<Release>/` | 针对一个 MATLAB Release 生成的安装目录 | 禁止直接修改，应重新生成。 |
| `tools/` | S-Function Builder 控制器示例和构建信息 | C++、wrapper、MAT 和 MEX 应保持一致。 |
| `future/` | 实验性机电模型 | 不属于稳定安装接口。 |

检出目录中可能已有 `install_path/R2022b` 和 `R2024b`，但仓库 `.gitignore` 将所有 Release 目录都视为生成物。

## 库模型组成

`slblocks.m` 将 `gmp_simulink_utilities` 注册为 Simulink Library Browser 中的 **GMP Utilities Library**。主要生成库包括：

| 库 | 作用 |
| --- | --- |
| `gmp_simulink_utilities` | GMP 控制器、被控对象、SIL、外设和数值工具的顶层入口。 |
| `gmp_fp_utilities` | GMP 浮点兼容函数、坐标变换、PWM 和测量辅助模块。 |
| `gmp_peripheral_utilities` | 虚拟 ADC、PWM、传感器、采样和外设行为。 |
| `gmp_sil_core_pack` | SIL 控制面板、通道打包、触发处理和 UDP 核心模块。 |
| `gmp_std_model_pck` | 标准变换器、电机、传感器和功率级模型。 |
| `gmp_component_model` | 部分 CTL 组件的 S-Function 封装。 |

权威模型以 `_src.slx` 结尾。`upgrade_gmp_simulink_lib` 会在 `install_path/<Release>` 中生成去掉 `_src` 的兼容副本。

## 安装、升级和卸载

先运行 GMP 安装程序，确保 `GMP_PRO_LOCATION` 指向仓库根目录，再在 MATLAB 中执行：

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), ...
    'slib', 'install_gmp_simulink_lib.m'));
```

安装流程为：

1. 检查 `powerlib` 和 MATLAB Release；
2. 临时切换到 `slib`，避免依赖调用者的当前目录；
3. 运行 `upgrade_gmp_simulink_lib` 生成 `install_path/<Release>`；
4. 复制 MATLAB 函数、MEX、图标和 avatar 资源；
5. 将 Release 目录及其 `src` 加入 MATLAB 路径；
6. 保存路径并刷新 Simulink 自定义项。

卸载当前 Release 的生成库和 MATLAB 路径：

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), ...
    'slib', 'uninstall_gmp_simulink_lib.m'));
```

`reg_path_gmp_simulink_lib.m` 只会重新注册已经生成的目录，不检查依赖，也不会重新构建模型。

卸载程序会移除生成目录对应的 MATLAB 路径，调用 `savepath` 永久保存这一变化，然后删除当前 Release 目录。

## SIL 结构

典型 GMP SIL 工程包含两个协作进程：

```text
Simulink 功率级与虚拟外设
        | ADC 码、时间、面板/数字输入
        v
GMP_SIL_Core / MEX_UDP_Helper  <--- UDP --->  Windows 控制器程序
        ^                                      （复用 suite/src 控制代码）
        | Enable、PWM compare、DAC/监视数据
        +--------------------------------------
```

原生控制器采用 `csp/windows_simulink` 并定义 `SPECIFY_PC_ENVIRONMENT`。主循环接收 `gmp_pc_simulink_rx_buffer_t`，调用 `gmp_base_ctl_step()`，再发送 `gmp_pc_simulink_tx_buffer_t`。每个 suite 的 `xplt.ctl_interface.h` 决定 ADC/PWM 的具体映射；模型和控制器必须在顺序、类型、尺寸、对齐、标度、极性与 UDP 端口上完全一致。

不要通过交换模型通道来掩盖平台映射错误。应在 suite 的仿真 README 中记录通道契约，并同步修改模型和 `xplt` 映射。

`mdl_gmp_simulink_connection` 检查纯 IPv4 地址并生成消息 TX/RX、命令 TX/RX 四端口向量。`sl_udp_server_init_cb` 会释放旧套接字、配置 MEX、连接网络并发送 `start`；`sl_udp_server_stop_cb` 发送 `stop` 后释放网络资源。

## SDPE 模型初始化

当前 suite 使用两层 SDPE 生成脚本：

```text
<suite>/sdpe_general/*_matlab_init.m
<suite>/project/<target>/sdpe_mgr/*_matlab_init.m
```

模型可调用：

```matlab
gmp_run_model_sdpe_init(bdroot)
```

该函数从已保存模型的位置计算路径，要求公共层和平台层各自恰好存在一个 `*_matlab_init.m`，并在 MATLAB base workspace 中先执行公共层、再执行平台层。目录缺失或脚本数量不唯一都会明确报错。

修改 SDPE JSON 后，应在打开或运行模型前重新生成两层配置：

```bat
sdpe_general\sdpe_generate.bat
project\simulate\sdpe_mgr\sdpe_generate.bat
```

不要在 `PreLoadFcn`、`PostLoadFcn` 或 `InitFcn` 中保存某台电脑的绝对路径；应根据模型文件位置解析，或调用 `gmp_run_model_sdpe_init`。

## 开发流程

1. 修改相应 `_src.slx` 或 `simulink_lib_src/src` 中的辅助函数。
2. 保证 block callback 和 mask 代码不依赖 MATLAB 当前工作目录。
3. 在 `simulink_lib_src/tests` 中增加或调整回归测试。
4. 运行测试：

   ```matlab
   addpath(fullfile(getenv('GMP_PRO_LOCATION'), 'slib', ...
       'simulink_lib_src', 'src'));
   results = runtests(fullfile(getenv('GMP_PRO_LOCATION'), 'slib', ...
       'simulink_lib_src', 'tests'));
   assert(all([results.Passed]));
   ```

5. 运行 `install_gmp_simulink_lib` 重新生成当前 Release。
6. 打开 Library Browser，检查 block link、mask、图标和依赖库。
7. 至少运行一个实际使用了该模块或通信契约的 suite 仿真。

修改 `tools/` 中的 S-Function Builder 组件时，应保持 `.cpp`、wrapper、`SFB__*.mat`、`rtwmakecfg.m` 和 MEX 的对应关系。已编译 MEX 文件不是可编辑源码。

## 故障排查

| 现象 | 检查内容 |
| --- | --- |
| 找不到或无法加载 `powerlib` | 安装并授权 Simscape Electrical Specialized Power Systems，使用 R2025b 或更早版本。 |
| Library Browser 中没有 GMP | 检查当前 Release 目录和 `src` 是否在 MATLAB path 中，再运行 `sl_refresh_customizations`。 |
| 源模型已修改但安装库仍旧 | 运行 `install_gmp_simulink_lib`，不要手工修改 `install_path`。 |
| 找不到 `GMP_SIL_Core` 或 `MEX_UDP_Helper` | 检查生成目录的 `src` 中是否存在与平台兼容的 MEX。 |
| UDP 超时 | 核对 `network.json`、模型 mask、进程启动顺序、防火墙和四个端口；重试前释放旧套接字。 |
| SDPE 变量未定义 | 生成公共层和平台层，并确认模型位于 `<suite>/project/<target>`。 |
| SIL 信号符号或通道错误 | 对照模型接线和 `xplt.ctl_interface.h`，同时检查 ADC 偏置/增益和 PWM 极性。 |

安装后的辅助函数可通过 `get_gmp_slib_version()` 查看库标识；浮点兼容 API 使用独立的 `gmp_float_macros_version()`。
