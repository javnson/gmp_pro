# LaunchXL-F280049C 四开关 Buck-Boost 控制工程

[English](README.md) | **简体中文**

在 CCS 中导入本目录，工程名为 `CTL_DP_FSBB_280049C`，选择 **Debug** 构建。使用 C2000Ware 5.04、TI C2000 编译器和 GMP C28x 产品配置。GMP 安装并分发工程辅助脚本后，先运行 `../../sdpe_general/sdpe_generate.bat`，再运行 `src/sdpe_mgr/sdpe_generate.bat` 和 `src/gmp_src_mgr/gmp_generate_src.bat`。修改 [目标 SDPE](src/sdpe_mgr/sdpe_requirement.json) 后重新生成再编译。

### 功率板接线

[接线与上电检查](README_WIRING_CN.md)包含主回路、电感、供电、PWM、反馈、过流和 CAN 接线。MCU 在 100 MHz TBCLK 下配置 20 kHz PWM 和 600 ns 死区，功率板自身也有门极死区。带主电源前按台架核对 [目标 SDPE](src/sdpe_mgr/sdpe_requirement.json) 中的控制等级、给定和保护门限。

### CAN 控制

实物构建上电后等待显式使能命令。将 LaunchPad 的 S9 拨到 CAN 路由，CAN 使用 **500 kbit/s**、标准帧。ID `0x101` 的首字节为 `0` 关断、`1` 使能、`2` 故障复位。确认故障消失后先复位，再重新发送使能。ID `0x102` 是两个小端有符号 32 位值，依次是电压给定和电流限值，编码为标幺值乘以 10000。Level 1 使用 SDPE 开环给定，Level 2 使用电流给定，Level 3 使用电压给定和电流限值。

### 运行 PIL

1. 只给 LaunchPad 接 USB，开启目标 SDPE 的 `ENABLE_GMP_DL_PIL_SIM`，按所需等级选 `BUILD_LEVEL`，重新生成并编译下载。首次运行还需执行 `../simulate/sdpe_mgr/sdpe_generate.bat`，生成虚拟功率级参数。PIL 期间物理门极保持关断。
2. 在 `tools/gmp_datalink/datalink_studio` 启动现有桥接器，将 `COMx` 替换成 LaunchPad 的 XDS110 Application UART。

   ```powershell
   python -m apis.examples.pil_bridge --sdpe ../../../ctl/suite/dps_fsbb/project/f280049c/src/sdpe_mgr/sdpe_requirement.json --port COMx --framed-udp
   ```

3. 在 MATLAB 中执行。运行器在内存中按 F280049C 的 PWM 计数配置现有 FSBB 模型。

   ```matlab
   addpath(fullfile(getenv('GMP_PRO_LOCATION'), ...
       'ctl/suite/dps_fsbb/project/f280049c/src/pil'));
   result = run_pil_stage(1, 0.05);
   ```

PIL 输入 ADC 顺序为 `[Vin, Vout, IL, Iout]`。返回的 PWM 依次为 Buck 上管和虚拟 Boost 上管，数字输出为虚拟使能。恢复实物控制前关闭 PIL 开关、重新生成并编译下载。

已验证 Level 1 实物和 PIL 固件均可用 TI C2000 22.6.1.LTS 编译和链接。MATLAB R2025b 使用模拟 Data Link 响应，与 FSBB 模型完成 6 步 UDP 会话回环。LaunchPad 真机运行和功率板带电测试待验证。
