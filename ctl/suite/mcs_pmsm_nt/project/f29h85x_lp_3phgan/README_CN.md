# MCS PMSM NT：F29H85x LaunchPad + BOOSTXL-3PHGANINV

本目标使用 `GMP-Core-C29x` Product、F29H85x SDK 1.02.01.00、SysConfig 1.25 和 C29
Clang 2.2.0.LTS。必须使用 CCS 21 或更新版本。

硬件映射来自 TI F29x Motor Control SDK 1.00.00.00 的
`servo_drive_BXL_3PHGANINV.syscfg`：EPWM1/2/3 驱动 U/V/W，相电流为 ADCB4、ADCA7、ADCC1，
母线电压为 ADCA14，相电压为 ADCD25、ADCD1、ADCE7，QEP1 使用 GPIO84/17/34，
门极 `nEN_uC` 使用 GPIO19（高电平禁用），故障输入使用 GPIO14。UARTA 的 GPIO38/43
用于 GMP 8 位 Data Link。

构建前先运行新版 Product 安装器并在 CCS 的 Products 设置中重新发现 Product：

```bat
tools\facilities_generator\ccs_product_installer\install_ccs_products.bat
```

本工程使用双层 SDPE 管理全部配置宏：公共控制参数来自
`../../sdpe_general/sdpe_requirement.json`，板卡和外设资源来自本目录
`sdpe_mgr/sdpe_requirement.json`。修改参数后先运行公共层生成器，再运行目标层生成器：

```bat
..\..\sdpe_general\sdpe_generate.bat
sdpe_mgr\sdpe_generate.bat
sdpe_mgr\sdpe_validate.bat
```

`xplt` 只能引用 SDPE 输出的 `INV_*`、`PHASE_*`、`EQEP_Encoder_BASE`、
`DEBUG_UART_BASE`、`PWM_ENABLE_*` 等宏；SysConfig 的 `MTR1_*` 名称只存在于全局板卡
实体实现中。这样更换芯片、板卡或 BoostXL 时无需改动公共控制源文件。

随后导入 `mcs_pmsm_nt_f29h85x_3phgan.projectspec`。也可以在 GMP 命令行中运行：

```bat
C:\ti\ccs2100\ccs\utils\bin\gmake.exe -B -j4 all
```

固件启动后始终先保持三个 ePWM 的 OST Trip 且 `nEN_uC` 为禁用电平。第一次上电必须断开母线或使用
限流电源，依次验证 PWM 极性/死区、ADC 原始码和偏置、QEP 方向，再从 `BUILD_LEVEL=1` 逐级调试。
