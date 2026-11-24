# LVFB_100_2PH 与 LaunchXL-F280049C 接线

[工程说明](README_CN.md) · [功率板原理图](../../LVFB_100_2PH.pdf) · [TI LaunchPad 引脚图](https://www.ti.com.cn/cn/lit/ug/swau114c/swau114c.pdf)

## 主回路与辅助供电

断开主电源和辅助电源，确认输入、输出电容已放电，再接线。以下 `CN`、`DC1`、`J2`、`J3` 均为 **LVFB 功率板**上的标号。

| 外部连接 | LVFB 端子 | 功能 |
| --- | --- | --- |
| 直流输入正极 | `CN11` | Bridge2 母线正端，Buck 侧 |
| 直流输入负极 | `CN10` | 功率地 `PGND` |
| 电感两端 | `CN7` ↔ 电感 ↔ `CN1` | `CN7` 为 Bridge2 中点，`CN1` 为 Bridge1 中点 |
| 负载正极 | `CN4` | Bridge1 母线正端，Boost 侧 |
| 负载负极 | `CN10` | 功率地 `PGND` |
| 12 V 辅助电源 | `DC1` | 给功率板驱动与检测电路供电 |

电感电流的正方向为 **CN7 → CN1**。LaunchPad 通过 USB 单独供电。两板只连接信号地 `GND`，保持 `CN10/PGND` 与信号地隔离。LVFB `J2-1/2` 的 5 V 和 `J3-4/6` 的 3.3 V 电源脚留空。

## PWM、使能和检测信号

LaunchPad 一列使用 BoosterPack 的 **板上丝印针号**。接线时同时核对针号和信号名。默认实物工程使用下表这两组 PWM 和两路模拟反馈。

| LVFB 针脚 | LVFB 信号 | LaunchPad 丝印针号 / 功能 | 含义 |
| --- | --- | --- | --- |
| `J2-3` | `PWM_1` → `PWM2_LS` | **79** / GPIO1 / ePWM1B | Bridge2（Buck）下管 |
| `J2-5` | `PWM_2` → `PWM2_HS` | **80** / GPIO0 / ePWM1A | Bridge2（Buck）上管 |
| `J2-4` | `PWM_3` → `PWM1_LS` | **76** / GPIO2 / ePWM2A | Bridge1（Boost）下管 |
| `J2-6` | `PWM_4` → `PWM1_HS` | **75** / GPIO3 / ePWM2B | Bridge1（Boost）上管 |
| `J2-9` | `Enable_uC` | **59** / GPIO27 | 门极使能，高有效 |
| `J2-7` 或 `J2-8` | `GND` | **60** / GND | PWM 和使能的信号回地 |
| `J3-3` | `Current_FBK` | **63** / ADCINA6 | 电感电流，零电流时约 1.65 V |
| `J3-5` | `Voltage_FBK` | **66** / ADCINC1 | 输出电压反馈 |
| `J3-9` | `CS_OC_FLAG` | **54** / GPIO31 | 硬件过流，低有效，触发两路 ePWM One-Shot Trip |
| `J3-7` 或 `J3-8` | `GND` | **62** / GND | ADC 和过流信号回地 |

在 LVFB 端将 `J2-9 (Enable_uC)` 用 **2.2 kΩ** 下拉到信号地，保证 LaunchPad 复位或控制线断开时驱动保持关断。`J2-10 (DISABLE_LS)` 和 `J3-10 (CS_Alert)` 在本接线方案中留空。

LVFB 提供 `Vout` 和 `IL` 的模拟反馈。如需测量 `Vin`，另接隔离调理电路，输出接 LaunchPad **24 / ADCINB0**，按调理电路核对增益与偏置，再启用 `FSBB_ENABLE_VIN_SAMPLE`。使用固定 Vin 时，按实际台架输入设置 `FSBB_LAUNCHXL_FIXED_VIN`。[目标 SDPE 配置](src/sdpe_mgr/sdpe_requirement.json) · [引脚配置](280049C_LaunchPad.syscfg)

## 首次上电检查

1. 保持主回路断电，只接 LaunchPad USB 和功率板 12 V 辅助电源。相对 `J3-7/8` 信号地测量，`J3-3` 约为 **1.65 V**（CCS 中 `adc_i_L.raw` 约为 **2048**），`J3-9` 约为 **3.3 V**。输出电容放电后，`J3-5` 应接近 **0 V**。
2. 确认复位时 `Enable_uC` 为低、门极关断。保持主回路断电，在 J2 控制侧确认两桥臂的互补 PWM 和死区，并核对 `/OC` 能触发两路 Trip 锁存。测功率器件的 `VGS` 时使用合适的差分探头。正常时 `g_fsbb_faults = 0`。若出现 `0x50`，先核对电流反馈是否接近零码，以及 `/OC` 电平和 Trip 状态。
3. 使用 CAN 控制时，适配器的 CANH、CANL 和信号地分别接 LaunchPad `J14-1`、`J14-2`、`J14-3`，将 S9 拨到 CAN 路由并按总线拓扑检查终端。实物固件等待显式使能。带主电源前，核对目标 SDPE 中的 `BUILD_LEVEL`、给定值和保护门限。[CAN 命令](README_CN.md#can-控制) · [TI LaunchPad 原理图](https://www.ti.com/lit/df/sprr423/sprr423.pdf?ts=1794583414859)
