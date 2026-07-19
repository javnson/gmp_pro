# GMP 软件在环工具

[English](readme.md) | **简体中文**

`tools/gmp_sil` 保存 GMP 软件在环仿真所需的上位机通信组件。

| 目录 | 作用 |
| --- | --- |
| `udp_svr` | 原生控制器进程使用的 UDP 服务器 |
| `udp_helper` | MATLAB/Simulink UDP 辅助模块和 S-function 支持 |
| `udp_helper_v2` | 新版辅助模块实验及平台相关构建 |

实际 SIL 应用由各 suite 的 `project/simulate` 工程负责。端口、数据包格式、启动顺序和 `BUILD_LEVEL` 验收标准以对应 suite 文档为准。共享文件必须通过 `GMP_PRO_LOCATION` 定位，不能写入开发者机器的绝对路径。
