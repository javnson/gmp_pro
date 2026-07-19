# GMP Debugger v2

[English](readme.md) | **简体中文**

GMP Debugger v2 是用于 Datalink 和处理器在环流程的 Python/PyQt 上位机工具，支持目标通信、变量观察、参数修改、内存视图和工程调试功能。

完成 GMP 安装后运行本目录带环境守卫的启动脚本。检测到完整私有环境时，脚本会自动选择 `bin/python`；Python 依赖统一维护在 `tools/gmp_installer/requirements-gmp.txt`，不能由调试器启动时临时安装。

目标固件必须配置兼容的传输接口，并在高频控制中断之外调度通信任务。数据包、UART 和 Datalink 任务配置以所属 suite 的说明为准。
