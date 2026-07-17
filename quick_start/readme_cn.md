# GMP 快速开始

[English](readme.md) | **简体中文**

推荐首先安装仓库私有环境：

```bat
install_gmp_virtual_env.bat
```

安装程序会注册 `GMP_PRO_LOCATION`，并把所需 Python 和命令行工具安装到 `bin`。只有编译 PC 仿真工程时才需要 Visual Studio，硬件开发并不强制依赖它。

安装后运行：

```bat
gmp_env.bat
```

`gmp_file_generator` 展示工程源码选择与生成流程，`usr` 提供最小用户目录示例。正式应用建议直接从 `ctl/suite/<suite>/project` 中已经维护的目标工程开始。

完整流程见仓库[中文说明](../README_CN.md)。
