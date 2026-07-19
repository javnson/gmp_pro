# GMP Pro 开发环境安装说明

[English](README.md) | **简体中文**

GMP Pro 提供经典系统环境和仓库私有环境两种兼容安装方式。两种方式都会先检查仓库路径，并注册用户环境变量：

```text
GMP_PRO_LOCATION=<gmp_pro 根目录的绝对路径>
```

推荐使用私有环境。Python、Git、CMake、Ninja、Doxygen、Graphviz 和 vcpkg 等由 GMP 管理的工具会安装到 `gmp_pro/bin`，不会写入用户的永久 `PATH`，也不会安装 Scoop。经典环境则通过 Scoop 安装用户级工具，并在检测到 Visual Studio C++ 时启用用户级 vcpkg 集成。

Visual Studio 是可选能力。没有安装 Visual Studio 或 C++ 工作负载时，硬件工程、CCS 注册、Python 工具、SDPE、源码管理和文档工具仍可正常安装；安装器只跳过 Visual Studio 仿真工程的 vcpkg 依赖恢复。

## 1. 在线安装私有环境

在仓库根目录运行：

```bat
install_gmp_virtual_env.bat
```

安装器会询问是否使用代理。检测到系统代理时会显示地址并要求确认；没有检测到代理时，选择 `Y` 后可以手动输入地址，例如：

```text
http://127.0.0.1:7890
```

安装成功后会在 `bin` 中创建：

```text
gmp_virtual_env_installed.flag
```

只有所有必要安装、CCS 注册和工程工具分发全部成功后才会创建该标志。

## 2. 经典系统环境

需要兼容历史安装方式时运行：

```bat
install_gmp.bat
```

该模式通过 Scoop 安装或验证 Git、Python、CMake、Ninja、Doxygen、Graphviz 和 vcpkg，并安装 GMP 所需的 Python 包。检测到 Visual Studio C++ 时，还会运行 vcpkg 集成并恢复所有 `ctl/suite/*/project/simulate/vcpkg.json` 依赖。

## 3. 部署其他电脑复制来的 `bin`

将已经准备好的完整 `bin` 文件夹复制到新的 `gmp_pro` 根目录，然后运行：

```bat
deploy_gmp_env.bat
```

该过程不会下载软件。它会验证 Python、便携应用和 vcpkg，重新记录本机代理选择，注册 CCS，并分发源码管理、SDPE 和独立工程 `.gitignore` 文件。

## 4. 进入 GMP 环境

运行：

```bat
gmp_env.bat
```

也可以直接执行单条命令：

```bat
gmp_env.bat python --version
gmp_env.bat cmake --version
```

如果检测到 Visual Studio C++，该命令还会导入 x64 开发者命令行环境；否则只给出警告，不影响其他 GMP 工具。

## 5. 代理和 vcpkg 修复

重新选择私有环境代理：

```bat
configure_gmp_proxy.bat
```

安装 Visual Studio C++ 后补齐私有环境的仿真依赖：

```bat
repair_gmp_vcpkg.bat
```

经典环境在后续安装 Visual Studio C++ 后，重新运行 `install_gmp.bat` 即可。

## 6. 自动化调用

安装入口在成功和失败后都会暂停，方便双击运行时查看结果。自动化脚本可关闭暂停：

```bat
set GMP_INSTALLER_NO_PAUSE=1
```

无人值守代理选择可以使用：

```bat
set GMP_INSTALLER_PROXY_CHOICE=Y
set GMP_INSTALLER_PROXY_URL=http://127.0.0.1:7890
```

或强制直连：

```bat
set GMP_INSTALLER_PROXY_CHOICE=N
```

## 7. 维护入口

Python 依赖统一维护在：

```text
tools/gmp_installer/requirements-gmp.txt
```

私有环境中的可执行工具和固定版本维护在：

```text
tools/gmp_installer/environment_manifest.json
```

工程独立 `.gitignore` 的公共子集维护在：

```text
tools/gmp_installer/project_gitignore.template
```

更完整的发布、服务脚本、vcpkg、代理和回归维护规则请阅读 [英文维护手册](README.md)。
