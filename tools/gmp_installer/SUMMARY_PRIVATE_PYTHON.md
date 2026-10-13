# GMP 私有 Python 修改摘要

日期：2026-10-13  
适用范围：Windows 私有环境安装、复制部署及公共 GMP 工具启动流程。

## 1. 修改目的与问题原因

将 GMP 使用的 Python 固定到仓库的 `bin/python/python.exe`，使安装和工具运行独立于本机已有的 Python。

原安装流程通过普通 Python EXE 安装程序传入 `TargetDir=<仓库>/bin/python`。本机已经安装了 Python 3.12，安装程序因此执行已有安装的升级或维护操作。

本机安装日志显示：

- 2026-10-10：已有 Python 3.12.3 被升级为 3.12.10，实际安装目录仍为用户目录下的 `Programs/Python/Python312`。
- 2026-10-13：再次运行时执行 `Modify`，安装程序返回 `0x0`，仓库内仍未生成 `bin/python/python.exe`。
- GMP 安装脚本随后检查私有解释器文件，报出 `Private Python executable was not created`，并返回错误码 1。

## 2. 修改后的行为

### 私有运行时安装

- 从 [python-build-standalone 20250409 发布页](https://github.com/astral-sh/python-build-standalone/releases/tag/20250409)下载 Windows x64 Python 3.12.10 独立运行时，校验 SHA-256 后解压到 `bin/python`。
- 版本、下载地址及校验值统一维护在 `environment_manifest.json`；环境版本由 `1.4.1` 更新为 `1.4.2`。
- 保留 pip 和 Tkinter，满足 Python 包安装和源码管理 GUI 的需要。
- 校验 Python 版本、64 位架构、`sys.executable`、`sys.prefix`、`sys.base_prefix`，并验证 SSL、ensurepip、Tkinter 和 Tcl 初始化。
- 在临时解压目录及最终安装目录分别验证运行时；临时目录清理前检查其位于仓库私有下载缓存内。
- 复用已通过校验的私有运行时和下载缓存，支持断点续传、下载重试及低速超时；下载继续使用安装器选择的代理。
- 已有 `bin/python` 不完整或版本校验失败时，报错并提示将该目录移开后重试，保留已有目录供用户检查。

首次安装需要 Windows PowerShell、`curl.exe` 和 `tar.exe`。引导过程直接解压运行时，不执行普通 Python EXE 安装程序，不注册 Python，也不修改系统 Python 或永久 PATH。

### Python 与 pip 隔离

- 安装、复制部署和安装器子进程明确使用仓库内的 Python。
- 激活时将私有 Python 放在 PATH 首位，设置私有 `PYTHONHOME`，清除外部 `PYTHONPATH`，禁用用户包目录。
- 私有环境清除外部 `PIP_TARGET`、`PIP_PREFIX`，禁用 pip 用户安装及配置文件，避免包被安装到外部目录。
- Visual Studio 子进程保留上述私有环境设置；批处理激活后重新检查 PATH 首项。
- 拆分长 PATH 的批处理处理语句，避免超出 CMD 命令行长度限制；重复激活不会重复增加 Python 路径。

通过环境变量设置的代理和包下载镜像仍可使用。此前依赖 pip 配置文件设置镜像的用户，可通过 `PIP_INDEX_URL` 等环境变量指定镜像。

### 工具启动保护

- 公共启动保护要求私有 Python 和安装完成标志存在；环境缺失或未安装完整时返回错误，提示运行私有安装器或复制部署入口。
- 已激活的私有环境允许安装器子流程在完成标志写入前继续运行，同时检查私有解释器文件是否存在。
- 取消普通 GMP 工具自动回退到系统 Python 的行为，并保留调用者的工作目录。

旧版显式系统安装入口 `install_system.bat` 仍保留，本次未修改其安装逻辑；公共工具启动保护统一选择私有环境。

## 3. 涉及文件

以下路径均相对于 `tools/gmp_installer`。

| 文件 | 修改内容 |
| --- | --- |
| [bootstrap_private_python.ps1](bootstrap_private_python.ps1) | 新增独立 Python 的下载、校验、解压、路径验证和安装计划输出。 |
| [install_online.bat](install_online.bat) | 调用新引导脚本，移除普通 EXE 安装方式，并隔离启动环境。 |
| [environment_manifest.json](environment_manifest.json) | 固定独立 Python 下载地址及 SHA-256，增加 Tkinter 检查，更新环境版本。 |
| [activate_env.bat](activate_env.bat) | 强制私有 Python 和 pip 隔离，处理长 PATH 与重复激活。 |
| [ensure_gmp_environment.bat](ensure_gmp_environment.bat) | 取消系统 Python 自动回退，检查私有解释器和安装完成状态。 |
| [environment_manager.py](environment_manager.py) | 隔离安装器子进程的 Python、pip 环境，并保护 Visual Studio 子进程设置。 |
| [deploy_portable.bat](deploy_portable.bat) | 复制部署时明确设置私有 Python 环境。 |
| [test_environment_manager.py](test_environment_manager.py) | 增加环境污染、私有环境不完整、长 PATH 和重复激活回归检查。 |
| [README.md](README.md)、[README_CN.md](README_CN.md) | 更新安装说明、启动保护行为及 Python 版本维护规则。 |

仓库根目录的 [install_gmp_virtual_env.bat](../../install_gmp_virtual_env.bat) 保持作为用户入口，继续调用上述在线安装流程。

## 4. 首次私有 Python 改造的验证

| 验证项 | 结果 |
| --- | --- |
| 实际下载、SHA-256 校验及私有运行时解压 | 通过，已生成 `bin/python/python.exe`。 |
| 版本、架构、解释器与运行时目录、Tkinter/Tcl 初始化 | 通过，版本为 Python 3.12.10。 |
| 再次运行 Python 引导脚本 | 通过，复用已有私有运行时。 |
| 在线安装 BAT 的 `--plan` 入口 | 通过，显示从 manifest 读取的私有 Python 版本。 |
| 安装器单元及回归测试 | 6 项测试通过，包括环境污染、4 种不完整状态、长 PATH 和重复激活检查。 |
| Python 编译检查、PowerShell 语法检查 | 通过。 |
| 实际激活后的 Python 和 pip | Python 首选路径及 pip 包路径均位于仓库 `bin/python`。 |
| `git diff --check` | 通过。 |

可在仓库根目录复查回归测试：

```bat
bin\python\python.exe -I -m unittest discover -s tools/gmp_installer -p test_environment_manager.py
```

## 5. 当前状态与后续使用

后续安装中，私有 Python、`requirements-gmp.txt` 中的 Python 包及私有 Git 已安装并验证。其他便携应用、vcpkg 和工程配置的完整安装尚未完成，因此当前没有生成 `bin/gmp_virtual_env_installed.flag`。Git 安装阶段的补充修复见下一节。

在仓库根目录重新运行以下入口，完成剩余安装：

```bat
install_gmp_virtual_env.bat
```

完整安装成功后可检查实际使用的解释器与 pip：

```bat
gmp_env.bat python --version
gmp_env.bat python -m pip --version
```

在线安装和复制部署继续在全部必要步骤成功后创建完成标志。本次验证覆盖 Python 引导、环境隔离和启动保护，未验证完整依赖安装、所有 GMP GUI、Visual Studio 仿真构建或跨机器复制部署。

## 6. 后续修复：Git 解压目录重命名被拒绝

Python 包安装和 `pip check` 成功后，安装器在将 `bin/apps/.git.staging-<进程号>` 重命名为 `bin/apps/git` 时遇到 `[WinError 5] 拒绝访问`。

使用已经下载的 MinGit 压缩包复现了同一错误。后续再次解压成功，私有 Git 可正常输出 `git version 2.47.1.windows.1`。已确认失败点是解压后的整目录重命名；具体造成访问拒绝的进程或系统组件未确认。

在 [environment_manager.py](environment_manager.py) 的共享 `extract_zip` 流程中增加了以下处理，Git、CMake、Ninja、Doxygen、Graphviz 和 vcpkg ZIP 安装均使用该流程：

- 遇到重命名 `PermissionError` 时，保留已经解压的临时目录，每隔 0.5 秒重试，最多尝试 10 次。
- 重试只执行目录重命名，复用下载缓存和已解压文件。
- 持续被拒绝时返回安装失败，显示目录、原始错误，以及关闭使用该目录的程序、检查权限后重试的提示。
- 继续在异常路径清理临时解压目录，完成标志仍由完整安装流程管理。

新增两项回归测试：短暂占用后安装成功，以及持续占用达到次数上限后失败并清理临时目录。安装器回归测试总计 **8 项通过**，Python 编译检查、`pip check`、私有 Git 版本命令和 `git diff --check` 均通过。

目前可重新运行 `install_gmp_virtual_env.bat` 继续安装。已有私有 Python、已安装的包及 Git 下载缓存会复用，完整安装结果仍需以后续安装输出为准。
