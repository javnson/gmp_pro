# GMP Pro 开发环境安装说明

[English](README.md) | **简体中文**

GMP Pro 提供经典系统环境和仓库私有环境两种兼容安装方式。两种方式都会先检查仓库路径，并注册用户环境变量：

```text
GMP_PRO_LOCATION=<gmp_pro 根目录的绝对路径>
```

推荐使用私有环境。Python、Git、CMake、Ninja、Doxygen、Graphviz 和 vcpkg 等由 GMP 管理的工具会安装到 `gmp_pro/bin`，不会写入用户的永久 `PATH`，也不会安装 Scoop。经典环境则通过 Scoop 安装用户级工具，并在检测到 Visual Studio C++ 时启用用户级 vcpkg 集成。

Visual Studio 是可选能力。没有安装 Visual Studio 或 C++ 工作负载时，硬件工程、CCS 注册、Python 工具、SDPE、源码管理和文档工具仍可正常安装；安装器只跳过 Visual Studio 仿真工程的 vcpkg 依赖恢复。

Linux 使用独立的仓库私有环境，不安装系统软件、不修改 shell profile，也不持久修改
`PATH`。在仓库根目录运行：

```bash
bash install_gmp_virtual_env.sh
source bin/linux/activate_gmp.sh
```

安装器会把现代 Python、虚拟环境、vcpkg 和本机依赖放入 `bin/linux`。即使系统只提供
旧版 Python，GMP 工具仍使用私有 Python 3.12。服务器默认安装无 GUI 配置；Linux
工作站需要 Qt 工具时可使用 `--with-gui`。只有 Python、vcpkg、Facility 依赖审计和
无 GUI 的 SDPE 回归检查全部通过后，才会生成
`bin/linux/gmp_virtual_env_installed.flag`。

如果默认 Python 或 PyPI 下载端点速度较慢，可以只对本次安装设置 `uv`/pip 的标准
环境变量；安装器不会持久保存镜像设置：

```bash
UV_PYTHON_INSTALL_MIRROR=<python-build-mirror> \
PIP_INDEX_URL=<python-package-index> \
bash install_gmp_virtual_env.sh
```

安装器也支持没有 `.git` 目录的源码压缩包。此时工程发现过程只会在
`bin/linux/cache/discovery-git` 中建立私有 Git 元数据，不会把源码目录转变为 Git
仓库。

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

该模式通过 Scoop 安装或验证 Git、Python、CMake、Ninja、Doxygen、Graphviz 和 vcpkg，并安装 GMP 所需的 Python 包。检测到 Visual Studio C++ 时，还会运行 vcpkg 集成并恢复所有 `ctl/suite/*/project/simulate/vcpkg.json` 依赖，以及 `environment_manifest.json` 显式登记的特殊项目；其中 MNA 求解器通过自己的 `vcpkg.json` 提供 Eigen3 代码生成依赖。

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

Windows 私有环境会先把所有工程的 vcpkg 依赖合并为一个总 manifest，再对共享安装目录
执行一次恢复。不得逐个 manifest 依次恢复到同一个目录，否则 vcpkg 会删除当前
manifest 未声明、但其他工程仍然需要的软件包。

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

## 7. 重新添加 TI CCS Product

完整安装与复制部署都会生成 C28x/C29x Product 元数据。仓库移动、GMP 更新或 Product
版本变化后，单独重新生成可运行：

```bat
tools\facilities_generator\ccs_product_installer\install_ccs_products.bat
```

关闭 CCS，在 Product discovery path 中移除旧的 GMP 根目录或失效路径，再添加：

```text
<GMP_PRO_LOCATION>\csp\c28x_syscfg   -> GMP-Core-C28x
<GMP_PRO_LOCATION>\csp\c29x_syscfg   -> GMP-Core-C29x
```

刷新 Product 列表并重启 CCS。C28x 可使用 CCS 18；C29x/F29H85x 必须使用 CCS 21 或
更新版本。工程 include path 应使用 Product 导出的 `GMP_PRO_ROOT` 以及对应的
`GMP_C28X_CSP_ROOT`/`GMP_C29X_CSP_ROOT`，不能保留旧的 `GMP-PRO-SDK` 宏。

## 8. 维护入口

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

分发范围包括 STM32、C28x、C29x CSP 工程和所有 `ctl/suite/*/project/*` 目标。

更完整的发布、服务脚本、vcpkg、代理和回归维护规则请阅读 [英文维护手册](README.md)。
