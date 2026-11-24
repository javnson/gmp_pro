# 安装 GMP 开发环境

[English](Install%20GMP.md) | **简体中文**

安装器面向 Windows x64。请把 GMP 仓库放在路径不含空格和非 ASCII 字符的位置，从仓库根目录执行以下命令。硬件工程可在没有 Visual Studio 的电脑上安装 GMP 环境。编译 C2000 工程仍需 TI Code Composer Studio、相应 C2000Ware 和 SysConfig。GMP 的 MATLAB/Simulink 库有独立的安装流程。

## 选择入口

| 命令 | 写入的位置和用途 |
| --- | --- |
| install_gmp_virtual_env.bat | 推荐。把工具装在仓库的 bin 目录，完成后登记用户环境变量 GMP_PRO_LOCATION。通过 gmp_env.bat 打开会话。 |
| install_gmp.bat | 使用同一套锁定工具，额外把工具目录加入 Windows 用户 PATH。检测到 Visual Studio C++ 时启用用户级 vcpkg 集成。适用于需要在普通终端调用工具的用户。 |
| deploy_gmp_env.bat | 把另一台电脑完整安装后的 bin 复制到本地仓库后运行。它会验证复制的环境，重新配置代理和当前仓库的工程文件。 |

两个在线入口使用同一份版本清单和同一个仓库内的 bin。系统安装及私有安装是同一份工具的两种使用方式。切换为私有安装不会自动撤销以前登记的用户 PATH 或 vcpkg 集成。现有 Scoop 安装由 Scoop 自行管理。

## 先预览或下载

~~~bat
install_gmp_virtual_env.bat --plan
install_gmp_virtual_env.bat --download-only
install_gmp_virtual_env.bat
~~~

- --plan 适用于三个安装入口。它显示工具版本和安装步骤，退出时不下载文件、不生成工程文件，也不修改用户环境变量。
- --download-only 适用于两个在线入口。它把全部固定的安装器、ZIP 和 wheel 下载并校验后缓存到 bin\cache。它不会安装软件、分发工程文件或改写完成标记。后续正式安装会重新核对缓存文件。
- 双击安装入口时，执行完成后会暂停。批量调用可先执行 set GMP_INSTALLER_NO_PAUSE=1。
- 在线私有安装可以交互选择代理。无人值守时，用 GMP_INSTALLER_PROXY_CHOICE=N 指定直连，或用 GMP_INSTALLER_PROXY_CHOICE=Y 配合 GMP_INSTALLER_PROXY_URL 指定代理。在线安装及离线部署会把本机选择保存在 bin\gmp_proxy.json 和激活脚本中。复制 bin 给其他人之前，应检查这两个文件是否含有本机代理凭据。

## 安装时具体执行哪些操作

1. 通过 [版本清单](../tools/gmp_installer/environment_manifest.json) 选择 CPython 3.12.10、MinGit 2.47.1.windows.1、CMake 3.31.6、Ninja 1.12.1、Doxygen 1.13.2、Graphviz 12.2.1、vcpkg ports 2026.04.27 和 vcpkg 工具 2026-04-08。[Python 锁文件](../tools/gmp_installer/requirements-gmp-windows.lock) 固定了 Windows x64 的 Python 直接依赖、传递依赖和 pip。
2. 下载到临时文件并核对 SHA-256，通过后发布到缓存。已有缓存也会校验。HTML 错误页、截断的文件或其他版本都无法通过检查。全部文件校验成功后才开始安装或重建工具。
3. 必要时静默运行官方 Python 安装程序，目标为 bin\python。pip 从本地已校验的 wheel 安装并再次验证哈希。便携工具解压前复检文件，安装时执行实际版本命令。vcpkg 复检工具哈希和登记的 ports 版本。
4. 若检测到 Visual Studio x64 C++，合并各 Suite 和 Simulink 的依赖清单，把 x64-windows 包一次恢复到共享目录。vcpkg 在该阶段还可能联网获取构建辅助工具和端口源码。这些文件由 vcpkg 按对应工具和端口清单校验。
5. 验证工具版本、Python 包版本及导入、pip 依赖关系和适用的仿真依赖。生成仓库内 CCS 产品元数据及设施配置，分发并运行源码管理器，分发 SDPE 批处理脚本，合并独立工程的 .gitignore。
6. 写入 bin\gmp_environment.json、用户级 GMP_PRO_LOCATION 和对应的完成标记。私有安装和离线部署使用 gmp_virtual_env_installed.flag。系统安装使用 gmp_system_env_installed.flag。标记存在后，受 GMP 环境检查保护的工程脚本才会使用这份工具环境。离线部署使用已复制的工具和依赖，跳过下载、解包和安装步骤。

## 安装后检查

~~~bat
gmp_env.bat python --version
gmp_env.bat cmake --version
gmp_env.bat bin\python\python.exe tools\gmp_installer\environment_manager.py doctor
~~~

环境清单在 bin\gmp_environment.json。后续添加 Visual Studio C++ 工作负载或变更工程依赖后，用 repair_gmp_vcpkg.bat 恢复共享依赖；需要更改代理时可先运行 configure_gmp_proxy.bat。安装选项 --skip-project-setup 和 --skip-vcpkg-packages 用于诊断，返回的是诊断步骤的结果，且不会生成完成标记。排查结束后运行完整安装。

离线部署的目标机若已安装 Visual Studio C++，请从已恢复仿真依赖的主机复制 bin。MATLAB 组件安装请参阅 [GMP MATLAB/Simulink 库指南](../slib/readme_cn.md)。安装维护细节请参阅 [安装器维护文档](../tools/gmp_installer/README.md)。
