# GMP Facilities 与源码生成工具

[English](readme.md) | **简体中文**

本目录提供仓库注册、CCS Product 元数据和 GMP 工程源码管理工具。

## 安装阶段任务

- `gmp_fac_install_ccs_product.py`：把 GMP 注册为 CCS Product。
- `gmp_fac_generate_cfg_json.py`：生成 facilities 配置元数据。
- `src_mgr/framework_distribute_tools_v3.py`：分发最新版源码管理脚本，并重新生成工程头文件和源码。

根目录安装程序会自动执行这些任务。所有程序必须通过 `GMP_PRO_LOCATION` 定位仓库，不能写入绝对路径。

## 源码管理器

每个工程的 `gmp_src_mgr/gmp_framework_config.json` 选择实际需要的 GMP 模块。分发的 BAT 工具会生成 `gmp_inc`、`gmp_src` 和 `gmp_compiler_includes.txt`。主仓库忽略生成结果；工程复制出去后则应保留 BAT 工具，保证独立生成能力。

只能修改 `src_mgr/gmp_src_mgr` 下的权威脚本，然后运行全局分发程序。位于 `Debug`、`Release` 等 Git 忽略构建目录中的重复工具不会成为分发目标。

旧实现记录保存在 [`development_notes.md`](development_notes.md)。
