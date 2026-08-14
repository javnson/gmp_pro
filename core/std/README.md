# Standardization Layer

`std` 只提供标准化契约和装配规则：

- `arch`：芯片架构所需的标准宏和类型宽度候选实现；
- `cc`：编译器属性、内联、弱符号和对齐服务；
- `ec`：错误码类型、定义和实现；
- `cfg`：配置选项、默认值、类型、字节序和有效性检查；
- `gmp.std.h`：标准层内部组合头文件；
- 仓库根目录 `gmp_type.h`：供组件使用的稳定轻量入口。

覆盖顺序为用户 `xplt.config.h`、CSP `csp.config.h`、GMP 默认配置。新增
标准元件必须给出默认实现、CSP 覆盖点和配置校验，且不得依赖 `base/dev/rt`。

Facility 模块 ID 为 `core|std`。

## Options 集合

`options.cfg.h` 只定义稳定选项值，用户或 CSP 在自己的配置中选择。
启动 Logo 使用 `SPECIFY_GMP_LOGO_MODE`：

- `GMP_LOGO_MODE_FULL`：完整 ASCII Logo（默认）；
- `GMP_LOGO_MODE_LITE`：单行低占用 Logo；
- `GMP_LOGO_MODE_DISABLED`：不编译 Logo 输出。

旧宏 `SPECIFY_DISABLE_GMP_LOGO` 仍会映射到 Disabled，但新项目应使用模式宏。
