# GMP Controller Template Library (CTL)

**English** | [简体中文](readme_cn.md)

CTL contains GMP's numeric contract, reusable C control components, dispatch
frameworks, complete cross-platform applications, portable integration
templates, and tests.

| Directory | Current role | Documentation |
| --- | --- | --- |
| `math_block` | `ctrl_gt` / `parameter_gt`, conversions, constants, transforms, and fixed-size algebra | [English](math_block/readme.md) / [中文](math_block/readme_cn.md) |
| `component` | Reusable control, interface, protection, instrumentation, motor, and power modules | [English](component/readme.md) / [中文](component/readme_cn.md) |
| `framework` | Standard and Nano control dispatch plus application state frameworks | [English](framework/doc/readme.md) / [中文](framework/doc/readme_cn.md) |
| `suite` | Runnable controller applications with shared logic and per-target adapters | [English](suite/readme.md) / [中文](suite/readme_cn.md) |
| `portable` | CTL-only integration without CSP or the GMP runtime entry framework | [English](portable/README.md) / [中文](portable/README_CN.md) |
| `unit_test` | Visual Studio/CTest regression workspace | [Test guide](unit_test/README.md) |

Reusable algorithms include `ctl/math_block/gmp_math.h`, use `ctrl_gt` helpers
on the real-time path, and keep chip registers and blocking I/O outside
`ctl_step_*` functions. Complete applications normally include `gmp_core.h`
and provide `xplt.config.h`, `ctl_main.h`, and `xplt.ctl_interface.h` through
their target include paths.

Module selection and source synchronization are driven by
`tools/facilities_generator/src_mgr/gmp_framework_dic.json`. The registry's
`sys_compile`, `sys_sim`, and `sys_hw` flags record different validation scopes;
a compilation flag must not be read as simulation or hardware evidence.
