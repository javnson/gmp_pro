# GMP Chip Support Package (CSP)

**English** | [简体中文](readme_cn.md)

CSP connects the platform-independent `core` and `ctl` layers to a chip, host OS, or simulation environment. A platform normally provides:

| File | Responsibility |
| --- | --- |
| `csp.config.h` | Platform defaults and feature selection |
| `csp.general.h` | C platform declarations and vendor headers |
| `csp.general.hpp` | Optional C++ platform entry |
| `csp.typedef.h` | Overrides for portable GMP and peripheral handle types |

Runtime hooks are declared in `core/rt/csp_port.h`; peripheral contracts are declared in `core/dev/peripheral_port.h`. Platform implementations must keep chip registers, vendor SDK calls, and interrupt plumbing in the CSP or project-local `xplt` layer.

Applications include the repository-root `<gmp_core.h>` for the full runtime or `<gmp_core.hpp>` for the C++ wrapper. Code that only consumes portable CTL algorithms may define `GMP_CTL_PORTABLE` before including `<gmp_core.h>`.

See the [CSP guide](CSP_GUIDE.md) for the current platform inventory and porting checklist.
