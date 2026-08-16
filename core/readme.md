# GMP Core

[简体中文](readme_cn.md)

`core` is organized by responsibility. The dependency direction is:

`std -> base -> mm / pm / dev / protocol -> rt`

| Layer | Responsibility | Documentation |
| --- | --- | --- |
| `std` | Architecture, compiler, error-code, type, and configuration contracts | [std](std/README.md) |
| `base` | Portable services and reusable data structures | [base](base/README.md) |
| `mm` | Memory management | [mm](mm/readme.md) |
| `pm` | Task, scheduler, and state-management services | [pm](pm/readme.md) |
| `dev` | Peripheral contracts, drivers, and data-link services | [dev](dev/readme.md) |
| `protocol` | Transport-neutral protocol engines and data models | [CANopen](protocol/canopen/README.md) |
| `rt` | Full GMP startup and runtime assembly | [rt](rt/README.md) |

Use `<gmp_type.h>` for configuration and portable types without the full GMP
runtime. Use `<gmp_core.h>` only when the application needs the complete runtime
assembly. Source-manager output under a project is generated and is never the
authoritative copy of these modules.

See [Core architecture and maintenance guide](CORE_GUIDE.md) for ownership and
README rules.
