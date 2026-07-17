# GMP Software-in-the-Loop Tools

**English** | [简体中文](readme_cn.md)

`tools/gmp_sil` contains the host communication components used by GMP
software-in-the-loop simulations.

| Directory | Purpose |
| --- | --- |
| `udp_svr` | UDP server used by a native controller process |
| `udp_helper` | MATLAB/Simulink UDP helper and S-function support |
| `udp_helper_v2` | Newer helper experiments and platform-specific builds |

The normal SIL application is owned by a suite's `project/simulate` directory.
Use the suite documentation for ports, packet layout, startup order, and
`BUILD_LEVEL` acceptance criteria. Do not hard-code repository paths; resolve
shared files through `GMP_PRO_LOCATION`.
