# CCTL power-electronics objects

**English** | [简体中文](readme_cn.md)

This directory contains experimental C++ models of power converters, switching devices, and electrical machines for host-side analysis and simulation.

Current model areas include universal buck-boost conversion, DFIG and PMSM machines, generic motor models, and MOSFET behavior. These objects are not the default embedded controller implementation; production control code normally belongs under `ctl/component` or a suite's shared `src/` directory.
