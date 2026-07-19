# GMP CTL Component Library

**English** | [简体中文](readme_cn.md)

The CTL component library provides reusable C control modules for motor drives,
digital power converters, grid-connected systems, signal processing, protection,
and online analysis. Components sit above `ctl/math_block` and below complete
applications in `ctl/suite`.

## Library structure

| Directory | Purpose | Documentation |
| --- | --- | --- |
| `intrinsic` | Basic, continuous, discrete, protection, and advanced controllers | [English](intrinsic/readme.md) / [中文](intrinsic/readme_cn.md) |
| `motor_control` | Motor interfaces, FOC building blocks, observers, and presets | [English](motor_control/readme.md) / [中文](motor_control/readme_cn.md) |
| `digital_power` | Converter, grid, MPPT, and repetitive-control modules | [English](digital_power/readme.md) / [中文](digital_power/readme_cn.md) |
| `interface` | ADC, PWM, DAC, encoder, and controller-facing interfaces | [English](interface/readme.md) / [中文](interface/readme_cn.md) |
| `dsa` | Dynamic signal-analysis and instrumentation helpers | [English](dsa/readme.md) / [中文](dsa/readme_cn.md) |
| `hardware_preset` | Reusable hardware parameter definitions | Header documentation |

## Integration rules

- Use `ctrl_gt` for real-time control values and `parameter_gt` for physical
  parameters and initialization calculations.
- Prefer the math and saturation helpers in `ctl/math_block`; do not assume
  that `ctrl_gt` is always a floating-point type.
- Follow the `ctl_<action>_<object>` naming convention.
- Keep step functions deterministic and free of allocation, blocking I/O, and
  host-only dependencies.
- Put reusable algorithms here and application state machines in a suite's
  shared `src` directory.
- Register new headers and sources in
  `tools/facilities_generator/src_mgr/gmp_framework_dic.json` so project source
  managers can select them.

## Starting points

For a complete module inventory and development status, see:

- [Component development specification](COMPONENT_DEVELOPMENT_SPEC.md)
- [Component module catalog](COMPONENT_MODULE_CATALOG.md)

The detailed Chinese component guide contains module-by-module API and usage
notes: [GMP CTL 组件库完整指南](readme_cn.md).
