# Four-switch buck-boost converter

**English** | [简体中文](README_CN.md)

This suite implements a four-switch buck-boost DC-DC controller with shared GMP control code for a C2000 hardware target and a PC simulation target. It is intended both as a reusable digital-power template and as an end-to-end example of the repository's SDPE-based deployment workflow.

## Layout

- `src/` contains controller code shared by all targets.
- `sdpe_general/` contains suite-wide SDPE configuration.
- `project/f280039c_Iris_node/` contains the C2000 hardware project.
- `project/simulate/` contains the Visual Studio simulation project.
- `doc/` contains UDP/SIL experiments and captured results.
- `tunable_variable_list.json` describes variables exposed to the tuning tools.

## Recommended workflow

Start with the lowest `BUILD_LEVEL` and validate sensing, PWM polarity, protection, and open-loop power-stage behavior before enabling closed loops. Keep common control parameters in `sdpe_general/` and target-specific mappings in each project's SDPE configuration.

The simulation target uses the repository-managed Python and vcpkg environment when available. Run the GMP installation entry point before building so generated source/include files, SDPE tools, and native dependencies are present. See the [UDP/SIL experiment report](doc/README.md) for the current validation record.
