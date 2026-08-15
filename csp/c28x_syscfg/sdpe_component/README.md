# C2000 SDPE component library

This directory is the canonical SDPE library for C2000 board-level schemas and
entities. `tools/SDPE_v2/sdpe_settings.json` registers both `sdpe_schemas` and
`sdpe_src`, so project requirements can include these boards without copying
component definitions into a CCS project.

LaunchPad application choices remain project-owned under
`launchpad/src/sdpe_mgr/requirements/<BOARD>`. The component entity describes
which ADC, PWM and DAC resources physically exist; the requirement selects how
the portable application uses those resources.

After changing a schema or entity, validate the global SDPE library, regenerate
the affected board binding, inspect the generated C and MATLAB outputs, and
rebuild that target.
