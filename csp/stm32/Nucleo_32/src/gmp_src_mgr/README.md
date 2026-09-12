# Shared GMP source-manager input

`gmp_framework_config.json` is the canonical module selection for every board
under `csp/stm32/Nucleo_32`. Do not commit hand-edited `gmp_inc` or `gmp_src`
trees as replacements for this file.

To materialize an exported CubeMX project, place the repository-standard
`gmp_generate_inc` and `gmp_generate_src` wrappers beside this configuration,
set `GMP_PRO_LOCATION`, generate headers first, and then generate sources. The
wrappers are distributed from `tools/facilities_generator/src_mgr/gmp_src_mgr`;
they are not copied here so there is only one maintained script template.

The initial module set supports the runtime entry, cooperative scheduler, GMP
Data Link, the CTL dispatch path, and the STM32 CSP. Add modules here only when
the shared bring-up application actually consumes them.
