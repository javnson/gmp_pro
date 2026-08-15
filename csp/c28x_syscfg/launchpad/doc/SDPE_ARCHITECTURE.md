# LaunchPad SDPE architecture

## Problem addressed

The first LaunchPad implementation stored a schema, an entity, a requirement
and generated files inside every `C2000Lib_*` directory. It also generated a
shared `ctrl_settings.h` independently from each board header. That layout
duplicated physical board facts and allowed a fresh board header to be compiled
with stale shared settings.

## Canonical composition

The corrected layout follows the established `ctl/suite` SDPE model. For
example,
`ctl/suite/mcs_pmsm_nt/project/f280049c/src/sdpe_mgr/sdpe_requirement.json`
binds suite-level data through `common_requirements` while retaining
target-specific hardware and options in the private project requirement.

The LaunchPad equivalent has three ownership levels:

1. `csp/c28x_syscfg/sdpe_component` stores reusable board schemas and entities.
2. `launchpad/src/sdpe_mgr/sdpe_requirement.json` stores portable Common
   application settings.
3. `launchpad/src/sdpe_mgr/requirements/<BOARD>/sdpe_requirement.json` stores
   one board include and its Private ADC, PWM and DAC choices.

Each board requirement declares `common_requirements` and is the only C
generation entry point. SDPE therefore emits one merged
`C2000Lib_<BOARD>/launchpad_board.h`. Source files include that header and do
not include a parallel `ctrl_settings.h`.

MATLAB generation retains SDPE's normal two-script loading contract. The board
script first runs `ctrl_settings_matlab_init.m`, then applies its private board
values. Both scripts are generated into the same
`requirements/<BOARD>` directory. The repository ignores generated MATLAB
scripts, and the generator refreshes the pair together.

## Channel selection contract

Every board entity exports option sets for all usable BOOSTXL ADC inputs, all
six BOOSTXL PWM outputs and all device DAC outputs. The Private requirement
uses those sets for:

- `LAUNCHPAD_CONTROL_ADC_SELECT`
- `LAUNCHPAD_CONTROL_PWM_BASE`
- `LAUNCHPAD_CONTROL_DAC_BASE`

The ADC selection token expands through generated maps to both its SOC alias
and matching result base. This prevents a user from selecting an ADC SOC while
accidentally reading another ADC module. Devices without a DAC expose only
`0U`; `LAUNCHPAD_HAS_EXTERNAL_DAC` then compiles the DAC mirror path out.

The control ISR reads the selected ADC, normalizes it to the selected PWM pair,
and writes the raw code to the selected DAC when one exists. PWM frequency and
task periods come from the Common requirement but appear in the same generated
board header.

## Repository registration and regeneration

`tools/SDPE_v2/sdpe_settings.json` registers both
`csp/c28x_syscfg/sdpe_component/sdpe_schemas` and
`csp/c28x_syscfg/sdpe_component/sdpe_src`. Consequently validation, editing and
generation resolve the same component library without copying it into a CCS
project.

For manual regeneration:

```bat
csp\c28x_syscfg\launchpad\tools\generate_board_sdpe.bat F280049C %GMP_PRO_LOCATION%
```

For CCS regeneration, set `GMP_PREBUILD_SDPE=1`. The pre-build hook derives
the repository root from the registered `GMP-Core-C28x` product, selects the
board from the active configuration name, and deletes the active generated
header if generation fails. This prevents CCS from silently compiling stale C
bindings.
