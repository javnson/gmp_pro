# SDPE v2 — System Design and Parameter Engineering

**English** | [简体中文](README_CN.md)

SDPE v2 manages reusable hardware entities, controller settings, and project
requirements for GMP power-electronics and motor-control applications. It uses
templates to define exportable parameters, entities to hold concrete hardware
data, sub-components to compose devices, and project requirements to bind those
values to generated control settings.

## Main entry points

| Command | Purpose |
| --- | --- |
| `gmp_sdpe_manager.bat` | Open the SDPE graphical manager |
| `gmp_sdpe_generate_all.bat` | Generate global hardware headers |
| `gmp_sdpe_validate.bat` | Validate schemas, entities, and requirements |
| `gmp_sdpe_deploy_project_mgr.bat` | Distribute current project launchers |

Project-local launchers live in `sdpe_mgr` or `sdpe_general`:

```text
sdpe_edit.bat       edit common and target data
sdpe_generate.bat   generate headers and MATLAB initialization
sdpe_validate.bat   validate project requirements and output
sdpe_settings.bat   inspect or change project settings
```

## Repository data flow

```text
ctl/hardware_preset/sdpe_schemas/       templates
ctl/hardware_preset/sdpe_src/           reusable entities
ctl/hardware_preset/                    generated global headers
csp/c28x_syscfg/sdpe_component/         C2000 board schemas and entities
ctl/suite/<suite>/sdpe_general/         suite-wide parameters
ctl/suite/<suite>/project/<target>/
  sdpe_mgr/                             target requirements and output
    private_hardware/<category>/        project-owned entity sources
```

All repository paths are resolved from `GMP_PRO_LOCATION`. Generated files and
project data must not contain developer-machine absolute paths.
The library search paths are declared once in `sdpe_settings.json`; both the
CTL hardware library and the CSP C2000 board-component library are therefore
available to the CLI, GUI, validation and project generators.

When one target needs a calibrated or modified version of reusable hardware,
select it on the Project page and use `Make private copy`. SDPE assigns a new
globally unique entity ID, writes the JSON beside that project's requirement,
and updates the project hardware and binding references. Project generation
automatically discovers `private_hardware`; an ID collision is an error rather
than an implicit override of the system preset.

Generated project headers and `*_matlab_init.m` files duplicate information in
the SDPE requirements and library, so they may be ignored in the main GMP
repository. When a project is copied into an independent repository, commit
those generated outputs together with the project-local tools so the exported
project remains directly buildable.

## Composed suite configuration

Each private project stores zero or more common requirement references in its
`common_requirements` array. A reference may be relative to the private JSON,
absolute, or use environment-variable syntax such as
`%GMP_PRO_LOCATION%/...`, `$GMP_PRO_LOCATION/...`, or
`${GMP_PRO_LOCATION}/...`. The editor presents the private data and every bound
common file as one view, identifies each item's source, and validates macro
duplicates across the complete composition.

Generation writes one final C header named by the private project's
`output_header`. It directly contains the private configuration followed by all
bound Common fallbacks, so Code preview, search, and source jumps always target
one file. MATLAB initialization scripts remain a Common-then-private chain to
preserve the existing Simulink loading contract. Items can be moved from a
private project into a selected common file, or from a common file into one or
more selected private projects.

Common requirements and macros remain editable in the merged project view and
are saved back to their owning Common JSON files. The Requirements context menu
creates Private and Common rows explicitly. `Cover with private requirement`
creates a private row with the same macro. The private row is then the tree parent, each
matching Common row is a child, and the Common definition is forced weak. The
final C header therefore emits the private definition before its Common fallback
definition, whose matching macro is guarded by `#ifndef`.

Save commits an active cell editor before writing every affected source file.
Delete removes the selected row even when focus is on a permanent checkbox; a
real text editor keeps normal character deletion. Deleting a group removes its
complete subtree, while deleting only a Private cover preserves its Common child.

Generated MATLAB initialization scripts finish with a console summary of the
project identity, suite/version, selected hardware, bound Common files, enabled
variable values, and disabled macro names.

### Canonical `ctl/suite` deployment pattern

Do not copy reusable schemas and entities into every target project. Register
their library directories once in `sdpe_settings.json`, then keep only
application requirements under the project. A target requirement should be the
generation entry point and normally contains:

- `hardware` for target-specific entities;
- `option_macros` and `options_preset` for selectable target resources;
- `common_requirements` for suite-wide algorithms, timing and protocols;
- `output_header` for the single C header consumed by that target.

This is the pattern used by existing suite targets such as
`ctl/suite/mcs_pmsm_nt/project/f280049c/src/sdpe_mgr/sdpe_requirement.json`.
The LaunchPad reference applies the same pattern to a multi-configuration CCS
project: reusable C2000 board components live in
`csp/c28x_syscfg/sdpe_component`, shared application policy lives in one Common
requirement, and eight Private requirements select one board and its ADC/PWM/DAC
channels.

Generate from the Private requirement, not separately from the Common file.
The C result is one merged target header, which prevents a fresh target binding
from being compiled with stale Common macros. MATLAB intentionally remains a
Common-then-Private script chain; generate both scripts into the same target
output directory so the chain cannot cross board configurations. A concrete
description and command are in
`csp/c28x_syscfg/launchpad/doc/SDPE_ARCHITECTURE.md`.

## Numeric-domain contract

Schema parameters use `numeric_domain` to distinguish `real`, `parameter`,
`ctrl`, and nonnumeric `raw` values. Physical and tuning values normally select
`parameter`; SDPE then emits `real2param(value)` without an `f` suffix so a
double `parameter_gt` retains the JSON precision. Use `ctrl` only when a macro
is explicitly consumed as cached real-time data. Project bindings provide the
matching `real`, `parameter`, and `ctrl` keys. The project editor displays
these choices as `real_gt`, `parameter_gt`, and `ctrl_gt`. Every numeric value
entered in the Binding Value cell is treated as a real literal before SDPE
converts it to the selected destination domain. New numeric requirements
default to `parameter_gt`; integer and complete C-expression bindings continue
to use `number` and `literal` explicitly.

MATLAB initialization scripts define `real2param`, `real2ctrl`, `param2ctrl`,
and `ctrl2param` function handles. Their default simulation domains are double;
projects may override the `GMP_SDPE_*_TYPE` variables and conversion handles
before running the generated scripts. The C conversion macros do not create an
explicit `real_gt` temporary, allowing embedded compilers to fold constants
directly into the selected destination representation.

## Suite layout contract

Suite requirements use one repository-wide top-level order: `Peripheral
Parameters`, `Per-Unit Base Parameters`, `Main Element Parameters`,
`Protection Parameters`, `Control Loop`, `Runtime Parameters`, `Voltage &
Current Sensor`, and `Commissioning Defaults`. Controller settings use a child
group such as `Control Loop / PLL Controller` or `Control Loop / Current
Controller`; do not collect unrelated controllers into one generic page.

PWM, ADC, GPIO, timer, encoder, and communication assignments belong in Option
Macros with the applicable hardware entity `options_preset`. Do not hide
selectable peripheral assignments in `code_sections`. Motors, grid filters,
resonant tanks, and other main components belong in `hardware`; project macros
bind to entity exports or parameters instead of duplicating component values.

Normalize and verify the layout after editing suite requirements:

```powershell
python .\normalize_suite_sdpe_layout.py --write
python .\normalize_suite_sdpe_layout.py --check
```

## Development

The Python CLI owns validation and generation. The PyQt manager edits the same
JSON model. Shared Python dependencies are installed through
`tools/gmp_installer/requirements-gmp.txt`; project launchers must not install
packages at run time.

For the complete schema, binding, override, GUI, and validation reference, see
the [detailed Chinese SDPE v2 manual](README_CN.md).
