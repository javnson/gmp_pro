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
ctl/suite/<suite>/sdpe_general/         suite-wide parameters
ctl/suite/<suite>/project/<target>/
  sdpe_mgr/                             target requirements and output
```

All repository paths are resolved from `GMP_PRO_LOCATION`. Generated files and
project data must not contain developer-machine absolute paths.

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
