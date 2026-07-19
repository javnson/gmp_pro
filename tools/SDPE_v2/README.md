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

## Layered suite configuration

Opening `sdpe_edit.bat` from `sdpe_general` loads the common layer and every
project layer in that suite. Opening it from one target `sdpe_mgr` loads that
target together with `sdpe_general`. Generate both layers after changes; a
Simulink target should resolve one MATLAB initialization script from each layer.

## Development

The Python CLI owns validation and generation. The PyQt manager edits the same
JSON model. Shared Python dependencies are installed through
`tools/gmp_installer/requirements-gmp.txt`; project launchers must not install
packages at run time.

For the complete schema, binding, override, GUI, and validation reference, see
the [detailed Chinese SDPE v2 manual](README_CN.md).
