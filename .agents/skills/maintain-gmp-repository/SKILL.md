---
name: maintain-gmp-repository
description: Navigate, diagnose, document, and safely modify the GMP general motor/power-control platform repository. Use for GMP core/CSP/CTL/CCTL code, suite projects, SDPE configuration, source-manager generation, system or private environment installers, vcpkg simulation projects, MATLAB/Simulink slib and UDP SIL/PIL tooling, project distribution, or repository documentation and maintenance rules.
---

# Maintain the GMP repository

Use repository evidence before changing GMP. Treat generated and distributed files as outputs of canonical tools, not independent sources.

## Start every task

1. Resolve the repository root from `GMP_PRO_LOCATION` when running GMP tools. When only inspecting a checkout, locate the root containing `gmp_core.h`, `tools/gmp_installer`, and `.gitignore`; never bake the current drive or checkout path into code.
2. Inspect `git status --short` and preserve unrelated user changes.
3. Read [references/architecture.md](references/architecture.md) for module ownership, runtime flow, and source-of-truth boundaries.
4. Read only the relevant sections of [references/workflows.md](references/workflows.md) before changing installers, generated code, SDPE, suites, or Simulink.
5. Consult [references/open-questions.md](references/open-questions.md) before relying on behavior that the code does not establish consistently. Ask the maintainer instead of silently choosing a contract.

## Choose the authoritative file

- Edit library code under `core`, `csp`, `ctl`, `cctl`, or `vcore`; do not patch a project's generated `gmp_src_mgr/gmp_inc` or `gmp_src_mgr/gmp_src` as the upstream fix.
- Edit source-manager templates and engines under `tools/facilities_generator/src_mgr`; project `gmp_src_mgr` scripts are distributed copies, while each project's `gmp_framework_config.json` is local configuration.
- Edit SDPE engines/templates under `tools/SDPE_v2`; suite `sdpe_requirement.json` files are project inputs, generated headers and `*_matlab_init.m` files are outputs, and project BAT files are distributed copies. Ignore generated SDPE outputs inside the main GMP repository, but retain and commit them when a project is copied into a standalone repository.
- Edit Simulink library sources under `slib/simulink_lib_src`; never make the primary fix under `slib/install_path/<Release>`.
- Edit shared suite algorithms under `ctl/suite/<suite>/src` and platform bindings under `project/<target>`; do not move target register access into reusable CTL components.
- Update the global `.gitignore` and the project-ignore distributor deliberately. Standalone project `.gitignore` files must retain tools needed after copying a project outside GMP.

## Preserve the control contract

Trace a control change in this order:

```text
gmp_base_entry
  -> gmp_base_init -> setup_peripheral -> ctl_init -> init
  -> gmp_base_loop -> CSP/background tasks

periodic ISR or SIL packet
  -> gmp_base_ctl_step
  -> ctl_input_callback -> ctl_dispatch -> ctl_output_callback
```

Verify the selected `xplt.config.h`, `csp.config.h`, generated SDPE header, `ctl_main.h`, and `xplt.ctl_interface.h` together. Preserve ADC/PWM ordering, physical direction, bias, scaling, polarity, dead time, packed-buffer layout, task frequency, and `BUILD_LEVEL` semantics.

Treat a suite README's validation record as evidence for that exact target and configuration, not as proof for every platform.

## Respect installation contracts

- Register `GMP_PRO_LOCATION` before every installation mode.
- Keep system installation (`install_gmp.bat`) and repository-private installation (`install_gmp_virtual_env.bat`) compatible.
- Consider Visual Studio optional. Only native simulation dependency restore requires its x64 C++ workload.
- Use `tools/gmp_installer/environment_manifest.json` for private executable/Python versions, `requirements-gmp.txt` for Python packages, and convention-based `ctl/suite/*/project/simulate/vcpkg.json` manifests for native packages.
- Preserve proxy selection for downloads and child Visual Studio/vcpkg processes.
- Create `bin/gmp_virtual_env_installed.flag` only after a complete private installation or deployment. Remove it before attempting a rebuild.
- Keep final repository setup in both installation paths: CCS registration, facility configuration, source-manager distribution/generation, SDPE distribution, and project `.gitignore` distribution.
- Ensure user-facing BAT entry points pause on both success and failure; internal BAT helpers should return error codes without introducing nested pauses.

## Work with generated code

When changing source selection:

1. Update `tools/facilities_generator/src_mgr/gmp_framework_dic.json`.
2. Check dependency/module names against real paths.
3. Run the canonical sync engine or the project BAT files.
4. Verify `gmp_inc` mirrors repository-relative header paths and `gmp_src` contains unique flattened source names.
5. Require globally unique flattened C/C++ source filenames. The current generator only warns and overwrites on collision, so audit for duplicates until a dedicated hard-fail checker is added.

When changing SDPE:

1. Update the appropriate general or target `sdpe_requirement.json` or the SDPE generator/templates.
2. Generate general settings before target settings.
3. Inspect generated C headers and MATLAB initialization scripts.
4. Rebuild the target because JSON changes alone do not alter compiled control code.

Fleet discovery must use Git's real ignore semantics. Reuse `framework_project_discovery.exclude_git_ignored`; do not implement an incomplete `.gitignore` parser.

## Work with MATLAB/Simulink

Read `slib/readme.md` for the current supported Releases, libraries, SIL data flow, SDPE initialization, and test commands.

- Keep model callbacks independent of MATLAB's current directory and of a developer's checkout path.
- Keep Simulink channel packing synchronized with the native buffer types and target `xplt` binding.
- Regenerate `install_path/<Release>` with `install_gmp_simulink_lib`; do not hand-edit it.
- After changing helpers, run `slib/simulink_lib_src/tests` and at least one affected suite simulation.
- Treat Linux SIL as an officially supported but currently incomplete path. Preserve it, document missing UDP/helper coverage accurately, and do not refactor it until dedicated optimization and validation work is authorized.

## Validate proportionally

For documentation-only work, check UTF-8, local links, language navigation, and `git diff --check`.

For Python tools, run `py_compile` plus focused unit/dry-run commands. For distribution code, prefer discovery dry-runs before copying.

For C/C++ changes, build at least the directly affected target. A host build does not validate C28x/STM32 compiler behavior, and a hardware build does not validate the Windows SIL ABI.

For Simulink changes, run MATLAB unit tests, regenerate the current Release, load the libraries, and exercise the affected model. Report explicitly when MATLAB, a compiler, hardware, or a licensed product is unavailable.

Never report generated-file success based only on a zero exit code if the tool treats missing inputs as a warning. Inspect the expected outputs.

## Maintain documentation

- Keep default `README.md` pages in English and link a sibling `README_CN.md` or `readme_cn.md` when a Chinese manual exists.
- From English indexes, link English/default pages; from Chinese indexes, link Chinese pages explicitly.
- Document physical signal conventions, generated/source boundaries, supported tool versions, and exact validation scope.
- Update this skill when a maintainer resolves an item in `references/open-questions.md` or changes a canonical workflow.
