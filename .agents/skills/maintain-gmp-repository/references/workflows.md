# GMP maintenance workflows

## Contents

1. Diagnose before editing
2. Change installation dependencies
3. Add or change a reusable module
4. Change suite or SDPE parameters
5. Change a platform binding
6. Change source/SDPE distribution
7. Change Simulink/SIL
8. Validate and document

## 1. Diagnose before editing

1. Reproduce with the same entry point, working directory, environment mode, target and `BUILD_LEVEL` as the user.
2. Record `GMP_PRO_LOCATION`, `GMP_ENV_MODE`, relevant compiler/MATLAB versions and whether the private completion marker exists.
3. Use `git check-ignore -v <path>` when discovery unexpectedly includes or excludes a generated tree.
4. Trace inputs from JSON/config through generated files into compiled includes. Do not stop at the configuration source.
5. Compare the platform `xplt` mapping with the controller ports and physical/model signal contract.
6. Implement only when requested; diagnosis alone should report cause and evidence.

## 2. Change installation dependencies

### Python package

1. Add one pinned requirement to `tools/gmp_installer/requirements-gmp.txt`.
2. Add its import name to `environment_manifest.json` when `doctor` must verify it.
3. Confirm both system and private modes install the same requirement set.
4. Run `pip check` and the consuming tool's focused test/import.
5. Update `tools/gmp_installer/README.md` and its Chinese counterpart when the maintenance contract changes.

### Portable application

1. Add name, pinned version, URL, destination, archive layout and expected executables to `environment_manifest.json`.
2. Update private PATH composition and doctor validation if needed.
3. Decide explicitly how system/Scoop compatibility mode obtains the same command.
4. Ensure proxy variables reach the downloader.
5. Test online construction and copied-bin deployment; the latter must not require re-downloading portable tools.

### vcpkg dependency

1. Put dependencies in the owning simulation project's `vcpkg.json`.
2. Keep the project under `ctl/suite/*/project/simulate` so convention discovery finds it, or add an exceptional manifest path to `environment_manifest.json`.
3. Do not add a Visual Studio solution without `vcpkg.json` under that convention.
4. Validate with the repository-local `VCPKG_ROOT`, downloads, binary cache and shared installed directory.
5. Test proxy propagation through MSBuild-triggered vcpkg, not only a manual shell command.

## 3. Add or change a reusable module

1. Choose ownership: general runtime in `core`, platform in `csp`, C control primitive in `ctl/component`, complete application in `ctl/suite`, host C++ experiment in `cctl`.
2. Follow existing CTL naming: type/object definition, `ctl_init_*`, `ctl_step_*`, optional clear/attach/get/set functions.
3. Avoid allocation, blocking I/O and host-only dependencies in real-time step functions.
4. Use `ctrl_gt`, `parameter_gt`, GMP math and saturation helpers.
5. Register headers/sources and dependencies in `gmp_framework_dic.json`.
6. Generate a representative project and ensure no flattened source filename collision occurs. Repository modules must not contain colliding flattened `.c`/`.cpp` names; use an explicit duplicate-name audit until the generator gains a hard-fail check.
7. Build/test the module under at least one relevant numeric/compiler mode.

## 4. Change suite or SDPE parameters

1. Determine whether the parameter is physical/common (`sdpe_general`) or target/peripheral-specific (`project/<target>/sdpe_mgr`).
2. Update the JSON source or the reusable preset referenced by it.
3. Run general generation first and target generation second.
4. Inspect generated C and MATLAB outputs for unit conversion, type, range, macro names and override order.
5. Rebuild the executable/firmware.
6. Use the lowest safe `BUILD_LEVEL`; validate sensing, PWM polarity, dead time and protection before closing loops.
7. Update the suite README when a channel contract, supported target or validated result changes.

Inside GMP, do not require generated SDPE headers or MATLAB initialization files to be committed. When exporting a suite/project as an independent repository, include those outputs and the project-local generation tools so it can build and regenerate without the parent checkout.

Never infer identical `BUILD_LEVEL` meaning between motor, DC-DC and grid suites.

## 5. Change a platform binding

Review as a set:

- `xplt.config.h`: feature selection, PC/hardware mode, buffer types and generated target settings;
- `xplt.peripheral.h/.c`: platform storage and peripheral/startup implementation;
- `xplt.ctl_interface.h`: input conversion, output conversion and fast enable/disable behavior;
- `ctl_main.h`: controller objects and dispatch;
- generated SDPE settings: bases, gains, limits, frequencies, channel assignments;
- model or schematic: physical direction and gate meaning.

Ensure an output-enable transition cannot publish stale PWM data. Preserve protection behavior in both the fast ISR path and slower task path.

## 6. Change source/SDPE distribution

1. Modify the canonical template/engine only.
2. Use Git ignore semantics through the shared discovery helper.
3. Run discovery first; inspect paths for `Debug`, `Release`, `slprj`, copied repos and other ignored trees.
4. Preserve project-owned JSON configuration.
5. Copy atomically where partial files could break a project.
6. Propagate child exit codes and summarize deployment separately from generation.
7. Run a representative project generation and inspect expected outputs.

Header generation precedes source generation by installer contract.

## 7. Change Simulink/SIL

1. Read `slib/readme.md` and the affected suite simulation README.
2. Edit `_src.slx`, MATLAB helper source, or native MEX source—not `install_path`.
3. Keep callbacks path-independent and load common SDPE before target SDPE.
4. Treat the UDP packed layout as an ABI. Compare data types/sizes/alignment and four ports at both ends.
5. Run MATLAB helper tests.
6. Regenerate the current Release with `install_gmp_simulink_lib`.
7. Load every affected library and inspect link/mask resolution.
8. Rebuild the Windows controller and run a focused SIL validation.
9. Check shutdown and error paths release UDP/MEX resources.

## 8. Validate and document

Use the smallest matrix that covers the changed contract:

| Change | Minimum evidence |
| --- | --- |
| Markdown | UTF-8, local links, reciprocal language links, `git diff --check` |
| Python generator | `py_compile`, focused unit/dry run, representative generated output |
| BAT installer/launcher | success and failure exit paths; visible final pause at user entry point |
| CTL component | host unit/build plus target compiler when compiler-specific behavior is touched |
| Suite control | exact `BUILD_LEVEL`, generated config, build, simulation or hardware result |
| Simulink helper | MATLAB unit tests |
| SLX/MEX/SIL ABI | Release regeneration, library load and affected suite co-simulation |

Report skipped checks and why. Do not imply hardware verification from simulation or simulation verification from compilation.

Use registry validation flags as follows:

- compilation: every component committed to GMP must compile in its supported configuration;
- simulation: set after a suite simulation or a relevant module unit test passes;
- hardware: set only after the module operates successfully on real hardware.

Record the target/configuration and evidence where practical. Expanding unit-test coverage to every reusable module is an explicit long-term maintenance goal.
