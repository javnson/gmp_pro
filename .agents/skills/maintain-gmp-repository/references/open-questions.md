# GMP maintainer decisions and open questions

Use confirmed decisions as repository contracts. Do not reopen deferred code changes without new failure evidence or maintainer authorization.

## Confirmed decisions

1. Leave the current packed-size implementation in `mdl_gmp_simulink_core.m` unchanged; it currently works in supported use cases.
2. Leave the print-and-return behavior in `upgrade_gmp_simulink_lib.m` unchanged for now.
3. Simulink uninstall must permanently remove the generated MATLAB paths with `savepath`.
4. `ENABLE_GMP_DL_PIL_SIM` is canonical. `ENBALE_GMP_DL_PIL_SIM` is a historical spelling error; do not introduce it in new code or configuration.
5. Keep current fixed MATLAB Release references unchanged. A MATLAB R2022b export will be supplied separately.
6. Linux SIL is formally supported but not yet optimized. Preserve the current implementation until dedicated completion work.
7. Freeze Specialized Power Systems models at MATLAB R2025b or earlier; no migration is currently planned.
8. Generated SDPE `.h` and `*_matlab_init.m` outputs need not be committed in the main GMP repository. Commit them after copying a project into a standalone repository.
9. Flattened source filename collisions are forbidden by repository design. A dedicated duplicate-`.c` checker is desired later; do not change generator behavior yet.
10. Every committed component should compile. Mark simulation validation after a relevant suite simulation or module unit test, and mark hardware validation only after real-hardware use. Per-module unit-test coverage remains an important future project.

## Remaining Simulink questions

1. **Function/file naming:** `reg_path_gmp_simulink_lib.m` declares `reg_gmp_simulink_lib`. Which name is the supported public entry point?
2. **Library close/register set:** installation loads `gmp_std_model_pck` but does not close it, generates `gmp_component_model` but does not load or close it, and calls `set_param(gcs,'EnableLBRepository','on')`. What is the intended set of directly registered/opened libraries?
3. **Version source:** `get_gmp_slib_version()` reports `20250730, version 1.0.1`, while the previous README recorded a 2026-03-01 SIL update. What file or release process owns the authoritative slib version?

## Remaining core/runtime questions

4. **CTL assert macro:** under `DISABLE_CTL_LIB_ASSERT`, `gmp_ctl_assert(assert_cond)` references `assert_condition`. Should the macro be a pure `(void)(assert_cond)` no-op?
5. **Fatal-stop contract:** the default `gmp_base_system_stuck()` implementation is empty, while some platforms implement `gmp_port_system_stuck()`. What symbol must a CSP override, and must the default always trap?
6. **Tick wrap handling:** `gmp_base_time_sub` has explicit wrap logic, but `gmp_base_get_diff_system_tick` and `gmp_base_get_diff_ctrl_tick` use raw subtraction. Is wrap-safe behavior required for all three APIs?
7. **Weak functions on TI:** the `__TI_COMPILER_VERSION__` branch in `gmp_std_port.c` contains no weak defaults. Is every TI project required to implement the full port set, or is compiler-specific weak support missing?

## Remaining generator questions

8. **Change detection:** source/header synchronization compares modification time and size, not content. Is preserving exact timestamps during a copied/portable deployment expected, or should hashing/content comparison be used?
9. **Missing project configuration:** sync engines return success after warning when the global registry or local configuration is missing. During installation, should a discovered `gmp_src_mgr` without configuration fail generation?
10. **Tracked root macro:** `gmp_framework_dic.json` contains a machine-specific `GMP_PRO_LOCATION` value even though current sync code overwrites it from the environment. Should the tracked macro be removed or converted to a neutral placeholder?
11. **Ignore migration:** generated SDPE outputs are currently tracked in several suites even though the confirmed policy allows ignoring them in GMP. Should they be removed from Git in one repository-wide migration, or only as each suite is next regenerated?
