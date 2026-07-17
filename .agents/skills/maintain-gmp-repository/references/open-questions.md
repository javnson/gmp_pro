# Open GMP contract questions

These items are evidence-backed ambiguities or inconsistencies found during repository reading. Do not silently choose a permanent contract. Ask the maintainer when a task depends on one, then update code, user documentation, and this file together.

## Simulink library

1. **Packed port size calculation:** `slib/simulink_lib_src/src/mdl_gmp_simulink_core.m` appends `mask_unpacked_data_size` while processing `MaskPackedDataSizes`. Is this a defect, and should regression tests cover asymmetric packed/unpacked sizes?
2. **Upgrade failure contract:** `upgrade_gmp_simulink_lib.m` prints and returns when required source files or MATLAB R2022b support are missing, so `install_gmp_simulink_lib.m` can continue and print success. Should these conditions throw errors?
3. **Uninstall persistence:** `uninstall_gmp_simulink_lib.m` calls `rmpath` and deletes the Release directory but does not call `savepath`. Should uninstall persist the path removal?
4. **Function/file naming:** `reg_path_gmp_simulink_lib.m` declares `reg_gmp_simulink_lib`. Which name is the supported public entry point?
5. **Library close/register set:** installation loads `gmp_std_model_pck` but does not close it, generates `gmp_component_model` but does not load or close it, and calls `set_param(gcs,'EnableLBRepository','on')`. What is the intended set of directly registered/opened libraries?
6. **Cross-platform scope:** a Linux `GMP_SIL_Core.mexa64` exists, but the UDP helper is Windows-only and several upgrade paths use mixed Windows separators. Is Linux SIL an active supported target or only an experiment?
7. **Version source:** `get_gmp_slib_version()` reports `20250730, version 1.0.1`, while the previous README recorded a 2026-03-01 SIL update. What file or release process owns the authoritative slib version?
8. **Release selection:** several CLLLC simulation scripts add `slib/install_path/R2024b` explicitly. Should suite scripts target a fixed validated Release or resolve the running MATLAB Release?
9. **SPS migration:** the installer blocks R2026a+ because existing models use Specialized Power Systems. Is the long-term plan to freeze the current model line at R2025b, maintain two backends, or migrate models to another Simscape Electrical technology?

## Core/runtime

10. **PIL macro spelling:** `core/std/gmp_cport.h` tests `ENBALE_GMP_DL_PIL_SIM`, while suite/application code uses `ENABLE_GMP_DL_PIL_SIM`. Which spelling is canonical, and is control-tick behavior currently wrong in PIL builds?
11. **CTL assert macro:** under `DISABLE_CTL_LIB_ASSERT`, `gmp_ctl_assert(assert_cond)` references `assert_condition`. Should the macro be a pure `(void)(assert_cond)` no-op?
12. **Fatal-stop contract:** the default `gmp_base_system_stuck()` implementation is empty, while some platforms implement `gmp_port_system_stuck()`. What symbol must a CSP override, and must the default always trap?
13. **Tick wrap handling:** `gmp_base_time_sub` has explicit wrap logic, but `gmp_base_get_diff_system_tick` and `gmp_base_get_diff_ctrl_tick` use raw subtraction. Is wrap-safe behavior required for all three APIs?
14. **Weak functions on TI:** the `__TI_COMPILER_VERSION__` branch in `gmp_std_port.c` contains no weak defaults. Is every TI project required to implement the full port set, or is compiler-specific weak support missing?

## Generators and repository policy

15. **Flattened source collision:** `framework_sync_src_v3.py` warns and lets the later same-named source overwrite the earlier one. Should this be a hard error to prevent nondeterministic module composition?
16. **Change detection:** source/header synchronization compares modification time and size, not content. Is preserving exact timestamps during a copied/portable deployment expected, or should hashing/content comparison be used?
17. **Missing project configuration:** sync engines return success after warning when the global registry or local configuration is missing. During installation, should a discovered `gmp_src_mgr` without configuration fail generation?
18. **Tracked root macro:** `gmp_framework_dic.json` contains a machine-specific `GMP_PRO_LOCATION` value even though current sync code overwrites it from the environment. Should the tracked macro be removed or converted to a neutral placeholder?
19. **Generated SDPE outputs:** generated `.h` and `*_matlab_init.m` files are currently present in suite source/project trees. Which outputs must be committed so a copied standalone project builds before SDPE regeneration, and which should always be ignored?

## Validation status semantics

20. **Registry task claims:** `gmp_framework_dic.json` records compile/simulation/hardware booleans for modules, including date-stamped claims. What procedure and artifact are required before setting each flag, and who owns stale-claim cleanup?
21. **Suite validation portability:** should a suite's validated status apply only to the exact board/model and generated SDPE revision, or is there an intended broader certification definition?
