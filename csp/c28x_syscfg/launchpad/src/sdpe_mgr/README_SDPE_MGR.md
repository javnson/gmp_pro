# SDPE Built-in Manager Demo

This folder is the project-local SDPE manager for the portable C2000
LaunchPad reference application.

It demonstrates the intended deployment model:

- the SDPE program stays in `%GMP_PRO_LOCATION%\tools\SDPE_v2`;
- the project-local manager stores its own requirement file and batch tools;
- generation and editing are launched through `GMP_PRO_LOCATION`;
- library paths and generation modes are read from `%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json`.

Files:

- `sdpe_requirement.json`: source of truth for portable application timing,
  Data Link and feature switches. Physical board routing does not live here.
- `sdpe_settings.bat`: local settings for this manager.
- `sdpe_edit.bat`: open the Project Requirement GUI.
- `sdpe_generate.bat`: regenerate the C and MATLAB bindings in this directory.
- `sdpe_validate.bat`: validate the library and inspect the requirement file.

Usage:

```bat
set GMP_PRO_LOCATION=E:\lib\gmp_pro
sdpe_validate.bat
sdpe_generate.bat
sdpe_edit.bat
```

Generated output:

```text
sdpe_mgr\
  ctrl_settings.h
  ctrl_settings_matlab_init.m
```

Physical bindings are generated separately inside every
`C2000Lib_<BOARD>/sdpe` project. Each board has a dedicated schema and entity,
and each entity contains the shared repository-level `boostxl_dual_site`
component. That project generates `C2000Lib_<BOARD>/launchpad_board.h`, which
is the only board-binding header consumed by the portable `src` tree.

For real suite projects, deploy a local manager with:

```bat
%GMP_PRO_LOCATION%\tools\SDPE_v2\gmp_sdpe_deploy_project_mgr.bat <project_dir>
```
