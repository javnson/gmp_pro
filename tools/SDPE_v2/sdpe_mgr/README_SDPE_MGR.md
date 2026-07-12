# SDPE Built-in Manager Demo

This folder is a complete project-local SDPE manager example.

It demonstrates the intended deployment model:

- the SDPE program stays in `%GMP_PRO_LOCATION%\tools\SDPE_v2`;
- the project-local manager stores its own requirement file and batch tools;
- generation and editing are launched through `GMP_PRO_LOCATION`;
- library paths and generation modes are read from `%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json`.

Files:

- `sdpe_requirement.json`: demo project requirement file.
- `sdpe_settings.bat`: local settings for this manager.
- `sdpe_edit.bat`: open the Project Requirement GUI.
- `sdpe_generate.bat`: generate headers into `build`.
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
  sdpe_demo_bindings.h
  hardware_preset\
```

Project-local generated headers use relative include paths.

For real suite projects, deploy a local manager with:

```bat
%GMP_PRO_LOCATION%\tools\SDPE_v2\gmp_sdpe_deploy_project_mgr.bat <project_dir>
```
