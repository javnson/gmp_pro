# SDPE Built-in Manager Demo

This folder is the project-local SDPE manager for the portable C2000
LaunchPad reference application.

It demonstrates the intended deployment model:

- the SDPE program stays in `%GMP_PRO_LOCATION%\tools\SDPE_v2`;
- the project-local manager stores its own requirement file and batch tools;
- generation and editing are launched through `GMP_PRO_LOCATION`;
- library paths and generation modes are read from `%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json`.

Files:

- `sdpe_requirement.json`: source of truth for the portable BOOSTXL contract,
  ADC/PWM choice and application timing.
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

The generated C header includes the repository-level
`c2000_launchpad_boostxl` preset.  It deliberately does not include a
device-specific LaunchPad preset; CCS supplies the selected board's
`BOOSTXL_*` aliases from its root `.syscfg` during each build.

For real suite projects, deploy a local manager with:

```bat
%GMP_PRO_LOCATION%\tools\SDPE_v2\gmp_sdpe_deploy_project_mgr.bat <project_dir>
```
