# MCS PMSM NT Common SDPE Manager

This manager owns the platform-independent controller contract shared by every
implementation under `project/`.

- `sdpe_requirement.json`: common control parameters, feature switches and
  `BUILD_LEVEL` semantics. It intentionally contains no board entity.
- `sdpe_settings.bat`: project-local paths and output settings.
- `sdpe_edit.bat`: open the common requirement and all seven platform requirements in one SDPE Project Requirement GUI.
- `sdpe_generate.bat`: generate SDPE headers for this project.
- `sdpe_validate.bat`: validate the central SDPE library and read this requirement file.

The scripts call `%GMP_PRO_LOCATION%\tools\SDPE_v2` and read `%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json`, so set `GMP_PRO_LOCATION` to the GMP repository root before using them.

Generated output:

```text
src\
  sdpe_mcs_pmsm_nt_common_settings.h
```

Each project generation combines the common requirement with target timing,
sensor scaling and peripheral mappings in its generated
`sdpe_mgr/ctrl_settings.h`. Shared controller sources consume the separately
generated common header through `ctl_settings_defaults.h`; that compatibility
include no longer owns a second set of defaults.

Project id: `mcs_pmsm_nt_common`
Macro prefix: `MCS_PMSM_NT_COMMON`
Suite: `mcs_pmsm_nt`
