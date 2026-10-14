# SDPE Project Manager

This folder is the project-local SDPE manager.

- `sdpe_requirement.json`: project requirement and hardware bindings.
- `sdpe_settings.bat`: project-local paths and output settings.
- `sdpe_edit.bat`: open the SDPE Project Requirement GUI.
- `sdpe_generate.bat`: generate SDPE headers for this project.
- `sdpe_validate.bat`: validate the central SDPE library and read this requirement file.

The scripts call `%GMP_PRO_LOCATION%\tools\SDPE_v2` and read `%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json`, so set `GMP_PRO_LOCATION` to the GMP repository root before using them.

Generated platform output:

```text
sdpe_mgr\
  sdpe_pgs_inv_gfl_iris_settings.h
```

Selected hardware entities are included from the repository-wide `ctl\hardware_preset` output.

Project id: `pgs_inv_gfl_f280039c_iris_node`
Suite: `pgs_inv_GFL_inverter`
