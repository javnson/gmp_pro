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
  ctrl_settings.h
  ctrl_settings_matlab_init.m
```

The generated project header includes selected hardware entities from the
repository-wide `ctl\hardware_preset` tree. Generated files may be ignored
inside GMP, but must be committed after a project is copied into a standalone
repository.

Project id: `mcs_pmsm_nt_f280039c_iris_node`
Suite: `mcs_pmsm_nt`
