# SDPE Project Manager

This folder is the project-local SDPE manager.

- `sdpe_requirement.json`: project requirement and hardware bindings.
- `sdpe_settings.bat`: project-local paths and output settings.
- `sdpe_edit.bat`: open the SDPE Project Requirement GUI.
- `sdpe_generate.bat`: generate SDPE headers for this project.
- `sdpe_validate.bat`: validate the central SDPE library and read this requirement file.

The scripts call `%GMP_PRO_LOCATION%\tools\SDPE_v2`, so set `GMP_PRO_LOCATION` to the GMP repository root before using them.

Default generated output:

```text
..\xplt\sdpe_generated
```

Project id: `pgs_sinv_rc_iris_node`
Suite: `pgs_sinv_rc`
