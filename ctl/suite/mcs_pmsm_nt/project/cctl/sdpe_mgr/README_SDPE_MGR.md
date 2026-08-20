# SDPE Project Manager

This folder is the project-local SDPE manager.

- `sdpe_requirement.json`: project requirement and hardware bindings.
- `sdpe_settings.bat`: project-local paths and output settings.
- `sdpe_edit.bat`: open this requirement together with every explicitly bound common requirement.
- `sdpe_generate.bat`: generate SDPE headers for this project.
- `sdpe_validate.bat`: validate the central SDPE library and read this requirement file.

The scripts call `%GMP_PRO_LOCATION%\tools\SDPE_v2` and read `%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json`, so set `GMP_PRO_LOCATION` to the GMP repository root before using them.

Default generated output:

```text
sdpe_mgr\
  ctrl_settings.h
  hardware_preset\
```

Project-local generated headers use relative include paths, so the generated project header can include the generated hardware headers without depending on the global `ctl\hardware_preset` output.

Use `common_requirements` in `sdpe_requirement.json` to bind zero or more common
requirement files. Each entry may be relative to this manager, absolute, or based
on an environment variable. The editor merges their hardware, requirements, and
macros with this private project while retaining the source of every item.

Project id: `mcs_pmsm_nt_cctl`
Suite: `mcs_pmsm_nt`
