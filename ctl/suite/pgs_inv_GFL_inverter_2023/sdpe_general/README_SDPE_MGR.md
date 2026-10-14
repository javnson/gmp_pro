# PGS GFL Inverter Common SDPE Manager

This manager owns the platform-independent controller contract shared by all
implementations under `project/`.

- `sdpe_requirement.json`: grid, P/Q-loop and commissioning parameters.
- `sdpe_edit.bat`: opens the common requirement and all four platform requirements.
- `sdpe_generate.bat`: generates the common header into `src/`.
- `sdpe_validate.bat`: validates the SDPE library and common requirement.

Generated output:

```text
src\sdpe_pgs_inv_gfl_common_settings.h
```

Project id: `pgs_inv_gfl_common`  
Macro prefix: `PGS_INV_GFL_COMMON`
