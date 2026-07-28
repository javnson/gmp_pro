# PGS GFM inverter common SDPE manager

This manager owns the platform-independent contract shared by all targets.

- `sdpe_requirement.json` defines voltage/current commissioning references,
  voltage-loop limiters, droop parameters, PLL-to-GFM transition settings,
  sequence control, and the 3D-SVPWM option.
- Each `project/<target>/sdpe_mgr/sdpe_requirement.json` defines BUILD_LEVEL,
  sampling, PWM, sensing, protection, and hardware bindings.
- `sdpe_generate.bat` generates
  `src/sdpe_pgs_inv_gfm_common_settings.h` and the MATLAB initialization file.
- `sdpe_validate.bat` validates the common requirement against SDPE schemas.

Project id: `pgs_inv_gfm_common`  
Macro prefix: `PGS_INV_GFM_COMMON`

