# F29H85x motor target SDPE manager

This directory is the target layer of the `mcs_pmsm_nt` two-layer SDPE model.
The requirement inherits the suite-wide controller and PIL requirements, then
binds them to the `launchxl_f29h85x_3phgan`, `ti_boostxl_3phganinv`, and
`sm060r20b30mnad` hardware entities.

Run `sdpe_edit.bat` to edit the project, `sdpe_generate.bat` after every change,
and `sdpe_validate.bat` before building. The generated C header and MATLAB init
file are checked-in target artifacts. Do not hand-edit their macros; edit
`sdpe_requirement.json` or the corresponding hardware entity instead.
