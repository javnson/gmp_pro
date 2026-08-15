# PGS two-stage single-phase inverter suite

This suite contains the MATLAB/Simulink models for the two-stage single-phase
inverter. The simulation project is under `project/simulate`.

SDPE is the canonical source of physical, sensing, protection, PWM and runtime
parameters:

- `sdpe_general/sdpe_requirement.json` defines the cross-platform contract.
- `project/simulate/sdpe_mgr/sdpe_requirement.json` defines simulation-specific
  bindings.
- Generated `.h` and `.m` files must be refreshed in that order: common first,
  then the simulation target.

The models initialize `project/simulate/sdpe_mgr/ctrl_settings_matlab_init.m`
from path-independent callbacks and reference the resulting SDPE workspace
variables in their plant masks.
