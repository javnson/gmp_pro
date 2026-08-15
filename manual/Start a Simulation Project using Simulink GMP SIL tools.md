# Start a Simulink SIL project with GMP

This guide describes the current Windows UDP SIL workflow. Start from a
maintained `ctl/suite/<suite>/project/simulate` project; the former
`core/usr/ctl_simulation_mtr_model` and
`tools/facilities_generator/sil_cfg_example` paths no longer exist.

## Prerequisites

1. Install GMP with `install_gmp_virtual_env.bat` or `install_gmp.bat`.
2. Install Visual Studio with the x64 C++ workload for the native controller.
3. Install a supported MATLAB/Simulink release. Models that use Specialized
   Power Systems require MATLAB R2025b or earlier.
4. In MATLAB, run `slib/install_gmp_simulink_lib.m` from
   `GMP_PRO_LOCATION`.

## Project contract

A maintained simulation target normally contains:

| Location | Responsibility |
| --- | --- |
| `*.slx` | Plant model and packed UDP interface |
| `GMP_Motor_Control_simulink.sln` | Native Windows controller solution |
| `network.json` | Host and UDP endpoint configuration |
| `gmp_src_mgr/gmp_framework_config.json` | Selected GMP modules |
| `sdpe_mgr/sdpe_requirement.json` | Simulation-specific parameters |
| `sdpe_mgr/ctrl_settings.h` | Generated C settings |
| `sdpe_mgr/ctrl_settings_matlab_init.m` | Generated MATLAB settings |
| `xplt/xplt.config.h` | Host/CSP feature and buffer-type selection |
| `xplt/xplt.ctl_interface.h` | Packed-buffer-to-controller input/output mapping |
| `xplt/xplt.peripheral.cpp/.h` | Simulation platform storage and peripheral hooks |

The Windows SIL CSP receives one packed UDP request, updates the suite input
buffer, calls `gmp_base_ctl_step()`, and returns the packed output buffer. The
model packing, `gmp_pc_simulink_rx_buffer_t` /
`gmp_pc_simulink_tx_buffer_t` types, and `xplt` mapping form one ABI and must be
changed together.

## Generate and build

Generate suite-common SDPE first, then simulation-target SDPE:

```bat
cd ctl\suite\<suite>\sdpe_general
sdpe_generate.bat
cd ..\project\simulate\sdpe_mgr
sdpe_generate.bat
cd ..\gmp_src_mgr
gmp_generate_inc.bat
gmp_generate_src.bat
```

Build from the GMP environment so MSBuild and vcpkg inherit the configured
toolchain and proxy:

```bat
gmp_env.bat msbuild ctl\suite\<suite>\project\simulate\GMP_Motor_Control_simulink.sln /p:Configuration=Release /p:Platform=x64
```

Use the exact solution name supplied by the chosen suite if it differs.

## Run and verify

Use the suite simulation README or its `run_*_cosim.m` /
`run_*_validation.m` script. These scripts configure model callbacks, load the
common MATLAB initialization script before the target script, start the native
controller, and collect suite-specific evidence. Do not infer a signal layout
from another suite.

If starting manually, launch the native controller and model in the order
specified by that suite. A timeout means the expected UDP peer did not answer;
it is not proof that configuration or controller initialization succeeded.

For the library contract, supported releases, tests, and Linux status, see
[`slib/readme.md`](../slib/readme.md). For transport details, see
[`tools/gmp_sil/sil_helper/README.md`](../tools/gmp_sil/sil_helper/README.md).
