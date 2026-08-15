# Start a C2000 SysConfig project with GMP

This guide describes the repository's current C28x project workflow. Use an
existing target as the template; do not copy the removed `core/usr` layout or
the obsolete `tools/facilities_generator/gmp_file_generator` directory.

## Prerequisites

Install GMP first with `install_gmp_virtual_env.bat` or `install_gmp.bat`. A
hardware build also requires a compatible Code Composer Studio, C2000Ware, and
SysConfig installation. The GMP installer registers `GMP_PRO_LOCATION` and
distributes the current source-manager and SDPE launchers.

## Choose a template

For a CSP-only example, start from
`csp/c28x_syscfg/f280049c_launchpad`. For a complete controller, copy the
closest target below `ctl/suite/<suite>/project`. Keep the target's IDE files,
`gmp_src_mgr`, `sdpe_mgr`, `xplt`, and the suite's shared `src` directory.

The current file boundary is:

| Location | Responsibility |
| --- | --- |
| `xplt/xplt.config.h` | GMP/CSP feature selection and generated settings include |
| `xplt/xplt.ctl_interface.h` | ADC/encoder input conversion, PWM output conversion, and fast enable/disable behavior |
| `xplt/xplt.peripheral.c/.h` | SysConfig board startup, ISR, UART, ADC, PWM, GPIO, and other target peripherals |
| `sdpe_mgr/sdpe_requirement.json` | Target-specific clocks, channels, gains, polarity, and build selections |
| `sdpe_mgr/ctrl_settings.h` | Generated project configuration header; do not edit by hand |
| `src/ctl_main.h/.c` | Shared controller objects, initialization, and dispatch |
| `src/user_main.h/.c` | Shared application and background tasks |

In the standard CTL framework, the periodic ISR calls `gmp_base_ctl_step()`,
which runs `ctl_input_callback()`, `ctl_dispatch()`, and
`ctl_output_callback()` in that order. A project using the Nano framework has
the separate `ctl_fmif_*` interface defined by `ctl/framework/ctl_nano.h`.

## Generate configuration and sources

If the suite has common SDPE data, generate it before the target data:

```bat
cd ctl\suite\<suite>\sdpe_general
sdpe_generate.bat
cd ..\project\<target>\sdpe_mgr
sdpe_generate.bat
```

Then run the target's source manager:

```bat
cd ..\gmp_src_mgr
gmp_generate_inc.bat
gmp_generate_src.bat
```

`gmp_inc` mirrors repository-relative header paths. `gmp_src` contains the
selected C/C++ sources in one flat directory, and
`gmp_compiler_includes.txt` records the generated include paths. Change module
selection with `gmp_config.bat`; do not patch generated files as the upstream
fix.

## Import and verify

Import the copied target into CCS, regenerate SysConfig output if the `.syscfg`
file changed, and build the exact configuration used by the target. Before
energizing hardware, verify the selected `BUILD_LEVEL`, ADC scaling and bias,
phase direction, PWM polarity, dead time, trip behavior, and output-enable
ordering. A successful PC build does not validate C28x compiler or hardware
behavior.

For existing C28x platform details, see
[`csp/c28x_syscfg/readme.md`](../csp/c28x_syscfg/readme.md) and the target's own
README.
