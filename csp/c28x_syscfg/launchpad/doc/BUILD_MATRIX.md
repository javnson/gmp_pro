# LaunchPad build matrix

Validated with CCS 12.8.1, C2000 compiler 22.6.1.LTS, SysConfig 1.21.0,
C2000Ware 5.04.00.00 and GMP-Core-C28x 2.10.00.00 on 2026-08-16.

| Board | Debug | Release | ADC SOCs | DACs | PWM pairs | Hardware run |
|---|---:|---:|---:|---:|---:|---:|
| F2800137C | pass | pass | 16 | 0 | 6 | not run |
| F280025C | pass | pass | 16 | 0 | 6 | not run |
| F280039C | pass | pass | 18 | 2 | 6 | not run in this matrix |
| F280049C | pass | pass | 19 | 2 | 6 | previous baseline; refactor not rerun |
| F28377S | pass | pass | 11 | 3 | 6 | not run |
| F28379D CPU1 | pass | pass | 14 | 3 | 6 | not run |
| F28P55X | pass | pass | 19 | 1 | 6 | not run |
| F28P65X CPU1 | pass | pass | 18 | 2 | 6 | not run |

All 16 configurations invoke SysConfig from CCS and generate files only below
their ignored configuration output directory. All configurations compile their
board-local DriverLib C sources and resolve DriverLib/device headers from the
registered C2000Ware installation; no local DriverLib archive or copied
C2000Ware header is used. The linker uses the default `_c_int00` ELF entry
point, while the Flash `codestart` section still branches through `code_start`.
TI warning #10063 is therefore absent. The two 400-sample, two-channel scope
buffers occupy the portable `mass_data` section, which every board linker file
maps to an available RAM region. F28379D also maps both DriverLib IPC message
sections explicitly. Warning #10247 is absent from all 16 builds; remaining
warnings are confined to TI DriverLib unused-local diagnostics.

Parallel compilation is already enabled on every builder with CCS
`parallelizationNumber="optimal"`. On the 28-logical-processor validation
machine, every clean and build invocation used `gmake -j 28`. The fleet matrix
still runs configurations sequentially because each full build has its own
clean, SysConfig, link and CCS headless phases; low average CPU utilization
during those serial phases does not mean that per-translation-unit parallelism
is disabled.

The generated `board.h` files were also audited for all 16 configurations.
Every target contains six standardized BOOSTXL ePWM instances, the listed ADC
SOC and DAC counts, and an ADCA INT1 definition.  F28379D uses TI's
`F2837xD_337ZWT` package metadata; the earlier 176-pin selection did not match
the LaunchPad.  F28377S SysConfig reports that ADCB4/ADCB5 have no package pin
in its metadata, although the official LaunchPad schematic routes both signals
to BOOSTXL; CCS still generates, compiles and links both configurations.

`ENABLE_PORT` and `OVER_TEMPERATURE` are present on every target.  `RELAY_PORT`
is present on five targets and deliberately omitted on F2800137C (unconnected),
F280025C (external-crystal conflict) and F28379D (pin 33 is not a GPIO).  These
physical-routing exceptions are documented in the project README rather than
being hidden behind conflicting SysConfig assignments.

The CCS configurations also contain two default-off, independent pre-build
variables: `GMP_PREBUILD_SDPE` and `GMP_PREBUILD_SRC_MGR`.  Enabling the first
regenerates the active board's composed Common/Private SDPE outputs; enabling the second refreshes
project-local GMP headers and sources.  Both resolve the repository through
the registered GMP C28x Product rather than a machine-specific checkout path.

Board schemas and entities are registered once from
`csp/c28x_syscfg/sdpe_component`. Eight board-specific requirements under
`launchpad/src/sdpe_mgr/requirements` select one ADC input, one synchronized
PWM output and one available DAC output. F2800137C and F280025C expose `0U`
for the DAC selection and compile the DAC write out. Serial Data Link and CAN
are separate scheduled tasks; their counters and service state remain global
for CCS Expressions inspection.

The build matrix proves compilation and linking, not electrical pin routing on
unconnected hardware. F280049C has debugger, scheduler, live ADC and Data Link
evidence for the previous task/SDPE layout; the current refactor has not yet
been reflashed. See `F280049C_HARDWARE_ACCEPTANCE.md` for that historical run.
