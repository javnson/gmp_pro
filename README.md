# General Motor Platform (GMP)

**English** | [简体中文](README_CN.md)

GMP is a cross-platform development framework for real-time control, motor
drives, and power-electronics systems. The repository combines reusable control
components, hardware abstraction, runnable control suites, parameter-engineering
tools, and MATLAB/Simulink software-in-the-loop (SIL) support.

GMP can be used as the complete Core+CSP+CTL platform, or as a lightweight
**CTL Portable (No CSP)** library embedded into an existing TI DSP, STM32, or
other C/C++ firmware project. Portable applications keep their current startup,
BSP, HAL, RTOS, and peripheral code: **wherever control is needed, GMP-CTL can
be deployed.**

![GMP Logo](manual/img/GMP_LOGO.png)

GMP is designed to reuse one control implementation across PC simulation, TI
C2000, STM32, and other targets while keeping these concerns separate:

- platform-independent control algorithms and state machines;
- ADC, PWM, encoder, UART, and other platform peripherals;
- circuit, sensor, controller, and protection parameters;
- the subset of GMP sources required by an individual project;
- Simulink plant models and the controller executable.

> GMP currently targets Windows development hosts. Hardware projects also need
> the appropriate vendor toolchain, such as Code Composer Studio, SysConfig,
> STM32CubeMX/CubeIDE, or Keil. Simulation projects require MATLAB/Simulink and,
> for current models, Simscape Electrical Specialized Power Systems.

## 1. Quick start

### 1.1 Clone the repository

Clone submodules recursively:

```bat
git clone --recursive <repository-url> gmp_pro
cd gmp_pro
```

For an existing non-recursive clone, run:

```bat
git submodule update --init --recursive
```

Keep the repository in a path without spaces, non-ASCII characters, or an
excessive path length.

### 1.2 Install the development environment

The recommended private environment installs Python, Git, CMake, Ninja,
Doxygen, Graphviz, vcpkg, and the required Python packages under `bin`:

```bat
install_gmp_virtual_env.bat
```

For compatibility with user-scoped system tools and Scoop, use:

```bat
install_gmp.bat
```

Both modes register:

```text
GMP_PRO_LOCATION=<absolute gmp_pro root>
```

After installation, open the GMP development prompt:

```bat
gmp_env.bat
```

It can also run one command directly:

```bat
gmp_env.bat python --version
gmp_env.bat cmake --version
```

See the [GMP environment installer guide](tools/gmp_installer/README.md) for
online installation, copied-bin deployment, proxy configuration, dependency
repair, and maintainer rules.

### 1.3 Install the Simulink library

Run this command in MATLAB:

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), ...
    'slib', 'install_gmp_simulink_lib.m'));
```

The installer deploys and registers the library for the active MATLAB release.
Current GMP models require Specialized Power Systems. MathWorks removed that
library in R2026a, so use MATLAB R2025b or earlier with Simscape Electrical and
Specialized Power Systems installed.

### 1.4 Start from a verified suite

Start a new application from the closest project in `ctl/suite` instead of
assembling an empty project manually:

| Application | Recommended suite | Notes |
| --- | --- | --- |
| Four-switch buck-boost DC-DC | [`dps_fsbb`](ctl/suite/dps_fsbb) | C2000 hardware, SDPE configuration, and UDP/SIL validation assets. |
| Bidirectional isolated CLLLC / DAB | [`dps_clllc`](ctl/suite/dps_clllc) | F280025C target with variable-frequency and phase-shift PWM support. |
| PMSM vector control | [`mcs_pmsm_nt`](ctl/suite/mcs_pmsm_nt) | Current PMSM template for C2000, STM32, and simulation. |
| PMSM parameter identification | [`mcs_pmsm_id`](ctl/suite/mcs_pmsm_id) | Motor parameter-identification workflow. |
| Three-phase grid-following converter | [`pgs_inv_GFL_inverter`](ctl/suite/pgs_inv_GFL_inverter) | Grid-following inverter control project. |
| Single-phase rectifier/inverter | [`pgs_sinv_rc`](ctl/suite/pgs_sinv_rc) | BUILD_LEVEL 1-5 workflow and repetitive control. |

`mcs_pmsm` and parts of `mcs_acm` use an older project layout. Keep them for
compatibility work, but prefer the newer templates for new applications.

### 1.5 Add CTL to an existing project

An application with an established BSP, HAL, RTOS, or vendor project does not
need to port a complete CSP merely to use CTL PID, filters, coordinate
transforms, modulators, or observers. Select the
[TI DSP or STM32 Portable template](ctl/portable/README.md), then:

1. copy the selected target directory into the application and add it together
   with the GMP root to the include paths;
2. manage the numeric backend with its `sdpe_requirement.json` and run
   `sdpe_generate.bat` to regenerate the configuration header;
3. define `GMP_CTL_PORTABLE` globally, or force-include the SDPE-generated
   header;
4. include `gmp_core.h`, include the required CTL headers, and compile only the
   corresponding CTL sources.

```c
#include <gmp_core.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>
```

In this mode, `gmp_core.h` bypasses `xplt.config.h`, CSP, the GMP device layer,
and runtime entry framework. Pure algorithm modules require no peripheral
hooks; only time-based components such as delays and calibration routines need
a system tick implementation.

## 2. Repository layout

| Directory | Purpose |
| --- | --- |
| [`core`](core) | Core runtime: scheduling, devices, memory, communication, and base services. |
| [`csp`](csp) | Chip Support Packages and host/target platform implementations. |
| [`ctl`](ctl) | Control Template Library: math blocks, components, lightweight Portable integration, frameworks, and suites. |
| [`cctl`](cctl) | C++ control and power-electronics objects. |
| [`vcore`](vcore) | HDL/Verilog support and experimental platforms. |
| [`slib`](slib) | MATLAB/Simulink library sources, installer, and release-specific output. |
| [`tools`](tools) | SDPE, source management, SIL/PIL, debugger, and installer tools. |
| [`quick_start`](quick_start) | Small examples and source-generation demonstrations. |
| [`manual`](manual) | User guides, coding conventions, and platform workflows. |
| [`third_party`](third_party) | Third-party sources maintained directly in this repository. |

The common C and C++ entry points are:

```c
#include <gmp_core.h>
```

```cpp
#include <gmp_core.hpp>
```

Applications normally use their local `gmp_src_mgr` to select and generate the
required subset instead of compiling every GMP source file.

## 3. Suite project model

Recommended suites use this layout:

```text
ctl/suite/<suite>/
├── src/                         shared control algorithms and user logic
├── sdpe_general/                cross-platform SDPE parameters
├── project/
│   ├── simulate/                PC controller + Simulink SIL project
│   │   ├── sdpe_mgr/            simulation-specific parameters
│   │   ├── gmp_src_mgr/         selected GMP dependencies
│   │   └── xplt/                simulation platform adapter
│   └── <hardware>/              target hardware project
│       ├── sdpe_mgr/            board, chip, and peripheral parameters
│       ├── gmp_src_mgr/         selected GMP dependencies
│       └── xplt/                hardware platform adapter
└── doc/                         reports, waveforms, and archived results
```

Responsibilities are intentionally separated:

- `src` contains control objects, state machines, ISR control logic, and
  Datalink user tasks without direct register access.
- `xplt` implements ADC inputs, PWM outputs, peripheral initialization, fast
  enable/disable behavior, UART, and other platform interfaces.
- `sdpe_general` defines topology, ratings, per-unit bases, control bandwidths,
  and protection thresholds shared by all targets.
- `project/<target>/sdpe_mgr` defines clocks, ADC/PWM resources, board wiring,
  and target-specific switches.
- `gmp_src_mgr` selects and synchronizes the GMP components required to build.

A typical real-time call chain is:

```text
main control ISR
  -> ctl_input_callback()       convert samples into control values
  -> gmp_base_ctl_step()
       -> ctl_dispatch()        run platform-independent control
  -> ctl_output_callback()      write PWM and other actuators
```

When porting hardware, change the target `xplt` and platform SDPE configuration
first. Change shared `src` only when the control strategy itself changes.

## 4. SDPE parameter engineering

SDPE (System Design and Parameter Engineering) describes power stages, motors,
sensors, boards, controller settings, and project parameters, then generates C
headers and MATLAB initialization scripts.

For suites using two SDPE layers:

1. `sdpe_general\sdpe_edit.bat` opens the common configuration and all project
   configurations in that suite.
2. `project\<target>\sdpe_mgr\sdpe_edit.bat` opens the common and selected
   target configurations together.
3. Run `sdpe_generate.bat` in both layers after editing.
4. Run `sdpe_validate.bat` to validate requirements and generated output.

Simulink resolves the two generated initialization scripts relative to the
model:

```text
../../sdpe_general/*_matlab_init.m
./sdpe_mgr/*_matlab_init.m
```

Each layer must contain exactly one generated MATLAB initialization script.
Never store developer-machine absolute paths in model callbacks or SDPE data.

Deploy the latest project SDPE launchers with:

```bat
tools\SDPE_v2\gmp_sdpe_deploy_project_mgr.bat
```

## 5. GMP source manager

Each independently buildable project normally contains `gmp_src_mgr`.
`gmp_framework_config.json` selects GMP modules. Generation produces a mirrored
header tree, flattened C/C++ sources, and a compiler include-path list.

Typical commands are:

```bat
project\<target>\gmp_src_mgr\gmp_config.bat
project\<target>\gmp_src_mgr\gmp_generate_inc.bat
project\<target>\gmp_src_mgr\gmp_generate_src.bat
```

Some IDE projects place `gmp_src_mgr` deeper, for example under
`project/<target>/src`. Use the scripts supplied by the target project. After a
GMP component changes, regenerate the target instead of editing an old generated
copy.

## 6. SIL co-simulation

In the standard GMP SIL architecture, the controller runs as a native PC
program while Simulink executes the plant and sensor model. UDP exchanges ADC,
PWM, state, and debug data:

```text
Simulink plant model
  <-> GMP SIL block / UDP helper
  <-> project/simulate controller executable
  <-> suite/src control algorithm
```

Recommended order:

1. Configure and generate both `sdpe_general` and simulation `sdpe_mgr` data.
2. Use `gmp_src_mgr` to synchronize dependencies and build the simulation
   controller.
3. Open the suite `.slx` model and verify that both SDPE initialization scripts
   execute.
4. Start the controller program, then start the Simulink simulation.
5. Validate each documented `BUILD_LEVEL`, from open-loop checks to closed-loop
   operation.

Suite-specific wiring, acceptance criteria, and launch procedures remain
authoritative for their own models. The [Chinese manual](README_CN.md) links the
currently available Chinese validation reports.

## 7. PIL, Datalink, and online debugging

- PIL runs control code on a real MCU while exchanging simulation input and
  output with the host.
- Datalink supports variable monitoring, online parameter changes, Memory
  Perspective, and PIL data exchange.
- The [headless Data Link Python API](tools/gmp_pil_server/gmp_debugger/apis/README.md)
  lets test programs and AI agents discover the Tunable table and memory
  whitelist, perform typed parameter and bounded memory access, configure Scope
  triggers, acquire continuous waveforms, execute exact PIL controller steps,
  bridge the standard Simulink UDP vectors, and export evidence to CSV. A
  [Chinese API manual](tools/gmp_pil_server/gmp_debugger/apis/README_CN.md) is
  provided alongside the English reference.
- Use `tools/gmp_pil_server/gmp_debugger/run_u8.bat` for byte-addressed CPUs and
  `tools/gmp_pil_server/gmp_debugger/run_u16.bat` for 16-bit-addressed DSP
  targets. Both launch the same maintained frontend and wire codec.
- Suite `user_main.c` files normally organize communication and background
  tasks; the target `xplt` owns UART and other physical interfaces.

Do not perform blocking serial or GUI communication in a high-frequency control
ISR. Move debug communication into scheduled background tasks.

## 8. Development conventions and English guides

- Put shared algorithms in suite `src` and chip/board differences in
  `project/<target>/xplt`.
- Put configurable physical values in SDPE instead of scattering unexplained
  constants through `ctl_main.c`.
- Simulation and hardware targets should share control sources and common
  parameter definitions.
- Update `gmp_framework_config.json` and regenerate after adding a dependency.
- Never commit drive-specific absolute paths in code, JSON, scripts, or model
  callbacks.
- Edit `slib/simulink_lib_src`, not installed copies under `slib/install_path`.
- Treat project SDPE and source-manager launchers as distributed copies inside
  the main repository; update their canonical templates under `tools`.
- Validate controllers progressively from sampling, PWM, and open-loop behavior
  through protection checks and closed-loop operation.
- Before committing, run the relevant generators, target build, simulation
  smoke test, and `git diff --check`.

English documentation:

- [Development environment and installer guide](tools/gmp_installer/README.md)
- [C29x SysConfig peripheral guide](csp/c29x_syscfg/doc/readme.md)
- [MATLAB/Simulink library and SIL guide](slib/readme.md)
- [Start a SysConfig project using GMP](manual/Start%20a%20Sysconfig%20Project%20using%20GMP.md)
- [Start a Simulink GMP SIL project](manual/Start%20a%20Simulation%20Project%20using%20Simulink%20GMP%20SIL%20tools.md)
- [CLLLC / DAB commissioning guide](ctl/suite/dps_clllc/doc/commissioning.md)
- [AI repository maintenance skill](.agents/skills/maintain-gmp-repository/SKILL.md)

For Chinese guides and the Chinese suite/component documentation index, switch
to the [简体中文说明](README_CN.md).

## 9. TI CCS Product installation and upgrades

GMP provides one CCS Product per TI device family. Run the data-driven
installer after initial installation, moving the repository, switching GMP
versions, or changing Product metadata:

```bat
tools\facilities_generator\ccs_product_installer\install_ccs_products.bat
```

Close CCS, remove any obsolete GMP repository-root entry from the CCS Product
discovery paths, and add the applicable CSP directories:

| Family | Product | Discovery path | Exported project macros |
| --- | --- | --- | --- |
| C28x | `GMP-Core-C28x` | `gmp_pro\csp\c28x_syscfg` | `GMP_PRO_ROOT`, `GMP_C28X_CSP_ROOT` |
| C29x | `GMP-Core-C29x` | `gmp_pro\csp\c29x_syscfg` | `GMP_PRO_ROOT`, `GMP_C29X_CSP_ROOT` |

Refresh/discover Products and restart CCS before importing or rebuilding a
project. Do not retain the obsolete `GMP-PRO-SDK` dependency. C28x targets may
use CCS 18; C29x/F29H85x targets require CCS 21 or newer.

## 10. CTL Portable and Keil Pack

CTL Portable is the supported lightweight deployment model for applications
that retain their existing infrastructure and only import control capability.
It uses the same CTL APIs and module sources as complete GMP projects. An
application may later move into a cross-platform suite, but Portable does not
require such a migration.

| Capability | Complete GMP + CSP | CTL Portable (No CSP) |
| --- | --- | --- |
| CTL math and control components | Supported | Supported |
| `ctrl_gt`/`parameter_gt` backend | Platform configuration and SDPE | Portable SDPE template |
| Startup and ADC/PWM/UART abstraction | CSP/xplt | Existing application BSP/HAL |
| GMP lifecycle, device layer, debug tool | Included | Not loaded |
| Typical use | Cross-platform suites, SIL/PIL, unified hardware framework | Add selected CTL modules to existing firmware |

The [`ctl/portable`](ctl/portable/README.md) contract currently provides TI
C28x/C29x and STM32 templates. It exposes only the required numeric selections,
raw ADC/DAC/PWM types, assertion mapping, and optional system tick. Peripheral
registers, RTOS integration, HAL code, and application lifecycle remain owned
by the host project.

GMP-CTL can therefore be distributed module by module without copying the full
repository into the final firmware. The source manager selects files from the
registry, and Keil users can compile the same registry into a CMSIS-Pack.

Keil users can build a CMSIS-Pack from the authoritative source-manager
registry with:

```bat
tools\facilities_generator\keil_pack\build_keil_pack.bat
```

Artifacts are written to `build\keil_pack`. See the
[Keil Pack generator guide](tools/facilities_generator/keil_pack/README.md).

## 11. License

GMP is licensed under the [Apache License 2.0](LICENSE.txt). See [NOTICE](NOTICE)
and component directories for third-party copyright and license information.
