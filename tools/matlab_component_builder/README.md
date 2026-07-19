# GMP MATLAB Component Builder

**English** | [简体中文](README_CN.md)

`matlab_component_builder` turns a real GMP C control component into an independently installed, masked Simulink block. It does not depend on or modify the existing GMP `slib` library.

The current catalog includes basic hysteresis/saturation/slope limiting, continuous PI/PID/LADRC1/LADRC2/SOGI, the PR family, lead/lag, biquad and first-order filters, 1P1Z/2P2Z/3P3Z, and advanced RC/FDRC. See the [component rollout ledger](docs/COMPONENT_ROLLOUT_CN.md) for the remaining queue.

## Source and generated boundaries

| Path | Role |
| --- | --- |
| `components/*.json` | Authoritative user component definitions. |
| `schemas/` | Definition format documentation. |
| `templates/` | Authoritative generated-code templates. |
| `python/gmp_mcb/` | Validation, generation, and PyQt editor. |
| `matlab/+gmp_mcb/` | MATLAB build, library, mask, model, and measurement package. |
| `build/` | Generated C++ source and registry; ignored. |
| `install/<Release>/` | Generated MEX and Simulink library; ignored. |
| `cache/` | Frequency measurement results; ignored. |

All GMP source/header locations in JSON are relative to `GMP_PRO_LOCATION`. Generated files may contain resolved paths for the local build but are never authoritative inputs.

## Quick start

Activate or install GMP first so that `GMP_PRO_LOCATION` is defined.

Open the editor:

```bat
tools\matlab_component_builder\run_builder.bat
```

Or validate and generate from the command line:

```bat
cd /d %GMP_PRO_LOCATION%\tools\matlab_component_builder
python matlab_component_builder.py validate
python matlab_component_builder.py generate
```

Install for the active MATLAB Release:

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), 'tools', ...
    'matlab_component_builder', 'matlab', ...
    'install_gmp_matlab_components.m'));
```

The MATLAB installer now invokes the canonical GMP environment guard. A completed
private installation uses `bin/python/python.exe`; otherwise it uses the system
Python provisioned by `install_gmp.bat`. MATLAB does not need to be launched from
an activated GMP command prompt. Missing Jinja2 diagnostics identify the selected
interpreter and direct the user to the appropriate GMP repair entry point; the
feature installer never performs an ad-hoc `pip install`.

The editor presents all definitions in a category tree. It manages scalar MIMO port tables, parameter groups and external-input eligibility, and previews generated C++ before generation. Open **GMP MATLAB Components** in the Simulink Library Browser after installation.

Each block mask has separate **Parameters** and **Simulation Analysis** tabs. An externalizable parameter has a checkbox: off uses its fixed Mask value; on disables that editor and adds a Simulink input port in parameter-table order. PID gains and both output/integrator limits support this gain-scheduling interface. Initialization frequency `fs` remains an initialization-only parameter.

To remove the registered MATLAB paths:

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), 'tools', ...
    'matlab_component_builder', 'matlab', ...
    'uninstall_gmp_matlab_components.m'));
```

Uninstalling removes paths but retains generated files for inspection. Delete the ignored `install/`, `build/`, or `cache/` directories manually when a clean rebuild is desired.

## Scheduling and `fs`

The generated S-Function uses inherited sample time. The `fs` mask value is passed to the GMP initialization function and does not force Simulink scheduling. A triggered or function-call subsystem can therefore execute the controller at a slower rate than the surrounding plant model.

The **Simulation Analysis** tab separately owns execution frequency, sweep range, point count, excitation amplitude, and measurement durations. The plot and measurement harness use the analysis execution frequency to map sample index to physical frequency. It is not passed to the controller and does not schedule the block. Set it to the actual trigger rate when that rate differs from initializer `fs`.

## PID model definitions

For parallel initialization, the exact implemented response is

```text
Kp + (Ki/fs_parameter)/(1-z^-1) + (Kd*fs_parameter)(1-z^-1)
```

where `z^-1 = exp(-j*2*pi*f/fs_execution)`.

For time-constant initialization, the implementation follows the current `ctl_step_pid_ser` code. Its derivative term is not multiplied by `Kp`; the analyzer intentionally preserves and displays that behavior.

Every SISO component retains **Measure and plot simulated frequency response**. The action creates a temporary discrete Simulink harness and runs a coherent sequential sine sweep. Components with analytical models overlay the continuous, implementation-reference, and measured MEX responses. Components without analytical models show a clean measured-only magnitude/phase plot. Results are stored under `cache/`.

For nonlinear components such as saturation and hysteresis, the measured response depends on excitation amplitude, bias, and settled operating condition; it is not a unique linear transfer function. Plot titles and cached results record the excitation, execution frequency, and settling/measurement durations.

For time-domain MIMO components such as LADRC, the **Simulation Analysis** tab exposes the excited input, observed output, and steady operating value for every input. The harness adds the sine only to the selected input, holds all other inputs at their configured operating values, and plots the local response for the selected input/output pair.

RC and FDRC use a fixed **Locked frequency** input during the entire sweep. At every sweep point, the error sine is first applied for the configured number of pre-learning periods (measured in locked-frequency periods), then the response is estimated over a separate coherent measurement window. This measures the conventional fixed-period repetitive-controller response; the lock frequency never follows the sweep frequency.

## Current scope

- SISO and scalar MIMO `double` Simulink ports with GMP `ctrl_gt` internally.
- PID template with parallel and time-constant initialization.
- R/PR/QR/QPR template; QR and QPR support standard and prewarped Tustin initialization.
- Parameter groups and selectable fixed-Mask/external-port values.
- Double-buffered dynamic workspaces committed in `mdlUpdate` and released in `mdlTerminate`.
- Inherited Simulink scheduling.
- Host MEX simulation on the active MATLAB platform.
- Ideal/implementation plots and sequential-sine measurement, including fixed-lock RC pre-learning.

MIMO, generic state probes, fixed-point MEX variants, Simulink Coder deployment, and separate boundary-condition testbenches are future extensions. A successful host MEX test is not hardware validation.

## Validation

Python checks:

```bat
python -m unittest discover -s tests -v
```

MATLAB checks require a configured MEX compiler. The implementation has been exercised with MATLAB R2024b and Microsoft Visual C++ 2022 by compiling all 22 MEX files, creating/loading the library and two-tab masks, checking PID first-sample and external-port behavior, and checking measured QPR, RC, and biquad responses.
