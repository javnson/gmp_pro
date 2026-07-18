# GMP MATLAB Component Builder

**English** | [简体中文](README_CN.md)

`matlab_component_builder` turns a real GMP C control component into an independently installed, masked Simulink block. It does not depend on or modify the existing GMP `slib` library.

The first supported template is a SISO PID using the implementations in `ctl/component/intrinsic/continuous/continuous_pid.h`. The generated block supports the parallel-gain and time-constant initializers, executes the original GMP step function, plots the ideal and exact implemented frequency models, and can measure the compiled MEX block with a coherent sine sweep.

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

Open **GMP MATLAB Components** in the Simulink Library Browser, add the PID block, and open its mask. The mask provides controls to plot the reference models and measure the compiled block.

To remove the registered MATLAB paths:

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), 'tools', ...
    'matlab_component_builder', 'matlab', ...
    'uninstall_gmp_matlab_components.m'));
```

Uninstalling removes paths but retains generated files for inspection. Delete the ignored `install/`, `build/`, or `cache/` directories manually when a clean rebuild is desired.

## Scheduling and `fs`

The generated S-Function uses inherited sample time. The `fs` mask value is passed to the GMP initialization function and does not force Simulink scheduling. A triggered or function-call subsystem can therefore execute the controller at a slower rate than the surrounding plant model.

The mask has a separate **Analysis/execution frequency** expression, defaulting to `fs`. Frequency plots and the generated measurement harness use this value to map sample index to physical frequency. Set it to the actual trigger rate when it differs from the initializer's `fs` parameter.

## PID model definitions

For parallel initialization, the exact implemented response is

```text
Kp + (Ki/fs_parameter)/(1-z^-1) + (Kd*fs_parameter)(1-z^-1)
```

where `z^-1 = exp(-j*2*pi*f/fs_execution)`.

For time-constant initialization, the implementation follows the current `ctl_step_pid_ser` code. Its derivative term is not multiplied by `Kp`; the analyzer intentionally preserves and displays that behavior.

The measurement button creates a temporary discrete Simulink harness, copies the selected masked block, runs a coherent sequential sine sweep, compares measured values with the implementation reference, and stores a MAT result under `cache/`.

## Current scope

- SISO, scalar `double` Simulink ports with GMP `float` `ctrl_gt` internally.
- PID-specific code template.
- Parallel and time-constant initialization modes.
- Inherited Simulink scheduling.
- Host MEX simulation on the active MATLAB platform.
- Ideal/implementation plots and sequential-sine measurement.

MIMO, generic state probes, fixed-point MEX variants, Simulink Coder deployment, arbitrary component templates, and separate boundary-condition testbenches are future extensions. A successful host MEX test is not hardware validation.

## Validation

Python checks:

```bat
python -m unittest discover -s tests -v
```

MATLAB checks require a configured MEX compiler. The implementation has been exercised with MATLAB R2024b and Microsoft Visual C++ 2022 by compiling the PID MEX, creating/loading the library, checking first-sample behavior, and comparing measured parallel/T-mode responses against the exact difference equations.

