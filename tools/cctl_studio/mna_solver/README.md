# CCTL Studio MNA Solver

[中文说明](README_CN.md)

This directory contains the first Python implementation of the CCTL Studio
linear-circuit solver. It converts a supported SPICE/TINA netlist through this
pipeline:

```text
netlist -> exact symbolic MNA -> numeric descriptor MNA
        -> algebraic elimination -> state/output model
        -> discretization -> time simulation or frequency response

TINA D/M netlist -> four commanded switch topologies
                 + two MOS body-diode topologies
                 -> previous-sample voltage selection -> switching simulation
```

The descriptor equation is

```text
E z_dot = A z + B u
```

where `z` contains non-ground node voltages and the extra MNA branch currents.
An SVD rank decomposition followed by an index-1 algebraic elimination produces

```text
x_dot = A x + B u
y     = C x + D u + F u_dot
```

`F` is normally zero. It is retained because observing a capacitor current can
mathematically depend on the derivative of an externally imposed voltage. This
makes that exceptional case explicit instead of silently returning an incorrect
`C/D` equation.

## Dependencies

- NumPy: numeric reduction, iteration, and frequency response.
- SymEngine: exact symbolic MNA matrices.
- Eigen3: fixed-size matrix operations in generated C++ classes.

Python packages are pinned in `tools/gmp_installer/requirements-gmp.txt`.
Eigen3 is declared by this directory's `vcpkg.json` and restored into GMP's
shared vcpkg tree by the installer. The deprecated `third_party/eigen` path is
not used.

## Supported netlist subset

| Element | Syntax | Meaning |
| --- | --- | --- |
| `R`, `L`, `C` | `R1 n+ n- value` | Passive element |
| `V`, `I` | `V1 n+ n- value` | Independent input source |
| `O` | `O1 n+ n- nout` | Ideal op amp |
| `E`, `G` | `E1 n+ n- nc+ nc- gain` | VCVS / VCCS |
| `F`, `H` | `F1 n+ n- Vctrl gain` | CCCS / CCVS |
| TINA current arrow | `VAM1 n+ n- ; Current Arrow` | Zero-voltage current probe |
| `D` | `D1 anode cathode model` | PWL diode in the switched solver |
| `M` | `M1 drain gate source bulk model` | PWL NMOS in the switched solver |
| TINA ideal op amp | `X1 n+ n- nout IdOpamp` | Three-pin ideal op amp |

Node names may be numeric or textual. `0` and `GND` are ground. SPICE suffixes
such as `K`, `MEG`, `M`, `U`, and `N` are accepted. `Symbolic` and other
non-numeric values are represented by the element name in SymEngine and must be
provided with `--param` before numeric reduction.

`.LIB`, `.TEMP`, `.AC`, `.TRAN`, and unrelated analysis directives are ignored.
`.PROBE`, `.PRINT`, and `.SAVE` register `V(node)`, `V(node+,node-)`, and
`I(element)` outputs. If there is no output directive, every non-ground node
voltage is selected.

## Command line

Run these commands from the repository root.

Print exact symbolic MNA matrices and the numeric state/output matrices:

```bat
python tools\cctl_studio\mna_solver\mna_solver.py analyze ^
  tools/cctl_studio/mna_solver/tb/0_divider.CIR
```

For symbolic passive or controlled-source values, repeat `--param`:

```bat
python tools\cctl_studio\mna_solver\mna_solver.py analyze ^
  tools\cctl_studio\mna_solver\tb\example5.cir ^
  --param R1=1K --param R2=10K
```

List the forward-Euler matrices and scalar `x[k+1]`/`y[k]` expressions:

```bat
python tools\cctl_studio\mna_solver\mna_solver.py discretize ^
  tools\cctl_studio\mna_solver\tb\example6.cir --dt 1N
```

Forward-Euler simulation writes a CSV file. Independent source values in the
netlist are the defaults; `--input` overrides them and is required for a source
whose value is symbolic.

```bat
python tools\cctl_studio\mna_solver\mna_solver.py simulate ^
  tools\cctl_studio\mna_solver\tb\example6.cir ^
  --dt 1N --duration 10U --input Vin=1 ^
  --output tools\cctl_studio\mna_solver\tb\example6_result.csv
```

Generate magnitude, dB magnitude, and phase for every input/output pair:

```bat
python tools\cctl_studio\mna_solver\mna_solver.py frequency ^
  tools\cctl_studio\mna_solver\tb\example6.cir ^
  --start 10 --stop 1MEG --points 200 ^
  --output tools\cctl_studio\mna_solver\tb\example6_frequency.csv
```

## Python API

```python
from mna_solver import (
    assemble_mna,
    assemble_outputs,
    assemble_symbolic_mna,
    discretize,
    frequency_response,
    parse_netlist,
    reduce_to_state_space,
    simulate,
)

circuit = parse_netlist("circuit.cir")
symbolic = assemble_symbolic_mna(circuit)
descriptor = assemble_mna(circuit, parameters={"R1": 1000.0})
outputs = assemble_outputs(circuit, descriptor, parameters={"R1": 1000.0})
state = reduce_to_state_space(descriptor, outputs)
discrete = discretize(state, dt=1e-6, method="forward_euler")
result = simulate(discrete, duration=1e-3, inputs={"Vin": 1.0})
response = frequency_response(state, [10.0, 100.0, 1000.0])
```

The discretization entry point deliberately takes a method name. Only
`forward_euler` and the stiff-circuit-friendly `backward_euler` are implemented;
RK methods can be added behind the same interface without changing parser or
state-space clients.

## Diode/MOSFET piecewise-linear simulation

`switched_solver.py` reads TINA continuation lines and `.MODEL` parameters. For
the supplied Buck and Boost cases, D1 uses `Vf=VJ`, `Ron=RS`, and a fixed junction
capacitance linearized from the SPICE `CJO/VJ/M/FC` depletion formula. T1 uses
`Ron=RD+RS+1/(KP*(W/L)*(Vdrive-VTO))`, `Roff=RDS`, `Coss=CBD+CGDO`, and a body
diode with `Vf=PB`, `Ron=RD+RS`. The MOS gate voltage source is removed from the
electrical MNA model and becomes an external `uint32_t PWM` command. Gate-source
discovery requires only the voltage source's positive terminal to be connected
to the MOS gate; its negative terminal may be ground or the MOS source node.

The four primary equations are:

```text
D1 off + MOS channel off     D1 off + MOS channel on
D1 on  + MOS channel off     D1 on  + MOS channel on
```

A physically complete MOS model needs a third path while the gate is off:
source-to-drain body-diode conduction. The solver therefore caches two extra
body-diode equations, one for each D1 state. At every sample it uses the previous
`V(D1.A)-V(D1.K)` and `V(T1.S)-V(T1.D)` to select the next mode. Default outputs
include `V(T1.D)`, `V(T1.S)`, `V(D1.A)`, and `V(D1.K)`.

List all topology matrices and scalar equations:

```bat
python tools\cctl_studio\mna_solver\switched_solver.py analyze ^
  tools\cctl_studio\mna_solver\tb_buck\buck.CIR --dt 1N
```

Run the supplied circuit with an external 10 kHz, 50% PWM command:

```bat
python tools\cctl_studio\mna_solver\switched_solver.py simulate ^
  tools\cctl_studio\mna_solver\tb_buck\buck.CIR ^
  --dt 50N --transition-substep 500P --duration 50M ^
  --pwm-frequency 10K --duty 0.5 --output-stride 100 ^
  --output buck_10khz.csv
```

Device approximations can be overridden with repeated options such as
`--device-param D1.Vf=0.7`, `--device-param T1.Ron=50M`, and
`--device-param T1.Coss=1N`. Backward Euler is the switching default because the
milliohm conduction paths and picofarad junction capacitances are stiff.
The transition substep is used only while both possible diode paths are open;
it prevents a macro-step voltage overshoot while keeping a 50 ms run practical.

At 10 kHz and 50% duty, the 50 ms generated-C++ references produce:

- Buck: `V(VF1)` tail mean about 2.178 V, with a 2.087--2.269 V range. The
  ground-referenced D1 conducts during the MOS-off interval.
- Boost with the supplied 100 uF output capacitor: `V(VF1)` tail mean about
  8.526 V, with an 8.295--8.769 V range. The output diode conducts during the
  MOS-off interval.
- FSBB with complementary `PWM1/PWM2` and `PWM3/PWM4`, and 1 us deadtime:
  `V(VF1)` tail mean about 3.838 V, with a 3.735--3.938 V range.

## Circuit data and Eigen C++ generation

`circuit_data.py` exports a versioned JSON boundary containing external ports,
probes, raw model values and extraction provenance, state/signal names, all
continuous topologies, normal/short-step discrete matrices, affine terms, and
the terminal indices used for previous-sample mode selection. The JSON can be
simulated without rebuilding MNA and is the only input to `cpp_codegen.py`.
One diode plus one MOS produces six topologies. A MOS-only switching circuit
uses three paths per device (`OFF`, channel, body diode), so the four-switch
FSBB exports `3^4 = 81` topologies.

Each case keeps its portable JSON beside the CIR file and writes C++ artifacts
under `generated\cpp`:

```text
tb_buck\buck.CIR
tb_buck\buck.json
tb_buck\generated\cpp\...
```

Generate a case without compiling it:

```bat
tools\cctl_studio\mna_solver\tb_buck\generate_code.bat buck.CIR
```

Only the Eigen calculation header is generated. Testbench source, CMake files,
waveform scheduling, CSV policy, and acceptance limits are deliberately
handwritten per circuit because those are application- and topology-specific.
Regenerating a calculation class therefore never overwrites its testbench.

The generated `BuckCircuit` exposes `step_short(PWM, VS1)`,
`step_normal(PWM, VS1)`, `run`, and `operator()`. Probe results are available as
`circuit.output.VF1` or `circuit["V(VF1)"]`. `FsbbCircuit` similarly exposes
`PWM1`, `PWM2`, `PWM3`, `PWM4`, and `VS1`. Build and run the handwritten
10 kHz, 50 ms testbenches with:

```bat
repair_gmp_vcpkg.bat
tools\cctl_studio\mna_solver\tb_buck\build_testbench.bat
tools\cctl_studio\mna_solver\tb_boost\build_testbench.bat
tools\cctl_studio\mna_solver\tb_fsbb\build_testbench.bat
```

`build_testbench.bat` locates the MNA tools through `GMP_PRO_LOCATION`; when the
BAT is copied beside another circuit, only its `NETLIST_FILE` variable needs to
change. The wrapper uses GMP's installed Eigen/vcpkg tree and disables manifest
restoration in private-environment mode. The generated C++ directory retains a
standalone handwritten `CMakeLists.txt` and `vcpkg.json` for copied projects.

## `1_OPAMP.CIR` validation

The corrected `XIOP1 4 1 VF1 IdOpamp` reduces to a first-order inverting low
pass. Its DC gain is -1 and its pole is 1000 rad/s (about 159.155 Hz). Tests
verify both the DC transfer and `A=-1000` state matrix.

## Current boundaries

- The circuit must be linear and time-invariant after any external
  linearization.
- Descriptor reduction supports regular index-1 MNA systems. Floating nodes,
  conflicting ideal sources, and higher-index ideal topologies are diagnosed as
  singular.
- Initial conditions are state-coordinate values. Mapping physical capacitor
  voltages/inductor currents to initial state coordinates is future work.
- MOS-only multi-switch circuits are supported and grow as `3^N` topologies.
  Mixed circuits with multiple independent diodes and multiple MOSFETs still
  need the general combinational device expansion. A Verilog/fixed-point
  backend over the same data file remains future work.

Run validation with:

```bat
tools\cctl_studio\mna_solver\run_tests.bat
```

The BAT file uses only `cmd.exe` syntax, pauses on success and failure, and runs
all unit plus CLI smoke tests. Automation can use `run_tests.bat --no-pause`.
