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

Both packages are already part of GMP's Python environment and are pinned in
`tools/gmp_installer/requirements-gmp.txt`. The solver adds no separate runtime
stack.

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
the supplied `2_buck.CIR`, it derives `D1` series resistance and junction
capacitance from `RS/CJO`, and approximates T1 with `Ron/Roff/Coss`. The MOS gate
voltage source is removed from the electrical MNA model and becomes a Boolean
external PWM command.

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
  tools\cctl_studio\mna_solver\tb\2_buck.CIR --dt 1N
```

Run the supplied circuit with an external 10 kHz, 50% PWM command:

```bat
python tools\cctl_studio\mna_solver\switched_solver.py simulate ^
  tools\cctl_studio\mna_solver\tb\2_buck.CIR ^
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

The 50 ms reference run covers five `L/(R1+R2)` time constants and 500 PWM
periods. Its final `V(VF1)` is about 2.25 V (tail mean about 2.25 V), close to
the ideal 2.5 V half-supply value; the remaining difference is the explicit
body-diode forward drop and finite path resistances.

The supplied MOS is connected with `D=4` and `S=3`, where node 3 is the +5 V
supply. Consequently its intrinsic S-to-D body diode catches the switch node
when the gate turns off; D1 remains off in the reference run. This orientation
should be checked if ground-diode Buck freewheeling was intended.

## `1_OPAMP.CIR` diagnostic

The parser recognizes `XIOP1 3 1 VF1 IdOpamp` as a TINA three-pin ideal op amp
with conventional `(+,-,output)` order. The resulting descriptor pencil is
singular for every frequency: node 3 is pulled to ground through R2, the ideal
op-amp constraint equates nodes 3 and 1, and VG1 independently fixes node 1.
Therefore this particular exported netlist does not currently describe a
solvable low-pass circuit. The solver reports the singular topology instead of
inventing a pin order or silently regularizing it. The schematic/netlist
connection must be corrected before a meaningful transfer function exists.

## Current boundaries

- The circuit must be linear and time-invariant after any external
  linearization.
- Descriptor reduction supports regular index-1 MNA systems. Floating nodes,
  conflicting ideal sources, and higher-index ideal topologies are diagnosed as
  singular.
- Initial conditions are state-coordinate values. Mapping physical capacitor
  voltages/inductor currents to initial state coordinates is future work.
- C/C++ iterative-expression and Verilog generation are the next layer; they are
  not emitted by this initial solver.

Run validation with:

```bat
tools\cctl_studio\mna_solver\run_tests.bat
```

The BAT file uses only `cmd.exe` syntax, pauses on success and failure, and runs
all unit plus CLI smoke tests. Automation can use `run_tests.bat --no-pause`.
