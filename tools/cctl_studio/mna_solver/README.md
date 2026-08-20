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
- Eigen3: default fixed-size matrix backend for generated C++ classes.
- CCTL `fixed_vector`/`fixed_matrix`: dependency-free inline-storage backend.

Python packages are pinned in `tools/gmp_installer/requirements-gmp.txt`.
Eigen3 is declared by this directory's `vcpkg.json` and restored into GMP's
shared vcpkg tree by the installer. The deprecated `third_party/eigen` path is
not used. Code generated with `--backend fixed` does not include or link Eigen.

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
  tools/cctl_studio/mna_solver/tb/basic/0_divider.CIR
```

For symbolic passive or controlled-source values, repeat `--param`:

```bat
python tools\cctl_studio\mna_solver\mna_solver.py analyze ^
  tools\cctl_studio\mna_solver\tb\basic\example5.cir ^
  --param R1=1K --param R2=10K
```

List the forward-Euler matrices and scalar `x[k+1]`/`y[k]` expressions:

```bat
python tools\cctl_studio\mna_solver\mna_solver.py discretize ^
  tools\cctl_studio\mna_solver\tb\basic\example6.cir --dt 1N
```

Forward-Euler simulation writes a CSV file. Independent source values in the
netlist are the defaults; `--input` overrides them and is required for a source
whose value is symbolic.

```bat
python tools\cctl_studio\mna_solver\mna_solver.py simulate ^
  tools\cctl_studio\mna_solver\tb\basic\example6.cir ^
  --dt 1N --duration 10U --input Vin=1 ^
  --output tools\cctl_studio\mna_solver\tb\basic\example6_result.csv
```

Generate magnitude, dB magnitude, and phase for every input/output pair:

```bat
python tools\cctl_studio\mna_solver\mna_solver.py frequency ^
  tools\cctl_studio\mna_solver\tb\basic\example6.cir ^
  --start 10 --stop 1MEG --points 200 ^
  --output tools\cctl_studio\mna_solver\tb\basic\example6_frequency.csv
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
  tools\cctl_studio\mna_solver\tb\buck\buck.CIR --dt 1N
```

Run the supplied circuit with an external 10 kHz, 50% PWM command:

```bat
python tools\cctl_studio\mna_solver\switched_solver.py simulate ^
  tools\cctl_studio\mna_solver\tb\buck\buck.CIR ^
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
- Single-phase inverter with a 30 V DC bus, 10 kHz centre-aligned bipolar SPWM,
  a 50 Hz reference at modulation index 0.8, and 1 us deadtime: the differential
  load voltage `V(2,1)` has a 22.031 V fundamental peak and 15.610 V RMS. Its
  50 Hz current fundamental peak is 2.203 A.
- Controlled-precharge bridge rectifier with a 32 Vrms, 50 Hz source: before
  bypassing the 10 ohm charging resistor, `V(VF1)` averages about 14.490 V;
  closing `SW2` at 60 ms raises the settled full-wave average to about 31.088 V
  with a 44.133 V peak.
- Three-phase inverter with a 5 V DC bus, 10 kHz centre-aligned SPWM, 50 Hz
  three-phase references at modulation index 0.8, and 1 us deadtime: the three
  phase-voltage fundamental peaks are 1.8566--1.8571 V and the current
  fundamental peaks are 0.18568--0.18573 A. The voltage phasor pairwise cosines
  are within 0.00025 of -0.5, confirming 120-degree separation.
- NPC(I) Buck with two 30 V sources, a 10 kHz carrier, and 1 us deadtime: the
  0.2 pu reference switches only between N/O and settles to about 11.050 V;
  the 0.8 pu reference switches only between O/P and settles to about 45.453 V.
  These loaded open-loop values include the extracted semiconductor, 50 mohm
  source-series, and deadtime losses, so they are below the ideal 12/48 V.

## PMSM current-source coupling

A complete, common-neutral current-source triplet named `IPMSMx_A`,
`IPMSMx_B`, and `IPMSMx_C` is recognized as one PMSM interface. The numeric
value carried by a TINA source, such as `10M`, is only a netlist placeholder:
its generated input default is forced to zero. The parser adds
terminal-to-neutral voltage outputs `VPMSMx_A/B/C`; portable JSON annotates the
three inputs with `pmsm_phase_current`, the three outputs with
`pmsm_phase_voltage`, and records their phase, terminal, and neutral mapping.
An incomplete triplet remains ordinary current sources, while a complete
triplet with different neutral nodes is rejected.

`cctl/circuit_model/pmsm_cs.hpp` supplies the motor side. Its initialization
function validates SI parameters, clears the state, and precomputes inverse
inductances/inertia, resistance and flux ratios, torque coefficients, and RK4
step factors. Runtime calls accept phase-to-neutral voltages and return
three-wire phase currents, d/q quantities, torque, rotor angle, mechanical
speed, and electrical frequency. Positive phase current flows from the
inverter terminal into the motor neutral, matching the SPICE current-source
direction. The model includes d/q cross coupling, permanent-magnet back EMF,
saliency torque, inertia, viscous friction, and external load torque.

The handwritten `tb\pmsm` testbench instantiates the generated 729-topology
inverter and motor separately, then explicitly connects currents and voltages.
It uses the `SM060R20B30MNAD` preset (converting g*cm^2 inertia and
micro-N*m*s/rad friction to SI), a 48 V bus, 10 kHz SPWM, 1 us deadtime, and a
0.5 s V/f ramp to 20 Hz. The 0.8 s run settles at 20.0001 Hz electrical /
300.002 rpm, with 3.311 A maximum phase current and
`|ia+ib+ic| <= 4.44e-16 A`. A locked-rotor analytical RL check runs first.

The requested 48 V bus intentionally exceeds
`SM060R20B30MNAD_MAX_DC_VOLTAGE` (14.2 V), so this is a numerical integration
test only and is not a hardware-safe operating point.

## Circuit data and fixed-size C++ generation

`circuit_data.py` exports a versioned JSON boundary containing external ports,
probes, raw model values and extraction provenance, state/signal names, all
continuous topologies, normal/short-step discrete matrices, affine terms, and
the terminal indices used for previous-sample mode selection. The JSON can be
simulated without rebuilding MNA and is the only input to `cpp_codegen.py`.
One diode plus one MOS produces six topologies. A MOS-only switching circuit
uses three paths per device (`OFF`, channel, body diode), so the four-switch
FSBB exports `3^4 = 81` topologies and the six-switch three-phase inverter
exports `3^6 = 729`. Independent diodes and TINA `VSWITCH`
devices use two paths each; the rectifier therefore exports `2^(4+1) = 32`
topologies. Each `VSWITCH` control source is removed from MNA and exposed as a
`uint32_t` command port.
Mixed circuits take the Cartesian product of both device families. The NPC Buck
has four three-path MOSFETs and two two-path clamp diodes, so its portable data
contains `3^4 * 2^2 = 324` compatible state/output models. Diode A/K and MOSFET
D/S terminal voltages remain internal signals used for previous-sample mode
selection.

Code generation separates logical topologies from calculation states. Every
logical topology remains addressable, preserving switching selection and
`last_topology_index()`, while numerically equivalent topologies may map to one
calculation state. Calculation states contain only indices into shared static
`StateMatrix`, `InputMatrix`, `StateVector`, `SignalMatrix`,
`SignalInputMatrix`, and `SignalVector` pools, so identical A/B/C/D and affine
terms have one C++ copy.

Equivalence means that every element differs by no more than the absolute
`matrix_tolerance`; relative tolerance is zero. The default is `1e-12`. Square
matrices first enter determinant hash buckets, while non-square objects use
their element sum. Adjacent buckets are included to cover quantization
boundaries, and every candidate still receives a full elementwise check, so a
determinant collision cannot merge unequal states. A complete state merges only
when normal/short A, B, bias and C, D, output bias all match, including internal
D/S and A/K mode-selection signals.

Both `circuit_data.py export` and `cpp_codegen.py` accept
`--matrix-tolerance`; the code-generator option overrides the value stored in
JSON. Their console output reports logical state count, discretization method,
normal/startup step sizes, tolerance, timed progress with ETA, unique and
deduplicated calculation-state counts, and per-pool sharing. Redirected output
uses sparse progress records instead of carriage-return animation. At the
current default tolerance the INV case keeps all 729 complete calculation
states because their internal selection/state matrices differ, but matrix
pooling reduces stored coefficients from 332424 to 187438, a 43.61% reduction.

### Test-case organization

All supplied circuits live below `tb`. Non-switching examples are grouped in
`tb\basic`; run their symbolic, numeric, transient, and known-result checks with:

```bat
tools\cctl_studio\mna_solver\tb\basic\run_tests.bat
```

Validated switching cases use the same layout:

```text
tb\buck\
├── buck.CIR
├── generate_code.bat
├── build_test.bat
├── generated\              generated JSON, calculation headers, and matrix archives
└── test\cpp\               handwritten testbench, CMakeLists and vcpkg.json
```

`boost`, `fsbb`, `sinv`, `rectifier`, `inv`, `buck_npc`, and `pmsm` follow the same
contract.

Every BAT entry point and CMake test project resolves the solver and installer
directly through the `GMP_PRO_LOCATION` environment variable. The CMake files do
not prompt for or accept a cache-variable replacement for the repository path.
The scripts do not depend on the checkout drive or current working directory.
User-facing scripts pause on success and failure, while automation can pass
`--no-pause`.

Each switching case writes its portable JSON and generated C++ calculation class
under `generated`. An Eigen class consists of a small `*.hpp` plus a same-stem
`*.archive`; fixed continues to embed coefficients in one header. JSON, archives,
and local CSV/build/IDE outputs are ignored because they are reproducible and can
be large.

### Generation and build

Generate a case without compiling it:

```bat
tools\cctl_studio\mna_solver\tb\buck\generate_code.bat
```

Each `generate_code.bat` declares `NETLIST_FILE`, `MATRIX_TOLERANCE`, and
`MATRIX_BACKEND` near the top. The backend accepts `eigen` or `fixed`; the
command-line equivalent is:

```bat
python tools\cctl_studio\mna_solver\cpp_codegen.py circuit.json generated ^
  --backend fixed
```

The CLI and every supplied case default to `eigen`. Select `--backend fixed`, or
change a case's `MATRIX_BACKEND`, only when the allocation-free fixed backend is
specifically required. Fixed generation remains supported but is not part of the
default generation or build path.

Only the calculation class artifacts are generated. Testbench source, CMake
files, waveform scheduling, CSV policy, and acceptance limits are deliberately
handwritten per circuit because those are application- and topology-specific.
Regenerating a calculation class therefore never overwrites its testbench.

The default Eigen backend writes the six deduplicated matrix pools, the
topology-to-calculation-state mapping, and nine pool indices per calculation
state into a version-1 binary archive. The format is little-endian `float64` in
Eigen column-major order and records dimensions, state counts, both step sizes,
matrix tolerance, source-netlist SHA-256, payload size, and an FNV-1a 64 checksum.
The generated header retains fixed-size Eigen types, typed ports and outputs,
topology selection, and the archive loader. Construction validates and loads the
pools once; `step` performs no file I/O or archive-related allocation.

The default constructor resolves the generated archive filename from the process
working directory. Supplied CMake tests copy it beside the executable. Other
deployments must do the same or pass an explicit path, for example
`BuckCircuit circuit("data/buckcircuit.archive");`. Missing files and archive
version, dimension, source, or checksum mismatches fail at construction with a
descriptive exception.

The fixed backend uses `cctl::fixed_matrix<T, Rows, Columns>`, implemented as an
inline array of fixed row vectors. It performs no heap allocation and supports
matrix-vector, matrix-matrix, matrix add/subtract, scalar operations, and a
fused `A*x + B*u + bias` transform used by generated steps. Define
`CCTL_FIXED_MATH_USE_AVX` and enable AVX2 (`/arch:AVX2` for MSVC or `-mavx2`
for GCC/Clang) to select 256-bit float/double dot-product and row-accumulation
kernels. Without the macro, the same API uses portable scalar loops. Selection
is at compile time with no runtime CPU dispatch: an AVX2 binary requires a
compatible CPU, and SIMD summation can differ by normal last-bit roundoff.
Generated fixed coefficient pools are C++17 `static constexpr std::array`
objects, so large matrices reside directly in static read-only storage instead
of creating multi-megabyte temporary objects on first access. Large MSVC targets
must raise the `/constexpr:steps` limit accordingly.

Voltage probes whose field starts with `VADC_` remain normal generated outputs
and also receive `role: adc_sample_voltage` plus the suffix as `adc_channel` in
portable JSON. The metadata identifies ADC-ready nodes; reference voltage,
resolution, physical channel routing, and SOC timing remain responsibilities of
the handwritten testbench or target peripheral layer.

The generated `BuckCircuit` exposes `step_short(PWM, VS1)`,
`step_normal(PWM, VS1)`, `run`, and `operator()`. Probe results are available as
`circuit.output.VF1` or `circuit["V(VF1)"]`. `FsbbCircuit` similarly exposes
`PWM1`, `PWM2`, `PWM3`, `PWM4`, and `VS1`. Build and run the handwritten
switching testbenches with:

```bat
repair_gmp_vcpkg.bat
tools\cctl_studio\mna_solver\tb\buck\build_test.bat
tools\cctl_studio\mna_solver\tb\boost\build_test.bat
tools\cctl_studio\mna_solver\tb\fsbb\build_test.bat
tools\cctl_studio\mna_solver\tb\sinv\build_test.bat
tools\cctl_studio\mna_solver\tb\rectifier\build_test.bat
tools\cctl_studio\mna_solver\tb\inv\build_test.bat
tools\cctl_studio\mna_solver\tb\buck_npc\build_test.bat
tools\cctl_studio\mna_solver\tb\pmsm\build_test.bat
```

The SINV testbench drives `PWM1=PWM3` and `PWM2=PWM4` as the two bipolar-SPWM
diagonals. It explicitly checks that neither bridge leg overlaps and that each
carrier period contains the requested 1 us both-off intervals. The netlist
registers `V(2,1)` because TINA's visual voltage meter is stored only in the
binary TSC file and is not emitted into the portable CIR netlist. Parallel DC
link capacitors are stamped additively; regression tests prove that the two
100 uF branches produce exactly the same 81 descriptor matrices as one 200 uF
branch.

The INV testbench drives three complementary bridge legs from sinusoidal
references separated by 120 degrees. It checks gate interlock and 1 us
deadtime, balanced voltage/current fundamentals, phase separation, and
`V(3,1)+V(4,1)+V(5,1)` cancellation. In addition to the three 1 MOhm capacitor
leakage branches R8--R10, the netlist explicitly references load neutral node 1
and capacitor neutral node 2 to ground through 1 MOhm R12 and R11. The all-off
topology's algebraic block is therefore full rank, and all 729 topologies reduce
without an implicit solver-added reference.

The rectifier maps TINA's `VSWGPIO1` control source to the public
`uint32_t SWGPIO1` port. `SWGPIO1=0` retains the 10 ohm charging resistor and
`SWGPIO1=1` closes `SW2` through its reduced 1 mOhm path. The generated model
uses the previous A--K voltage of D1--D4 to select the two conducting bridge
pairs. Its handwritten testbench applies a 32 Vrms, 50 Hz sinusoid and removes
the charging resistor after 60 ms.

The NPC Buck testbench treats `VS1=VS2=30 V`, so 1 pu is the complete 60 V DC
bus. At 0.2 pu it holds PWM3 on and commutates PWM2/PWM4 to select N/O; at 0.8
pu it holds PWM2 on and commutates PWM1/PWM3 to select O/P. Both complementary
pairs include a measured 1 us deadtime. This stiff Coss circuit uses a 1 ns
normal step and 100 ps startup step, and writes separate 0.2/0.8 pu CSV traces.
The explicit 50 mohm `R3` and `R4` in the updated netlist make every ideal-source
cutset reducible without adding hidden solver regularization. Because this
series-fed midpoint is not fixed to ground by an ideal source, both extracted
460 pF clamp-diode junction capacitances remain in the seven-state model.

The four extracted 460 pF diode junction capacitances touch terminals of the
ideal time-varying `VS1`. Such capacitances introduce an input-derivative
(`u_dot`) term and an index-2 descriptor that cannot be represented by the
current ordinary `x_dot=Ax+Bu` boundary. Their extracted values and suppression
reason are retained in JSON, but they are omitted from this case's implemented
state matrices. A future descriptor/input-derivative backend can restore them.

`generate_code.bat` defines `NETLIST_FILE` near its beginning, so direct
invocation uses the case's default CIR; an explicit CIR may also be passed as
the first argument. `build_test.bat` generates the calculation header and archive first,
then configures the handwritten `test\cpp` project. Every default case uses the
Eigen package maintained by the GMP installer/vcpkg environment. An explicitly
selected fixed build can still enable `CCTL_FIXED_MATH_USE_AVX` and the target
compiler's AVX2 option.

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
  Diode/VSWITCH-only networks grow as `2^N`. Mixed circuits containing both
  MOSFETs and multiple independent diodes or VSWITCH devices still need the
  general combinational device expansion. A Verilog/fixed-point backend over
  the same data file remains future work.

Run validation with:

```bat
tools\cctl_studio\mna_solver\tests\run_tests.bat
```

The BAT file uses only `cmd.exe` syntax, pauses on success and failure, and runs
all Basic, unit, CLI, and C++ case tests. Automation can use
`tests\run_tests.bat --no-pause`.
