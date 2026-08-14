# CTL Unit-Test Workspace

Open `ctl_unit_test.sln` in Visual Studio and use **Test > Test Explorer** to
run or debug the tests. The solution contains three independent native C++
unit-test projects:

| Project | Responsibility |
| --- | --- |
| `CTL Math Unit Tests` | Float, double, emulated TI IQ24, explicit conversions, constants, angle domains, nonlinear functions, and lite vector/matrix algebra |
| `CTL Intrinsic Unit Tests` | Basic blocks, continuous PI/PID controllers, and discrete filters |
| `CTL Interface Unit Tests` | Gain/bias models, ADC/DAC conversion, PWM channels, and SPWM modulation |

Each project has `NativeUnitTestProject`, `IsTestProject`, `TestProject`, and
`TestContainer` metadata. Microsoft native C++ tests are packaged as DLL test
containers by design; the project type and Test Explorer metadata make them
unit-test projects, and Test Explorer is the supported run/debug entry point.

All tested GMP implementation files are referenced from their authoritative
locations through paths relative to `ctl/unit_test/projects`. Tests do not keep
private copies of CTL source files. The projects use `GMP_CTL_PORTABLE`, so they
do not require a CSP, peripheral service, or the full GMP runtime.

## Visual Studio validation

Build `ctl_unit_test.sln` with the `Debug|x64` configuration. Test Explorer
discovers 27 tests across the three test containers. The math project compiles
the emulated IQmath implementation from `third_party/iqmath/src` with
`GLOBAL_IQ=24`. The imported compatibility sources use a project-local
third-party warning boundary; GMP test and production sources remain at
`/W4 /WX`.

## Command-line and Linux validation

Run `install_gmp_virtual_env.bat` after cloning or changing `vcpkg.json`. The
installer restores Google Test into the repository-private vcpkg tree. Run all
Windows-hosted CMake/CTest contracts with:

```bat
ctl\unit_test\run_tests.bat
```

On Linux, source `bin/linux/activate_gmp.sh` and run:

```bash
bash ctl/unit_test/run_tests.sh
```

The CMake entry point exposes separate math, intrinsic, and interface targets.
It also includes the canonical contracts from `ctl/math_block/tests`, including
IQ24 with float and double parameter domains, architecture compilation, lite
algebra, and the SInv QR-controller contract.

## Coverage status

| Domain | Current coverage | Status |
| --- | --- | --- |
| Math: float | Conversions, constants, arithmetic, nonlinear functions, radian/per-unit angles, vector/matrix lite | Active |
| Math: double | Precision-preserving conversions, constants, nonlinear functions, radian/per-unit angles, vector/matrix lite | Active |
| Math: IQ24 | Quantization, conversions, multiply/divide, square root, trigonometry, `atan2`, exp/log, and power | Active |
| Intrinsic: basic | Saturation, slope limiter, and hysteresis state | Active |
| Intrinsic: continuous | PI/PID coefficients, step behavior, output and integrator limits | Active |
| Intrinsic: discrete | Low-pass behavior and stationary/reverse ramp-generator frequencies | Active |
| Interface models | Sensor gain and bias equations | Active |
| Interface channels | ADC and DAC scaling, PWM saturation/inversion, and SPWM zero-vector mapping | Active |
| Intrinsic advanced/protection | Per-component initialization, step response, limits, and reset behavior | Planned |
| Motor control | Transforms, observers, current/mechanical loops, and motor modulators | Planned |
| Digital power | PLL, resonant controllers, protection, and power stages | Planned |
| Framework and DSA | State transitions, scheduling, triggers, and trace behavior | Planned |

Every CTL bug fix should retain a regression test. Tests must use public CTL
APIs and tolerances appropriate to the numeric backend. Code that supports
fixed-point execution must receive an IQmath test. Keep all test code and
comments in English ASCII.
