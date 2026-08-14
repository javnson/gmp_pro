# CTL Unit-Test Workspace

Open `ctl_unit_test.sln` with Visual Studio 2022 and select **Test > Test
Explorer**. The solution currently builds the same contract suite for both
`ctrl_gt=float` and `ctrl_gt=double`; each DLL uses the Visual Studio native C++
unit-test framework, so Test Explorer can discover, run, debug, and group every
test without an additional adapter. The tests use `GMP_CTL_PORTABLE`, so they
validate CTL without bringing in a CSP, peripheral services, or the GMP runtime.

Run `install_gmp_virtual_env.bat` after cloning or after changing `vcpkg.json`.
The GMP installer restores Google Test into the repository-private shared vcpkg
tree. A command-line regression run is available through:

```bat
ctl\unit_test\run_tests.bat
```

The command-line and Linux CMake target uses Google Test to execute the same
contracts outside Visual Studio. It also includes the canonical tests from
`ctl/math_block/tests`, including IQ24 with float and double parameter domains,
the architecture compile contract, and the SInv QR-controller contract. It is
usable on Linux after sourcing
`bin/linux/activate_gmp.sh`:

```bash
bash ctl/unit_test/run_tests.sh
```

## Coverage status

| Domain | Current coverage | Status |
| --- | --- | --- |
| Numeric contracts | `real2param`, `real2ctrl`, `param2ctrl`, `ctrl2param`; float/double type selection | Active |
| Constants and angles | Control/parameter constants, epsilon, radian and per-unit sine/cosine contracts | Active |
| C++ lite algebra | Generic vector arithmetic, dot product, matrix-vector and matrix-matrix multiplication | Active |
| Intrinsic continuous | PID initialization, cached coefficients, output limiting, integrator limiting | Partial |
| Fixed-point IQmath | IQ24 conversions and nonlinear math with float/double parameter domains | Active |
| Intrinsic discrete/advanced/protection | Per-component initialization, step response, limits, reset behavior | Planned |
| Motor control | Transforms, observers, current/mechanical loops, modulators | Planned |
| Digital power | PLL, resonant controllers, modulators, protection, power stages | Planned |
| Framework and DSA | State transitions, scheduling, triggers, trace behavior | Planned |

Every new CTL bug fix should add a failing test before the fix and retain it as
a regression case. Tests must exercise at least the supported float and double
domains; code that supports fixed-point must also receive an IQmath test. Keep
all test code and comments in English, use the public CTL API, and compare
results with tolerances appropriate to the selected numeric domain.
