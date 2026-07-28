# Inverter component host simulation

This focused host test validates:

- four-leg 3D-SVPWM duty bounds and alpha-beta-zero reconstruction;
- stand-alone LC capacitor-voltage regulation, including switchable coupling feed-forward;
- voltage-loop output saturation and back-calculation recovery after reference reversal;
- QPR zero-sequence current rejection and control-boundary tuner deployment.

Configure and run from a Visual Studio developer shell:

```powershell
cmake -S . -B build -G Ninja -DGMP_PRO_LOCATION="$env:GMP_PRO_LOCATION"
cmake --build build
.\build\gmp_inv_host_sim.exe
.\build\gmp_inv_header_cpp_smoke.exe
```

The test uses the real GMP headers and controller source files. It is a
principle/host simulation, not a hardware validation.
