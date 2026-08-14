# Inverter Component Host Simulation

This CMake project exercises the reusable grid-forming inverter control path on
a desktop compiler. It builds against the repository headers directly and uses
the local `config` directory for the minimal GMP type/configuration contract.

Configure, build, and test with:

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The test is a host-level numeric and integration check. It does not constitute
physical hardware verification.
