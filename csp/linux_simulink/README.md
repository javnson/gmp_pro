# Linux Simulink CSP

This CSP runs a GMP controller as a native Linux process and connects it to a
MATLAB/Simulink SIL or PIL endpoint through the unified framed TCP/UDP helper.
It is intended for hosted targets, build servers, and remote simulation nodes;
it does not modify the host operating system or emulate physical peripherals.

The application owns `xplt.config.h` and the concrete
`gmp_pc_simulink_rx_buffer_t` and `gmp_pc_simulink_tx_buffer_t` mappings. The CSP
supplies the runtime loop, one-millisecond system tick mapping, trace writer,
hosted peripheral stubs, and optional CAN-hook dispatch. `network.json` remains
the authoritative transport and buffer-ABI contract.

Builds require C++17, pthreads, standalone Asio, and nlohmann-json. The Linux
GMP installer places the required host tools and libraries under `bin/linux`;
source its generated `bin/linux/activate_gmp.sh` before configuring a project.

The CSP compile contract can be checked without a suite application:

```bash
cmake -S csp/linux_simulink/tests -B csp/linux_simulink/tests/out -G Ninja
cmake --build csp/linux_simulink/tests/out
```

Do not select this module together with another CSP. A source-manager project
selects it as `csp|linux_simulink`; its dependency closure supplies the GMP
runtime and standard platform layers.
