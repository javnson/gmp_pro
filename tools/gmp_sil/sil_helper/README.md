# GMP SIL Helper

**English** | [简体中文](README_CN.md)

This directory is the single source tree for GMP SIL/PIL communication. It
contains TCP and UDP transports, the framed protocol, the controller-side
`gmp_sil_helper`, the Simulink S-function, the MATLAB MEX builder, and tests.
`slib` receives only the compiled `GMP_SIL_Core` MEX artifact and never a C++
source copy.

## Main files

| File | Purpose |
| --- | --- |
| `gmp_sil_protocol.hpp` | Frame header, state, connection ID, sequence, and ABI validation |
| `asio_tcp_helper.hpp` | Exact framed I/O over a TCP byte stream |
| `framed_udp_helper.hpp` | Symmetric framed I/O over UDP datagrams |
| `gmp_sil_helper.hpp` | Unified controller API and JSON configuration parser |
| `mdl_gmp_sil_core.cpp` | Simulink Level-2 C++ S-function implementation |
| `GMP_SIL_Core.cpp` | Single MATLAB MEX/code-generation compilation entry |
| `build_gmp_sil_mex.m` | Builds the S-function with MATLAB's configured MEX compiler |

The controller normally starts first and blocks for Simulink. Simulink sends a
hello/first frame before causal request-response exchange begins. Completion is
reported by an explicit simulation-state frame rather than by timeout.

The controller/server has no startup timeout by default: it may be launched
first and wait indefinitely for Simulink, with an explicit listening message.
The Simulink MEX client opens the connection at its first real major step and
applies a five-second connect/handshake timeout; failure releases the transport
and stops the simulation. Its first ten
complete valid responses use the short timeout before the session switches to
the debugger-friendly established timeout. Invalid IDs, ABI lengths, sequences,
or partial frames do not advance the count.

Each helper instance owns an isolated connection. Concurrent projects must use
unique connection IDs and port pairs. Prefer UDP for localhost and TCP for
cross-machine or unreliable networks.

## MATLAB MEX

Dependencies are restored by the GMP private environment; MATLAB does not run
vcpkg or download packages.

```matlab
addpath(fullfile(getenv('GMP_PRO_LOCATION'), 'tools', 'gmp_sil', 'sil_helper'));
artifact = build_gmp_sil_mex;
```

For a normal installation run `slib/install_gmp_simulink_lib.m`. It invokes the
same builder in a temporary directory and copies only the MEX artifact into the
source-package and release-specific installation directories.

## C++ tests

On Windows, the single entry point creates `build`, compiles, and runs the
tests through `gmp_env.bat`:

```bat
tools\gmp_sil\sil_helper\build.bat Release
```

Equivalent manual commands are:

```powershell
cmake -S tools/gmp_sil/sil_helper `
      -B tools/gmp_sil/sil_helper/build `
      -DCMAKE_TOOLCHAIN_FILE=bin/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build tools/gmp_sil/sil_helper/build --config Release
ctest --test-dir tools/gmp_sil/sil_helper/build -C Release --output-on-failure
```

The tests cover TCP/UDP exchange, fragmented TCP frames, connection/ABI/sequence
validation, explicit completion, active abort, startup timeout policy, and two
concurrent isolated sessions. See `network.example.json` for the complete JSON
schema.

Rapid Accelerator may probe the S-function with an `mdlStart`/`mdlTerminate`
lifecycle while building its target. `GMP_SIL_Core` only allocates buffers in
`mdlStart`; its first real major-step `mdlOutputs` opens the session and sends
`session_hello`. The build probe therefore cannot consume the controller's
single session or send a false terminal state. Unexpected-frame diagnostics
include frame kind, sequence, flags, and payload length. Use a MEX built by the
matching MATLAB Release on both Windows and Linux.
