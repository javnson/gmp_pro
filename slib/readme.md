# GMP MATLAB/Simulink Library (`slib`)

**English** | [简体中文](readme_cn.md)

`slib` provides GMP Simulink libraries, standard models, SDPE initialization,
and the unified TCP/UDP software-in-the-loop (SIL) bridge. The only maintained
C++ source tree is `tools/gmp_sil/sil_helper`; `slib` publishes MATLAB code,
models, and the `GMP_SIL_Core` MEX built during installation.

## Support boundary

- MATLAB/Simulink R2022b through R2025b are supported.
- Plant models require Simscape Electrical Specialized Power Systems and `powerlib`.
- The installer rejects R2026a and later because that product is no longer available there.
- Build MEX artifacts with the target MATLAB Release and operating system; do not copy them across platforms.
- Unified TCP/UDP SIL is supported on Windows and Linux. Rapid Accelerator validation must use a fixed-step model.

## Source and generated-artifact boundary

| Path | Role | Maintenance rule |
| --- | --- | --- |
| `install_path/R2024b/gmp_sil_core_pack.slx` | Editable/debuggable R2024b SIL library | Complete Mask and block debugging here before export. It is normally a local generated artifact. |
| `simulink_lib_src/gmp_sil_core_pack_src.slx` | Published R2022b-compatible source model | Generate only with `export_gmp_simulink_lib_src`; do not maintain a divergent copy. |
| `simulink_lib_src/src/` | MATLAB helpers and compiled MEX | Maintain MATLAB code here; rebuild MEX from the canonical C++ tree. |
| `simulink_lib_src/tests/` | MATLAB/SIL regression tests | Run after protocol, Mask, or installer changes. |
| `tools/gmp_sil/sil_helper/` | Sole C++ source for protocol, controller API, S-function, and tests | Never duplicate these sources under `slib`. |
| `install_path/<Release>/` | Installed output for one MATLAB Release | Regenerate with the installer, apart from the explicit R2024b editing workflow above. |

Other `_src.slx` files remain the cross-version sources for their respective
libraries. SIL Core deliberately uses an R2024b-edit/R2022b-publish workflow so
the current model can be debugged before exporting a compatible baseline.

## Export the R2022b source

After validating the editable library with MATLAB R2024b or later, run:

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), ...
    'slib', 'export_gmp_simulink_lib_src.m'));
```

The exporter copies the R2024b model, upgrades its unified transport Mask, and
uses `Simulink.exportToVersion(...,'R2022B')` before atomically publishing
`simulink_lib_src/gmp_sil_core_pack_src.slx`. Verify the artifact with:

```matlab
info = Simulink.MDLInfo(fullfile(getenv('GMP_PRO_LOCATION'), ...
    'slib', 'simulink_lib_src', 'gmp_sil_core_pack_src.slx'));
assert(strcmp(info.ReleaseName, 'R2022b'));
```

## Install

Restore ASIO and nlohmann-json through the GMP environment installer first,
then run:

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), ...
    'slib', 'install_gmp_simulink_lib.m'));
```

The installer:

1. validates the MATLAB Release, `powerlib`, and pre-restored GMP dependencies;
2. calls `tools/gmp_sil/sil_helper/build_gmp_sil_mex.m` with MATLAB's own `mex` compiler;
3. stages only `GMP_SIL_Core.<mexext>` in `simulink_lib_src/src`;
4. generates the current `install_path/<Release>` libraries from the R2022b source;
5. copies helpers/MEX, registers paths, and refreshes the Library Browser.

MATLAB never invokes vcpkg or downloads packages during installation. Repair
the GMP private environment through `gmp_env.bat` when dependencies are absent.

## SIL Core Mask

The network portion of the SIL Core Mask contains only:

- `Transport`: TCP or UDP;
- `Target address`: `localhost`, `127.0.0.1`, or a numeric IPv4 address;
- `Target receive port`;
- `Local receive port (UDP)`, shown only for UDP and hidden for TCP.

Local TCP selection emits a UDP performance recommendation; remote UDP emits a
TCP integrity recommendation. Legacy command ports, manual connection IDs, ABI
byte counts, and protocol state are not network Mask inputs. Packing/unpacking
types, sizes, and alignment remain because they define the Simulink signal ABI,
not the network connection.

Initialization derives a unique connection ID from the model and block and
writes the ordinary single-controller contract to `network.json`, including
both ports, payload lengths, and protocol state. Concurrent controllers or
models must each use their corresponding JSON, distinct port pairs, and matching
connection IDs; the IDs also reject crossed sessions.

The normal startup order is controller first. The controller waits indefinitely
for Simulink and prints its listening transport/port. The Simulink MEX client
uses a fixed five-second connect/handshake timeout; a missing controller, bad
address, or rejected handshake releases the network resources and stops model
initialization. Completion still uses an explicit simulation-state frame. The
first ten complete valid responses must pass before Simulink switches to the
debugger-friendly established timeout.

## Development and validation

```matlab
addpath(fullfile(getenv('GMP_PRO_LOCATION'), 'slib', ...
    'simulink_lib_src', 'src'));
results = runtests(fullfile(getenv('GMP_PRO_LOCATION'), 'slib', ...
    'simulink_lib_src', 'tests'));
assert(all([results.Passed]));
```

Recommended SIL library workflow:

1. edit/debug `install_path/R2024b/gmp_sil_core_pack.slx` in R2024b;
2. export and verify the published model reports R2022b;
3. run `install_gmp_simulink_lib` and verify the current-Release library/MEX;
4. run MATLAB regression tests, fixed-step SIL/Rapid Accelerator, and one affected suite model.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `powerlib` is unavailable | Use R2025b or earlier and install/license Specialized Power Systems. |
| ASIO/JSON is absent | Repair the private environment from `gmp_env.bat`; do not invoke vcpkg from MATLAB. |
| `GMP_SIL_Core` is missing | Re-run the installer and check the platform MEX in the installed `src`. |
| TCP/UDP cannot connect | Check address, both ports, startup order, firewall, and JSON ABI lengths. |
| Concurrent SIL sessions cross | Assign distinct port pairs and verify connection IDs. |
| Rapid Accelerator does not communicate | Use fixed-step execution, a current-Release MEX, and inspect the Rapid target build log. |
| Published source has the wrong Release | Re-export from R2024b and verify it with `Simulink.MDLInfo`. |

Run `slib/uninstall_gmp_simulink_lib.m` to remove the current Release installation.
