# GMP Data Link Python API

**English** | [简体中文](README_CN.md)

This package is the supported headless interface to the GMP Data Link Tunable,
Memory Perspective, Scope, and Processor-in-the-Loop services. It is intended for test scripts,
automated validation, laboratory tooling, and AI-assisted hardware debugging.
It uses the same wire protocol and descriptor codecs as the graphical debugger,
so a target does not need separate firmware for GUI and API clients.

The public entry point is `GmpDatalinkClient`. A client owns one serialized
request/response transport and exposes three service objects:

| Service | Attribute | Main operations |
| --- | --- | --- |
| Tunable | `client.tunables` | Discover, read, and write registered parameters |
| Memory Perspective | `client.memory` | Discover, read, and write whitelisted byte ranges |
| Data Link Scope | `client.scope` | Discover, configure, arm, wait, and download waveforms |
| Processor-in-the-Loop | `PilApi` / `PilBridge` | Execute exact controller steps and bridge standard Simulink UDP vectors |

## Installation and import

Run scripts from the debugger directory so that the `apis` package is on the
Python import path:

```powershell
cd tools/gmp_pil_server/gmp_debugger
python -m pip install pyserial
python -m apis.examples.ai_debug_session --port COM5 --baudrate 256000
```

Alternatively, add `tools/gmp_pil_server/gmp_debugger` to `PYTHONPATH`. The
headless API requires Python 3.10 or newer and `pyserial`; it does not require
PyQt or NumPy.

## Quick start

```python
from apis import GmpDatalinkClient

with GmpDatalinkClient("COM5", 256000) as dl:
    parameters = dl.tunables.discover()
    regions = dl.memory.discover()
    scopes = dl.scope.discover()

    print(dl.tunables.read_all())
    if regions:
        print(dl.memory.read_region(regions[0], byte_length=16).hex(" "))
```

The context manager opens and closes the serial port. `timeout` is the timeout
of one request attempt, and `retries` is the number of retries after the first
attempt:

```python
dl = GmpDatalinkClient(
    "COM5",
    921600,
    timeout=0.8,
    retries=3,
)
```

Do not open the same serial port in the GUI and the headless API at the same
time. The serial transport is thread-safe and serializes transactions, but a
single port still has only one request/response owner.

## Tunable parameters

Always discover the target table before modifying a controller. Each
`TunableParameter` contains `item_id`, `name`, `data_type`, and `permission`.
Target names may be empty; the host supplies `Parameter N` as a display name,
and an ID or the descriptor object remains a stable selector.

```python
from apis import AccessPermission

with GmpDatalinkClient("COM5", 256000) as dl:
    table = dl.tunables.discover()
    for item in table:
        print(item.item_id, item.name, item.data_type.name, item.permission.name)

    frequency = dl.tunables.read("sine_frequency_hz")
    dl.tunables.write("sine_frequency_hz", 50.0)
    assert dl.tunables.read("sine_frequency_hz") == 50.0

    writable = [item for item in table if item.permission == AccessPermission.READ_WRITE]
    values_by_id = dl.tunables.read_many(writable)
```

Selectors accepted by Tunable methods are:

- exact display name, such as `"sine_frequency_hz"`;
- integer target ID, such as `0`;
- a `TunableParameter` returned by `discover()`.

`read_all()` returns a name-to-value dictionary for convenient reporting.
When a target intentionally registers duplicate names, use IDs or descriptor
objects because a name selector must identify exactly one entry.
`write_many()` performs bounded batches and rejects read-only descriptors
before sending the request:

```python
dl.tunables.write_many({
    "sine_frequency_hz": 50.0,
    "sine_gain": 1.25,
    "sine_offset": 0.0,
})
```

The API encodes the target-reported type (`U16`, `I16`, `U32`, `I32`, or
`F32`). Python values must fit that wire type.

## Memory Perspective

`discover()` obtains the target-owned whitelist. A `MemoryRegion` contains only
the information required for memory access: `region_id`, byte `address`,
`byte_length`, `permission`, and optional `name`.

```python
import struct

with GmpDatalinkClient("COM5", 256000) as dl:
    regions = dl.memory.discover()
    state = dl.memory.resolve("controller_state")

    raw = dl.memory.read_region(state)
    first_value = struct.unpack_from("<f", raw, 0)[0]

    if state.permission.name == "READ_WRITE":
        dl.memory.write_region(state, struct.pack("<f", 1.0), offset=0)
```

The convenience methods are:

- `read_region(selector, offset=0, byte_length=None)`;
- `write_region(selector, data, offset=0)`;
- `read(address, byte_length, chunk_size=240)`;
- `write(address, data, chunk_size=240)`.

All wire addresses and lengths are in 8-bit bytes, including on C28x. Multiply
a native C28x linker word address by two only when entering an address manually;
target-discovered addresses are already normalized. Large transfers are split
into protocol-sized chunks. A multi-chunk write is not atomic, so do not use it
to update live state that requires an indivisible change.

After discovery, the client validates every requested range and permission
before sending it. The target whitelist remains authoritative and independently
rejects invalid access. This prevents arbitrary host reads or writes from
escaping the explicitly registered memory regions.

## Data Link Scope

The Scope service owns waveform acquisition independently of Memory
Perspective. Discovery returns `ScopeResource` descriptors containing channel
count, depth, sample format, layout, nominal sample rate, and name.

```python
from apis import GmpDatalinkClient, ScopeConfiguration, ScopeTriggerMode

configuration = ScopeConfiguration(
    mode=ScopeTriggerMode.RISING_EDGE,
    trigger_channel=0,
    trigger_level=0.0,
    trigger_position_percent=25.0,
    auto_timeout_ms=1000,
    sample_divider=9,
)

with GmpDatalinkClient("COM5", 256000) as dl:
    scope = dl.scope.discover()[0]
    frame = dl.scope.capture(scope, configuration, timeout=5.0)
    print(frame.generation, frame.sample_rate_hz)
    print(frame.time_seconds[:5])
    print(frame.channels[0][:5])
    frame.save_csv("capture.csv")
```

Trigger modes are `IMMEDIATE`, `RISING_EDGE`, `FALLING_EDGE`,
`RISING_EDGE_AUTO`, and `FALLING_EDGE_AUTO`. Auto modes complete after
`auto_timeout_ms` even when no qualifying edge occurs.

`trigger_position_percent` controls how much of the frame precedes the trigger.
For example, 25% places the trigger near one quarter of the horizontal axis:
approximately 25% of samples describe history before the edge and 75% describe
activity after it. The target continuously maintains the required pre-trigger
history before completing the post-trigger samples.

`sample_divider` is a non-negative integer. Zero means no division. The decoded
effective rate is:

```text
effective_sample_rate = registered_sample_rate / (sample_divider + 1)
```

Protocol-version-1 Scope resources do not support a nonzero divider; the API
raises `ValueError` instead of silently ignoring it.

For continuous acquisition, use the generator and always give unattended tests
a finite `count`:

```python
with GmpDatalinkClient("COM5", 256000) as dl:
    for index, frame in enumerate(
        dl.scope.iter_captures("control_scope", configuration, count=20)
    ):
        frame.save_csv(f"capture_{index:03d}.csv")
```

Each iteration arms only after the previous ready frame has been completely
downloaded. This handshake prevents the target from overwriting a snapshot
during upload. A capture timeout raises `GmpDlTimeout` and does not leave the
host transport busy; the caller may immediately send a new configuration or
use an immediate/auto trigger.

Advanced code may control the sequence explicitly with `configure()`, `arm()`,
`status()`, and `read_snapshot()`.

## Processor-in-the-Loop

`PilConfiguration` loads the target-local SDPE requirement and validates the
PIL enable switch, command allocation, channel masks, channel indices, UART
rate, and UDP endpoints. This makes SDPE the connection source of truth for
both firmware and host tooling.

From `tools/gmp_pil_server/gmp_debugger`, start the standard bridge with:

```powershell
python -m apis.examples.pil_bridge `
  --sdpe ../../../ctl/suite/mcs_pmsm_nt/project/f280049c/sdpe_mgr/sdpe_requirement.json `
  --port COM5 `
  --trace ../../../ctl/suite/mcs_pmsm_nt/project/f280049c/pil/results/manual/bridge_trace.csv
```

The Simulink side uses the standard 264-byte input vector and 200-byte output
vector. Each accepted UDP input causes exactly one Data Link STEP transaction
and one controller execution. A STEP transaction is never automatically
retried: if a response is lost after the target executes, retrying the same
request would advance the controller twice. The CSV trace records simulation
time, selected encoder input, all ADC/PWM/monitor channels, output-enable state,
and target round-trip time.

`ENABLE_GMP_DL_PIL_SIM` must be an independent target-local SDPE feature. When
it is disabled, the normal ADC ISR and physical PWM path compile and operate as
before. A target that enables it must isolate physical outputs and allow only a
validated PIL STEP request to dispatch the controller. Never use a PIL firmware
image to drive connected power hardware.

## Errors and low-level extension

The public exception hierarchy is:

- `GmpDlError`: base API/transport error;
- `GmpDlTimeout`: no valid response or no completed Scope frame in time;
- `GmpDlProtocolError`: malformed or inconsistent target response;
- `GmpDlTargetError`: explicit target rejection;
- standard `ValueError`, `KeyError`, `PermissionError`, and `struct.error` for
  invalid caller input or incompatible values.

New Data Link submodules can reuse the transport without duplicating serial
framing:

```python
response = dl.transact(command=0x70, payload=b"\x01\x02")
```

Alternative transports used by simulators and tests only need a synchronous
`transact(command: int, payload: bytes) -> bytes` method. Custom service command
bases can be supplied through `tunable_command`, `memory_command`, and
`scope_command` when constructing `GmpDatalinkClient`.

## Recommended automation and AI workflow

1. Discover all resources and record their descriptors.
2. Read and preserve the initial Tunable values before changing anything.
3. Confirm target state and permissions; apply only bounded, meaningful changes.
4. Read values back after every write.
5. Acquire Scope evidence with a finite timeout and save it to CSV.
6. Restore temporary parameter changes in a `finally` block.
7. Stop immediately on protection events, repeated timeouts, or inconsistent
   descriptors; never compensate by issuing arbitrary memory writes.

Writes act on a live embedded system. The API provides protocol and whitelist
safety, but it cannot determine whether a requested control value is physically
safe for connected power hardware.

## Verification

From `tools/gmp_pil_server/gmp_debugger` run:

```powershell
python -m unittest discover -s apis/tests -v
python -m py_compile apis/__init__.py apis/client.py apis/protocol.py apis/pil.py apis/examples/pil_bridge.py
```

The complete runnable example is
[`examples/ai_debug_session.py`](examples/ai_debug_session.py). The target-side
wire protocol is documented in
[`core/dev/datalink/readme_dl_protocol.md`](../../../../core/dev/datalink/readme_dl_protocol.md).
