# GMP SIL TCP Helper v2

**English** | [简体中文](README_CN.md)

This directory contains an independent, framed TCP transport prototype for
future GMP SIL/PIL use over unreliable or cross-machine networks. It is not
installed into MATLAB, referenced by a Simulink library, or selected by any
Suite project.

## Why TCP needs a GMP frame

TCP preserves byte order and retransmits lost IP packets while a connection is
alive, but it does not preserve application message boundaries. A single
`send()` can be received in fragments or several sends can be coalesced. The
helper therefore uses `async_read` for exact lengths and places this 24-byte,
network-byte-order header before every payload:

| Offset | Width | Field |
|---:|---:|---|
| 0 | 4 | Magic `GMPT` |
| 4 | 2 | Protocol version (`1`) |
| 6 | 2 | Frame kind |
| 8 | 4 | Flags/reserved |
| 12 | 8 | Transaction sequence |
| 20 | 4 | Payload byte count |

One full-duplex connection multiplexes data requests/responses, commands,
heartbeats, and error frames. `TCP_NODELAY` is enabled by default because SIL
uses small causal request/response transfers.

## Failure contract

- Connect, accept, read, and write operations have finite configurable
  timeouts. A silent or disconnected peer returns an explicit error instead of
  blocking indefinitely.
- A timeout or partial frame closes the stream. Continuing on that stream would
  make the next frame boundary ambiguous.
- Payload size is checked before allocation.
- Transaction responses must have the expected sequence and frame kind.
- Transactions are never retried automatically. TCP removes ordinary packet
  loss, but after a connection break the caller cannot know whether the remote
  controller executed the final request. Automatic replay could execute a
  control tick twice.

The current class deliberately serializes operations on each helper object. It
is intended for the causal SIL pattern `request -> controller step -> response`,
not for several concurrent requests in flight.

## Configuration

`parse_configuration()` accepts JSON in this form:

```json
{
  "transport": "tcp",
  "role": "client",
  "target_address": "192.168.1.20",
  "bind_address": "0.0.0.0",
  "port": 12510,
  "connect_timeout_ms": 5000,
  "io_timeout_ms": 5000,
  "max_payload": 16777216,
  "no_delay": true,
  "keep_alive": true
}
```

The server uses `bind_address`; the client uses `target_address`. IPv4 numeric
addresses are required in this prototype so that name-resolution behavior
cannot add an unbounded external dependency.

## Build and test

The module is header-only apart from its test executable. From the repository
root on Windows:

```powershell
cmake -S tools/gmp_sil/tcp_helper_v2 `
      -B tools/gmp_sil/tcp_helper_v2/build `
      -DCMAKE_TOOLCHAIN_FILE=bin/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build tools/gmp_sil/tcp_helper_v2/build --config Release
ctest --test-dir tools/gmp_sil/tcp_helper_v2/build -C Release --output-on-failure
```

Tests cover stable header encoding, 512 binary SIL-sized request/response
transactions, intentionally fragmented TCP writes, bounded timeout, partial
disconnect, oversized-frame rejection, heartbeat, and JSON parsing.
Transaction sequence mismatch and malformed frame magic are rejected as well.

## Integration boundary

No MATLAB MEX/S-Function, Simulink mask, UDP compatibility shim, native Suite
controller binding, or installer entry is included yet. A future integration
must define which side listens, map one controller execution to one sequenced
`data_request`, retain the major-step guard, and decide how the user chooses
UDP versus TCP without changing the packed SIL payload ABI.
