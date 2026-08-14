# GMP Data Link Protocol

Chinese documentation: [readme_dl_protocol_cn.md](readme_dl_protocol_cn.md)

## Purpose

GMP Data Link (DL) is a framed transport used by PIL, Tunable Parameters,
Memory Perspective, and Data Link Scope. It supports conventional byte-addressed processors and
TI C28x-style targets whose smallest C addressable unit is 16 bits.

The public API and project source list are platform-independent. Applications
always include the canonical headers:

```c
#include <core/dev/datalink/datalink.h>
#include <core/dev/datalink/tunable.h>
#include <core/dev/datalink/mem_presp.h>
#include <core/dev/datalink/scope.h>
```

Projects that use these services always compile the same source files:

```text
core/dev/datalink/src/gmp_datalink.c
core/dev/datalink/src/gmp_tunable.c
core/dev/datalink/src/gmp_mem_presp.c
core/dev/datalink/src/gmp_scope.c
```

## Backend selection

The canonical headers and sources select a backend using
`GMP_PORT_DATA_SIZE_PER_BYTES`:

| Macro value | Backend | Target model |
| --- | --- | --- |
| `1` | `u8` | Byte-addressed CPUs such as STM32, x86, and Arm |
| `2` | `u16` | 16-bit-addressed CPUs such as TI C28x |

Any other value produces a compile-time error. The backend implementation files
live in `core/dev/datalink/`; applications must not include them directly.

The u8 backend uses `uint8_t` directly for protocol storage and APIs; it does
not introduce a second byte alias. Platform-neutral application and transport
code uses `byte_gt`, the GMP addressable data unit. Consequently `byte_gt` is
an 8-bit unit on STM32 and a 16-bit unit on C28x. The backend selector prevents
the u8 implementation from compiling when the platform unit is not one byte.

## Stable wire format

Both backends exchange the same sequence of 8-bit protocol octets on the wire:

```text
'{' escaped-header '}' [payload payload-crc16]
```

The decoded six-octet header is:

| Offset | Size | Field |
| --- | ---: | --- |
| 0 | 1 | Sequence ID |
| 1 | 1 | Command |
| 2 | 2 | Payload length, little-endian |
| 4 | 2 | CRC16-CCITT of header octets 0 through 3 |

Header octets equal to `{` (`0x7B`), `}` (`0x7D`), or `%` (`0x25`) are escaped
as `%` followed by the octet XOR `0x20`. Payload data is not escaped. A nonempty
payload is followed by a little-endian CRC16-CCITT; an empty payload has no
payload CRC. The CRC initial value is `0xFFFF` and the polynomial is `0x1021`.

## Runtime integration

Initialize one context and feed received protocol units into its FIFO from an
ISR or DMA callback:

```c
gmp_datalink_t dl;

void app_init(void)
{
    gmp_dev_dl_init(&dl);
}

void uart_rx_callback(const byte_gt* data, size_gt count)
{
    gmp_dev_dl_push_str(&dl, data, count);
}
```

Call `gmp_dev_dl_loop_cb()` from a task or main loop. On
`GMP_DL_EVENT_RX_OK`, offer the frame to application services and finally call
`gmp_dev_dl_default_rx_handler()` if no service claimed it. On
`GMP_DL_EVENT_TX_RDY`, transmit the header and payload buffers and call
`gmp_dev_dl_tx_state_done()` only after the hardware has finished using them.

DMA implementations may transmit the escaped header and payload as two chained
transfers. Circular RX DMA should forward every new span at half-transfer,
transfer-complete, and/or idle events so continuous streams do not depend on an
idle gap.

An interrupt-driven transport must have exactly one owner of the hardware RX
FIFO at a time. If both a background poll and an RX interrupt drain the FIFO,
serialize them with a short critical section and use non-blocking hardware
reads. Never snapshot a FIFO depth and then perform a blocking read: the other
context can consume that depth first and leave the control application blocked.
On framing, parity, break, or overrun errors, reset the peripheral FIFO and call
`gmp_dev_dl_request_rx_reset()` from the ISR. The protocol task then discards
the incomplete software queue and resumes searching for the next start marker.

## Tunable Parameters

Tunable Parameters maps small integer IDs to a static whitelist. Read is
`base_cmd`; write is `base_cmd + 1`. A one-unit request on `base_cmd + 1` is an
indexed descriptor query; useful write requests are longer and remain
unambiguous. Version 2 dictionary descriptors report only ID, native type,
read-only or read-write permission, and name:

```text
[version=2:u8][status:u8][total:u8][id:u8][type:u8][permission:u8]
[name-length:u8][name...]
```

Put an engineering unit in the optional display name when it is useful, for
example `Signal Frequency (Hz)`. This avoids one pointer per registered item on
the embedded target. A `NULL`, `0`, or empty name is valid and the host generates
`Parameter N` automatically. The host also accepts legacy version 1 descriptors. The
u8 backend uses `memcpy` for native values so unaligned or
strict-aliasing-sensitive CPUs remain safe.

## Memory Perspective

Memory Perspective exposes only explicitly registered regions. Read is
`base_cmd`; write is `base_cmd + 1`. A one-unit request on `base_cmd + 1` is an
indexed descriptor query. Region descriptors report name, address, byte length,
and permission only:

```text
[version=2:u8][status:u8][total:u8][id:u8][address:u32][byte-length:u32]
[permission:u8][name-length:u8][name...]
```

The host also accepts legacy version 1 descriptors but ignores their obsolete
display metadata. Data interpretation is selected in the Memory page, while
waveform shape and sample metadata belong exclusively to Data Link Scope.
A `NULL`, `0`, or empty region name is valid and appears as `Memory Region N`.
Normal read/write requests use:

```text
[address:u32][item-size:u8][item-count:u16][write-data...]
```

The address and lengths in the protocol are always byte quantities. On a u8
target, the address is the native byte address. On a C28x/u16 target, the host
tool continues to use the historical byte-address convention, and the backend
translates it to native word addresses. Only item sizes 1, 2, and 4 are valid.
The complete access must fit inside one registered region and must satisfy its
permission.

## Data Link Scope

Data Link Scope is independent of Memory Perspective. A target registers named
waveform resources, immutable sample metadata, and optional configure, arm, and
status callbacks. The host never needs a physical buffer address.

One service command carries an operation byte:

| Operation | Value | Purpose |
| --- | ---: | --- |
| Discover | 0 | Query one indexed named scope resource |
| Configure | 1 | Set mode, channel, level, trigger position, auto timeout, and sampling divider |
| Arm | 2 | Reset and start one capture |
| Status | 3 | Read waiting/capturing/ready state and generation |
| Read | 4 | Read a bounded byte range from the registered snapshot |

This single-command design keeps command allocation compact while maintaining
one host page and one target module per tool.

Scope protocol version 2 appends a little-endian 16-bit sampling divider to the
Configure request. The effective sample rate is the reported base rate divided
by `divider + 1`; consequently, the default value `0` samples every control
tick. Targets accept the version 1 Configure request without this field and
apply a divider of zero, while the host emits the older request for a target
that reports Scope protocol version 1.

The snapshot buffer is readable only in the ready state. It remains owned by
the host throughout all chunked reads, and the target does not capture into it
again until a later Arm request. Host implementations should bound and retry
Configure, Arm, and Read transactions because packet loss is permitted. While
an edge-triggered capture is waiting, a new configuration may be queued and
applied after the outstanding status transaction finishes; the user must not be
locked into an unreachable trigger condition.

## Host tools and validation target

- `tools/gmp_pil_server/gmp_debugger/run_u8.bat` selects the byte-addressed
  target profile.
- `tools/gmp_pil_server/gmp_debugger/run_u16.bat` selects the DSP/C28x target
  profile.
- `tools/gmp_pil_server/gmp_debugger/apis` provides the documented headless
  Python API for automation and AI-assisted hardware debugging.
- `tools/gmp_pil_server/stm32_dl_dbger` is the NUCLEO-C092RC u8 validation
  firmware and hardware smoke test.

The two launchers share one debugger and stable wire codec. Their distinction
documents the target memory-address model rather than defining a different
wire protocol.
