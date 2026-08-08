# GMP Data Link Protocol

Chinese documentation: [readme_dl_protocol_cn.md](readme_dl_protocol_cn.md)

## Purpose

GMP Data Link (DL) is a framed transport used by PIL, Tunable Parameters, and
Memory Perspective. It supports both conventional byte-addressed processors and
TI C28x-style targets whose smallest C addressable unit is 16 bits.

The public API and project source list are platform-independent. Applications
always include the canonical headers:

```c
#include <core/dev/datalink.h>
#include <core/dev/tunable.h>
#include <core/dev/mem_presp.h>
```

Projects that use these services always compile the same source files:

```text
core/dev/src/gmp_datalink.c
core/dev/src/gmp_tunable.c
core/dev/src/gmp_mem_presp.c
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

`gmp_dl_octet_t` is the physical Data Link transfer unit. It is `uint8_t` for
the u8 backend and retains the legacy `data_gt` representation for the u16
backend. UART ingress and egress code should use this type or the public buffer
accessors instead of assuming that `data_gt` is one physical byte.

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

void uart_rx_callback(const gmp_dl_octet_t* data, size_gt count)
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

## Tunable Parameters

Tunable Parameters maps small integer IDs to a static whitelist. Read is
`base_cmd`; write is `base_cmd + 1`. Dictionary entries declare their native
type and read-only or read-write permission. The u8 backend uses `memcpy` for
native values so unaligned or strict-aliasing-sensitive CPUs remain safe.

## Memory Perspective

Memory Perspective exposes only explicitly registered regions. Read is
`base_cmd`; write is `base_cmd + 1`. Requests use:

```text
[address:u32][item-size:u8][item-count:u16][write-data...]
```

The address and lengths in the protocol are always byte quantities. On a u8
target, the address is the native byte address. On a C28x/u16 target, the host
tool continues to use the historical byte-address convention, and the backend
translates it to native word addresses. Only item sizes 1, 2, and 4 are valid.
The complete access must fit inside one registered region and must satisfy its
permission.

## Host tools and validation target

- `tools/gmp_pil_server/gmp_debugger_u8` is the normal debugger entry point for
  byte-addressed processors.
- `tools/gmp_pil_server/gmp_debugger_u16` preserves the DSP/C28x address model.
- `tools/gmp_pil_server/stm32_dl_dbger` is the NUCLEO-C092RC u8 validation
  firmware and hardware smoke test.

The two debuggers share the stable wire codec. Their distinction documents and
enforces the target memory-address model rather than defining a different wire
protocol.
