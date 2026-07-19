# GMP Peripheral Router Protocol 1

The transport carries a little-endian packet followed by CRC-16/CCITT-FALSE.
Serial links COBS-encode the complete packet and append one zero delimiter. A
packet header is 18 bytes:

| Offset | Type | Field |
| ---: | --- | --- |
| 0 | `u16` | magic `0x4752` |
| 2 | `u8` | protocol version |
| 3 | `u8` | request, response, or event |
| 4 | `u32` | request sequence |
| 8 | `u8` | peripheral family |
| 9 | `u8` | operation |
| 10 | `u16` | logical endpoint |
| 12 | `u16` | physical channel |
| 14 | `i16` | status |
| 16 | `u16` | payload length |

`endpoint` distinguishes independently routed functions on one board. `channel`
selects the peripheral instance or GPIO pin. The host service adds a device name,
so `(device, endpoint, channel)` remains unambiguous with many attached boards.

## CAN payload ABI

CAN uses synchronous requests for configuration and asynchronous event messages
for traffic and state changes. Integers remain little-endian.

- capabilities: `u32 features`, six `u16` values (TX slots, RX FIFOs, filter
  slots, maximum data length, timestamp width, reserved), then two `u32` maximum
  bitrates;
- configuration: two `u32` bitrates followed by four `u16` values (nominal and
  data sample points in permille, flags, mode);
- transmit frame: `u32 token`, `u32 identifier`, `u16 flags`, `u16 length`, then
  0 to 64 payload bytes;
- RX event: `u32 timestamp`, `u16 filter index`, `u16 metadata flags`, `u32
  identifier`, `u16 frame flags`, `u16 length`, then the frame payload;
- filter: `u16 slot`, two `u32` identifier values, then `u16 flags` and mask;
- state: three `u16` values (state, TX errors, RX errors), two reserved bytes,
  then four `u32` diagnostic counters;
- TX completion event: `u32 token`, `i16 status`, two reserved bytes, `u32 timestamp`;
- state event: the 24-byte state payload followed by a `u32 timestamp`.

Classic CAN and CAN FD share the frame layout. Flag bits match
`core/dev/can/can.h`. Boards without CAN return `UNSUPPORTED` and advertise zero
CAN channels in HELLO; they must not emulate a successful transmission.
