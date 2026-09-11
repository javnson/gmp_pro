# H753ZI acceptance record

The target is accepted only when all of the following pass:

1. IOC/schema/entity consistency validation and SDPE generation.
2. Clean GCC build with the repository-local GMP source manager output.
3. SWD erase/program/verify/reset through the onboard ST-Link.
4. GMP Data Link discovery, PIL, Tunable, Memory and Scope tests over USART3,
   TCP, and UDP.
5. A 20 kHz ADC DMA callback rate while PWM output state remains zero.
6. Ethernet link-up, raw UDP echo on `192.168.137.2:50000`, TCP Data Link on
   port `50001`, and UDP Data Link on port `50002`.
7. Concurrent USART3 plus TCP and concurrent USART3 plus UDP transactions with
   independent protocol state and shared application resources.

The USB Ethernet adapter used for the reference test is configured as
`192.168.137.1/24`. No power stage is connected or driven by this test.

## Reference hardware result (2026-09-11)

- Board/ST-Link: NUCLEO-H753ZI, V3J5M2, SWD program/verify/reset passed.
- Build: both TCP and UDP GCC release configurations passed; IOC and all 31
  schemas / 79 entities validated.
- GMP DL: COM71 at 921600 baud; discovery, PIL, Tunable, Memory, Scope, DMA
  stress, CRC rejection and recovery passed.
- Control path: measured ADC DMA callback rate 20,136.4 Hz; LED heartbeat and
  six ADC samples valid; PWM output state remained zero.
- Ethernet: 100 Mbps link; UDP echo at `192.168.137.2:50000` passed with RX/TX
  counters `1/1`, 25 payload bytes, and no Ethernet errors.
- TCP DL: full u8 acceptance suite passed at `192.168.137.2:50001`; observed
  DL RX/TX counters `38/36`, byte counters `2879/3150`, and zero DL errors.
- UDP DL: the same suite passed at `192.168.137.2:50002`; observed DL RX/TX
  counters `38/36`, byte counters `2879/3150`, and zero DL errors.
- Both runs preserved the 20 kHz control ISR, six ADC channels, LED heartbeat,
  CRC rejection/recovery, and disabled physical PWM output state.
- Concurrent UART+TCP and UART+UDP tests each completed 48 synchronized ECHO
  transactions per link, including 256-byte bursts. Both endpoint facility
  tables matched, cross-link Tunable write/read/restore passed, and all link
  error counters remained zero.
