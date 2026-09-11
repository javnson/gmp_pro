# H755ZI-Q dual-core acceptance record

The target is accepted only when all of the following pass:

1. CubeMX open/save/generate round-trip plus IOC/schema/entity validation.
2. GCC Release builds for both CM7 and CM4 with TCP and UDP selected through
   `GMP_DATALINK_TRANSPORT`.
3. SWD erase/program/verify/reset of CM7 bank 1 and CM4 bank 2 through the
   onboard ST-Link.
4. CM7 owns TIM1, TIM3, ADC1, Ethernet and its scheduler; CM4 owns USART3,
   I2C1 and its scheduler.
5. GMP Data Link discovery, PIL, Tunable, Memory and Scope over CM4 USART3 and
   over CM7 TCP or UDP.
6. A 20 kHz CM7 ADC DMA callback rate while physical PWM outputs remain off.
7. Ethernet link-up, raw UDP echo on `192.168.137.3:50000`, TCP Data Link on
   port `50001`, and UDP Data Link on port `50002`.
8. Concurrent USART3 plus TCP and USART3 plus UDP transactions, with both
   scheduler heartbeats and the CM7 control heartbeat advancing in shared SRAM4.

The USB Ethernet adapter used for the reference test is configured as
`192.168.137.1/24`. No power stage is connected or driven by this test.

## Reference hardware result (2026-09-11)

- Board/ST-Link: NUCLEO-H755ZI-Q, serial `003F001A3234510C33353533`, firmware
  V3J5M2. Dual-bank SWD program/verify/reset passed.
- CubeMX 6.17.0 successfully reopened, saved and regenerated the project. The
  six ADC ranks, three DMA requests, CM7 SWD pins, per-pin core assignments,
  CM4 I2C1 and all RMII pins remained serialized in the IOC.
- Ethernet RX descriptors use the low D2 SRAM reservation at `0x30000000`;
  TX descriptors and receive buffers use SRAM3 from `0x30040060`. CM4 RAM uses
  the non-overlapping alias range `0x10008000–0x1003ffff`.
- Build: both CM7 and CM4 GCC Release images passed for TCP and UDP. The board
  entity and all 31 SDPE schemas / 80 entities validated.
- CM4 serial DL: COM72 at 921600 baud; discovery, PIL, Tunable, Memory, Scope,
  DMA stress, CRC rejection and recovery passed. Observed UART RX/TX callbacks
  were `62/76`, with zero UART errors.
- CM7 TCP DL: full u8 suite passed at `192.168.137.3:50001`; observed control
  ISR rate was 20,247.9 Hz, DL RX/TX counters `40/38`, byte counters
  `2907/3217`, and zero DL errors.
- CM7 UDP DL: full u8 suite passed at `192.168.137.3:50002`; observed control
  ISR rate was 20,244.7 Hz, DL RX/TX counters `40/38`, byte counters
  `2907/3217`, and zero DL errors.
- Raw UDP echo and ICMP ping passed with the 100 Mbps Ethernet link up. All
  runs kept physical PWM outputs disabled.
- Concurrent UART+TCP and UART+UDP tests each completed 48 synchronized ECHO
  transactions per link, including 256-byte bursts. Both endpoint facility
  tables matched, each endpoint's Tunable write/read/restore passed, all link
  error counters stayed zero, and shared CM7/CM4 scheduler plus CM7 control
  counters advanced.
