# CAN communication definition

All signed 32-bit values use a scale factor of 10000 and are transmitted as
two 16-bit C2000 words. Power, voltage, current, phase and modulation values
are per-unit unless an explicit physical unit is shown.

| Mailbox | CAN ID | Direction | Payload word 0 (bytes 0-3) | Payload word 1 (bytes 4-7) |
| --- | --- | --- | --- | --- |
| 1 | 0x101 | RX | Enable command: 0 = disable voltage, 1 = enable operation | Reserved |
| 2 | 0x102 | RX | Active-power reference P (PU) | Reactive-power reference Q (PU) |
| 3 | 0x103 | RX | Reserved | Reserved |
| 4 | 0x201 | TX | Grid voltage peak magnitude (PU) | PLL frequency (Hz) |
| 5 | 0x202 | TX | Measured active power P (PU) | Measured reactive power Q (PU) |
| 6 | 0x203 | TX | Grid voltage RMS (PU) | AC current RMS (PU) |
| 7 | 0x204 | TX | Instantaneous current reference (PU) | Saturated modulation reference (PU) |
| 8 | 0x205 | TX | DC bus voltage (PU) | Reserved |
| 9 | 0x206 | TX | PLL phase angle (PU/revolution) | Grid-voltage feedforward (PU) |
| 10 | 0x207 | TX | CiA402 status word | Active protection error bitmap |

The SCI debug/datalink interface uses 921600 baud.
