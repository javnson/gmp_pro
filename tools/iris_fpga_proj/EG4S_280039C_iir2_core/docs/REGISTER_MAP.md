# AXI4-Lite register map

[中文](REGISTER_MAP_CN.md)

All addresses are byte offsets and all words are little-endian 32-bit values.

| Offset | Name | Access | Description |
| --- | --- | --- | --- |
| `0x000` | ID | RO | `0x49325232` |
| `0x004` | VERSION | RO | `1.0` |
| `0x008` | CONTROL | RW/W1P | bit0 run, bit1 clear IIR state, bit2 auto ADC, bit3 auto DAC, bit8 clear counters/errors |
| `0x00C` | STATUS | RO | busy, manual pending, config error, result seen |
| `0x010` | MANUAL_SAMPLE | RW | Q3.28 write enqueues one sample |
| `0x014` | LATEST_RESULT | RO | most recently consumed result |
| `0x018` | SAMPLE_COUNT | RO | completed output samples |
| `0x01C` | SAT_COUNT | RO | clock cycles containing at least one section saturation |
| `0x020` | ACTIVE_SECTIONS | RW shadow | `0..NUM_SECTIONS` |
| `0x024` | COMMIT | W1P | bit0 atomically commits all coefficients while idle and clears state |
| `0x028` | FORMAT | RO | data width in bits31:24, fraction width in bits23:16 |
| `0x02C` | NUM_SECTIONS | RO | synthesized maximum |
| `0x030` | ADC_CONTROL | RW | mask bits7:0, selected channel bits18:16, auto ADC bit19, auto DAC bit20 |
| `0x034` | ADC_SELECTED | RO | selected raw ADC sample |
| `0x038` | PWM_STATUS | RO | trip latch bit0 |
| `0x040 + 0x14*n` | SOS[n].B0 | RW shadow | Q3.28 |
| `+0x04/+0x08` | SOS[n].B1/B2 | RW shadow | Q3.28 |
| `+0x0C/+0x10` | SOS[n].A1/A2 | RW shadow | denominator values used with subtraction |
| `0x0A0` | PWM_CONTROL | RW/W1P | enable, center mode, result-to-compare; bit8 clears trip |
| `0x0A4` | PWM_PERIOD | RW | timer period clocks |
| `0x0A8` | PWM_COMPARE | RW | shadow compare clocks |
| `0x0AC` | PWM_DEADTIME | RW | dead-time clocks |
| `0x0B0` | PWM_PHASE | RW | phase load value |
| `0x0B4/0x0B8` | DAC_MANUAL | RW | packed channel pairs |
| `0x0C0..0x0DC` | ADC_CH0..7 | RO | raw ADC samples |
| `0x0E0` | DMA_CONTROL | RW/W1P | bit0 memory mode, bit1 start, bit2 identification route |
| `0x0E4` | DMA_READ_BASE | RW | 4-byte-aligned source address |
| `0x0E8` | DMA_WRITE_BASE | RW | 4-byte-aligned destination address |
| `0x0EC` | DMA_SAMPLE_COUNT | RW | number of 32-bit samples |
| `0x0F0` | DMA_STATUS | RO | read busy, write busy, done-seen, error |

TD SPI uses a 16-bit command followed by a 16-bit data word. Command bit15 is
read, bits14:8 are the 16-bit register index. Two adjacent SPI indices form one
AXI word; even is the low half and odd is the high half. The FPGA system clock
must be at least four times the SPI clock so a read can complete before its data phase.
