# Architecture

[中文](ARCHITECTURE_CN.md)

## Data paths

Live control uses `ADC -> adc_axis_adapter -> iir2_control_core -> DAC/ePWM`.
Batch identification uses `external memory -> AXI4 MM2S -> IIR2 AXI4-Stream -> AXI4 S2MM -> external memory`.

AXI4-Lite was selected for configuration because coefficient and peripheral
registers need no bursts. AXI4-Stream carries ordered samples with lossless
backpressure. Full AXI4 is reserved for bulk external-memory traffic.

The DMA stores only addresses, counters, and handshake state. It infers no
payload RAM, uses INCR bursts of up to `MAX_BURST_LEN`, never crosses a 4 KiB
boundary, and supports concurrent read/write channels with one outstanding
burst per direction.

## IIR2 arithmetic

The default format is signed 32-bit Q3.28. Each section implements:

`y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]`

Five multiplications execute in parallel. Products, the widened adder, and the
scaled/saturated result are registered. A section has a three-edge calculation
pipeline and accepts a sample every three clocks. Sections in the cascade work
on different samples concurrently. At 100 MHz, the first-section initiation
capacity is about 33.3 Msamples/s, well above normal power-control rates.

Coefficients use shadow registers and an idle-only atomic commit. The small
coefficient and recurrence-state set remains in registers: placing feedback
state in external memory would make real-time latency dependent on memory
arbitration. Bulk waveform, capture, and result storage is external.

## Portability boundary

All control, DMA, AXI, and modulation RTL is vendor-neutral Verilog. For AMD
Xilinx, connect the AXI4 master to MIG/SmartConnect or a Zynq PS HP/HPC port and
the AXI4-Lite slave to the processor interconnect. For TD, connect the same AXI4
master to the selected external-memory controller/adapter. The checked-in TD
board top currently exercises the live ADC/DAC path; external memory has no pin
definition in the supplied EG4S20 board constraints, so its controller and pins
must be supplied by the target board design.
