# Test and user report

[中文](TEST_AND_USER_REPORT_CN.md)

## Validation result

Validation was run on 2026-08-04 with Icarus Verilog 12.0 (development build)
and Python 3.13.0. `tests/run_tests.ps1` passed:

| Test | Covered behavior | Result |
| --- | --- | --- |
| `tb_iir2_operator` | recursive impulse response, TLAST, saturation | PASS |
| `tb_iir2_pipeline` | two SOS sections, cascade value, stalled-output stability | PASS |
| `tb_control_core_axi` | AXI4-Lite read/write, identity default, shadow commit, result counter | PASS |
| `tb_epwm_modulator` | PWM period events, complementary dead time, trip latch/clear | PASS |
| `tb_data_adapters` | ADC offset-binary/Q3.28 conversion and DAC inverse mapping | PASS |
| `tb_spi_axi_bridge` | TD SPI reads both AXI halfwords and writes a control register | PASS |
| `tb_external_memory_system` | two-burst control, identification bypass/capture, unaligned-request rejection | PASS |
| TD top elaboration | all board modules with a simulated clock-manager stub | PASS |
| `test_host_tools.py` | fixed-point/TF conversion, register image, chirp and response analysis | PASS (5 tests) |

The complete suite returned exit code 0. Icarus reported no RTL warnings in the
final run. AMD Vivado is not installed in this environment. The installed TD
executable is 5.0 while the existing project metadata requests TD 6.2, so no TD
place-and-route or timing result is claimed. Physical ADC, DAC, PWM, external
memory, and closed-loop hardware validation remains target-board work.

## Live controller use

1. Integrate `iir2_control_core` and connect its AXI4-Lite and AXI4-Stream ports.
2. Write all five coefficients for every section into the shadow window.
3. Write `ACTIVE_SECTIONS`, then write `1` to `COMMIT`. Poll `STATUS.busy=0`
   before commit. Commit clears recurrence state.
4. Set `CONTROL.run=1`, then exchange Q3.28 samples over AXI4-Stream.
5. Use `LATEST_RESULT`, `SAMPLE_COUNT`, and `SAT_COUNT` for monitoring.

The default coefficient set is identity. A section count of zero bypasses all
sections while preserving the stream handshake.

## External-memory batch use

1. Integrate `iir2_external_memory_system`; connect its full AXI4 master to the
   platform memory controller. Source and destination buffers must be 4-byte
   aligned, non-overlapping, accessible to that master, and large enough for
   `count * 4` bytes.
2. On systems with CPU caches, flush the source buffer before DMA and invalidate
   the destination after completion, or allocate coherent memory.
3. Write `DMA_READ_BASE`, `DMA_WRITE_BASE`, and `DMA_SAMPLE_COUNT`.
4. Write `DMA_CONTROL=0x3` to select controller batch mode and start. Poll
   `DMA_STATUS` until both busy bits clear and done-seen is set; reject error.

In batch mode, each external-memory input word passes through the configured SOS
cascade and each result is written to the destination buffer. Payload data is
never buffered in FPGA BRAM.

## Injection identification use

Use the external-memory wrapper's injection and measurement AXI4-Stream ports.
Writing `DMA_CONTROL=0x7` routes DDR reads to `m_axis_injection_*` (typically a
DAC scaling/output block) and routes `s_axis_measurement_*` (typically timestamped
ADC samples) to the DDR write channel. Both streams contain exactly
`DMA_SAMPLE_COUNT` words and must assert TLAST on the final word.

Generate a logarithmic injection and a DDR-ready little-endian Q3.28 image:

```powershell
python host\gmp_fpga_control.py make-injection --sample-rate 20000 --duration 10 `
  --f-start 1 --f-stop 5000 --amplitude 0.05 --csv injection.csv --binary injection.bin
```

After capture, prepare a CSV with `input` and `output` columns and estimate the
frequency response:

```powershell
python host\gmp_fpga_control.py analyze --input measurement.csv `
  --sample-rate 20000 --output frequency_response.csv
```

Injection amplitude, power-stage limits, trip logic, sampling coherence, and
anti-alias filtering must be commissioned for the actual plant before enabling
power hardware.

## Controller compilation

Compile scipy-format SOS rows from the example file:

```powershell
python host\gmp_fpga_control.py compile --sos-json host\example_sos.json `
  --output controller_registers.json
```

Or, with SciPy installed, compile a transfer function directly:

```powershell
python host\gmp_fpga_control.py compile --numerator "0.1,0.1" `
  --denominator "1,-0.8" --output controller_registers.json
```

The JSON contains floating values, Q3.28 words, AXI byte addresses, and TD SPI
transactions. Coefficients outside the representable range are rejected.
