# GMP F28388D ControlCARD three-core reference

[中文说明](README_CN.md)

This target is the standard GMP reference for the TMDSCNCD28388D ControlCARD.
It preserves the C28x system-u16 data model on CPU1, CPU2, and the Cortex-M4
Communication Manager (CM), so Data Link structures and command payloads have
one ABI across all three cores.

## Core ownership

| Core | Runtime role | GMP scheduler | Owned interfaces |
| --- | --- | --- | --- |
| CPU1 | Control I/O, multicore boot, serial Data Link | Yes | ePWM, ADC, eQEP, SCI-A, I2C-A, SPI-C/SD, DAC, D1/D2 |
| CPU2 | Deterministic sine/cosine calculation | Yes | CPU timer and CPU1/CM message RAM |
| CM | Dedicated communication processor | Yes | Ethernet, USB, EtherCAT and Ethernet Data Link |

CPU1 is the single board-initialization authority: it configures communication
clocks, resets, pin mux, USB device mode, EtherCAT ESC memory, and shared-
peripheral allocation before booting the other cores. Runtime communication
ownership is then held by CM; neither C28x CPU services those protocols.

## ControlCARD resource contract

- EPWM1/2/3 provide three complementary PWM pairs. EPWM1 SOCA triggers all six
  feedback conversions.
- ADCA3/4/5 and ADCC2/3/4 are the fixed feedback channels.
- EQEP1 provides the ABZ/index capture interface.
- SCI-A is routed to the XDS100 virtual COM port and carries system-u16 GMP DL
  at 115200 baud.
- I2C-A supplies the general board-control bus.
- SPI-C plus GPIO103 supplies the SD socket baseline.
- DACA/B/C are initialized as optional analog debug outputs.
- D1 and D2 are initialized. CAN-A is deliberately omitted from the default
  variant because its standard ControlCARD TX routing conflicts with D1/GPIO31.

The editable resource definitions are `cpu1.syscfg` and `cpu2.syscfg`. CPU2 is
intentionally pin-free. Re-open and save these files with the C2000Ware 5.04
SysConfig version used by the project specification.

## Data Link endpoints

The example keeps both debug paths active:

| Link | Owner | Address | Profile |
| --- | --- | --- | --- |
| SCI-A | CPU1 | XDS virtual COM, 115200 8-N-1 | system-u16 |
| Ethernet TCP | CM | `192.168.137.2:50001` | system-u16 |
| Ethernet UDP | CM | `192.168.137.2:50002` | system-u16 |

Select TCP or UDP by building the CM `CM_FLASH_TCP_U16` or
`CM_FLASH_UDP_U16` configuration. The host adapter should normally use
`192.168.137.1/24` with no gateway. CM explicitly packs the low octet of each
u16 Data Link unit onto the Ethernet byte stream and expands received octets
back to u16 units.

Both links expose the same command bases: Tunable `0x30`, Memory `0x50`, and
Scope `0x60`. They can update CPU2's sine frequency, gain, and offset and can
capture the resulting two-channel waveform.

## Configuration and generated sources

- `src/sdpe_mgr/sdpe_requirement.json` is the project SDPE source.
- The reusable board entity and schema live in `../sdpe_component`.
- `src/common/ctrl_settings.h` and its hardware preset are generated outputs.
- One root `gmp_src_mgr` supplies identical GMP library sources to all cores.
- `src/common/tricore_shared.h` is the message-RAM ABI shared by CPU1, CPU2,
  and CM; its compile-time checks compare bit sizes rather than C `sizeof`
  units.

Each core follows the standard GMP application/platform split:

```text
src/<core>/user/user_main.c,h       # GMP facilities, tasks, portable algorithm
src/<core>/xplt/xplt.config.h       # CSP selection and build configuration
src/<core>/xplt/xplt.peripheral.c,h # registers, ISRs, pins, transport binding
src/<core>/xplt/xplt.ctl_interface.h# CTL platform extension point
```

All three `user` layers enter GMP through the common
`setup_peripheral -> init -> mainloop` lifecycle and do not access DriverLib,
lwIP, or board registers. Project import preserves these as `src/user` and
`src/xplt`, so the application layer has the same shape as other standard GMP
targets.

Run the supported build entry point from this directory:

```powershell
.\tools\build.ps1
```

It validates and regenerates SDPE, regenerates common GMP sources, imports the
three CCS projects into a fresh workspace, builds CPU1, CPU2, and both CM
network variants, then copies the four images to the ignored `artifacts`
directory.

## Flash and verify

Flash CM first, CPU2 second, and CPU1 last. The script then resumes all three
debug sessions, allowing CPU1 and CM to cross both IPC boot barriers before
resuming CPU2. This preserves CPU1-owned peripheral initialization while
removing ambiguity when UniFlash/GEL leaves a secondary boot ROM halted.

```powershell
.\tools\flash.ps1 -EthernetProtocol Tcp
.\tools\test_dl.ps1 -Link Both -EthernetProtocol Tcp -SerialPort COM74
```

For UDP, rebuild both variants once, then select `Udp` in both commands. Add
`-CaptureScope` to the test command to acquire and save a waveform after the
Tunable, Memory, and Scope discovery checks pass.

Use the same entry point for sustained triggering and TCP resource-reclamation
stress:

```powershell
.\tools\test_dl.ps1 -Link Ethernet -EthernetProtocol Tcp `
    -StressCaptures 200 -ReconnectCycles 100
```

This repeatedly configures, arms, and downloads Scope on one connection while
interleaving Tunable and Memory reads every ten frames, then creates fresh TCP
connections to verify that the target continues accepting clients.

## Hardware validation

The following matrix was exercised on a TMDSCNCD28388D ControlCARD on
2026-09-12. The XDS virtual serial port was `COM74`; the host USB Ethernet
adapter was `192.168.137.1/24`. After the debugger-assisted diagnostics, all
three images were programmed and verified in CM -> CPU2 -> CPU1 order; the
serial and TCP rows were repeated successfully from that Flash-started image
without an active DSS debug session.

| Path | Checks completed | Result |
| --- | --- | --- |
| CPU1 SCI-A system-u16 | Tunable discovery/readback, Memory discovery/readback, two-channel 400-sample Scope capture | Pass |
| CM Ethernet TCP system-u16 | Tunable discovery/readback, Memory discovery/readback, two-channel 400-sample Scope capture | Pass |
| CM Ethernet UDP system-u16 | Tunable discovery/readback, Memory discovery/readback, two-channel 400-sample Scope capture | Pass |

The sustained regression completed 200 consecutive two-channel, 400-sample
captures on one TCP connection, with generation advancing frame-by-frame to
201. It then completed 20 connection changes; a separate rapid test
completed 100/100 reconnects. CPU1 serial completed 50 captures and UDP
completed 100 captures. Every generation was consecutive, including the
interleaved Tunable and Memory reads. The final CM Flash image is the verified
TCP system-u16 variant.

The TI `NO_SYS` Ethernet port delivers packets from the EMAC interrupt path.
Its callback now only copies network octets into a 2048-byte single-producer,
single-consumer ring; the CM scheduler drains that ring into GMP DL, keeping
the parser and response state machine in one execution context. Scheduler
access to the lwIP raw API remains protected with the port's
`SYS_ARCH_PROTECT` primitive. TCP close also detaches callbacks and explicitly
aborts a PCB when `tcp_close` cannot reclaim it. If a new connection arrives
before the old connection's FIN, the listener reclaims the stale PCB and
adopts the newest client, preventing gradual resource loss during sustained
captures or rapid reconnects.

The original hang risk was concurrent mutation of one GMP DL parser/transmit
state from the EMAC interrupt callback and the CM scheduler, not only
concurrent lwIP PCB access. Scope `state/generation` readback is also protected
by a critical section so a C28x interrupt cannot tear the 32-bit generation.
CPU1/CM now consume a complete seqlock snapshot from CPU2, and parameter
commands are published only when values change, reducing message-RAM traffic.

## Current protocol scope

The CM target initializes Ethernet/lwIP and implements complete TCP/UDP GMP
Data Link service. CPU1 initializes and hands off USB device mode and the
EtherCAT controller/ESC memory before CM starts. USB class/application service
and a full EtherCAT SSC application on CM are extension points; controller
initialization alone must not be reported as protocol interoperability.

The reference was compiled with CCS 12.8.1, C2000Ware 5.04.00.00, C28 compiler
22.6.1.LTS, and ARM compiler 20.2.7.LTS. A successful build is not a substitute
for target flash, physical-link, and DL facility tests.
