# LAUNCHXL-F280049C GMP CSP and Data Link Validation Project

This CSP reference project validates the 16-bit-addressed GMP Data Link backend on a TI LAUNCHXL-F280049C. It follows the standard GMP application structure: platform bindings are in `xplt`, application behavior is in `user`, and all required GMP headers and C sources are copied into `gmp_src_mgr`. The generated project therefore has no absolute dependency on the GMP repository.

## Implemented functions

- SCIA transport through the XDS110 Application/User UART on GPIO28 and GPIO29.
- An RX FIFO interrupt at eight characters plus continuous background draining of residual FIFO characters.
- A 1 kHz CPU Timer 0 interrupt that generates a 50 Hz sine/cosine pair.
- Two standard GMP scheduler tasks: a 500 ms LED heartbeat and a continuous non-blocking Data Link service.
- U16 Data Link discovery, ECHO, Tunable, Memory, and dedicated Scope services.
- Three physical Tunable parameters: signal frequency, gain, and DC offset.
- A sandboxed 128-byte scratch-memory resource with protocol byte-address translation for the C28x 16-bit address space.
- A DSA Trigger/Scope resource containing two channels and 400 float32 samples per channel at 1 kHz.

The Scope trigger position is expressed in permille of the 400-sample record. The timer ISR continuously stores samples in a circular history. When the selected rising, falling, or automatic trigger occurs, the requested pre-trigger portion is copied from that history and the remainder of the frame is recorded after the trigger. For example, a position of 250 places 100 pre-trigger samples before the trigger sample.

## Tool requirements

- Code Composer Studio 12.8
- TI C2000 Code Generation Tools 22.6.1 LTS
- C2000Ware 5.3 registered as a CCS product
- Python 3 with `pyserial` for the hardware smoke test

The CCS product setup mirrors `csp/c28x_syscfg/iris_280039c_board`: the project declares `C2000WARE:5.3.0.00` and uses `COM_TI_C2000WARE_INSTALL_DIR` and the standard product macros. A matching project-local DriverLib header set and EABI library are also included so the source tree remains portable.

## Build, flash, and test

Open this directory as an existing CCS project and build the `CPU1_LAUNCHXL_FLASH` configuration, or run:

```bat
build_ccs.bat
flash.bat
test_dl.bat
```

`build_ccs.bat` uses `C:\ti\ccs1281\ccs` by default. Set `CCS_ROOT` before running it if CCS is installed elsewhere.

The default UART selection is 921600 in `xplt/xplt.config.h`. To use the conservative speed, change `XPLT_DL_BAUD_RATE` to `115200UL`, rebuild, flash, and run:

```bat
test_dl.bat 115200
```

An explicit port can be supplied as the second argument, for example `test_dl.bat 921600 COM5`. When omitted, the test locates the XDS110 Application/User UART automatically.

At 921600, the F280049C uses a 50 MHz LSPCLK and BRR=6. The resulting physical rate is 892857 baud, an error of -3.12 percent relative to the nominal host setting. This configuration passed the complete hardware test and ten consecutive stress iterations on the connected LaunchPad. The default 115200 configuration also passed the same complete test.

The test checks target/backend discovery, a 256-byte ECHO, Tunable discovery and write/readback, Memory discovery and byte writes, Scope configuration and capture, all 3200 waveform bytes, RMS, quadrature, periodicity, DC offset, and trigger position.

## Regenerating local GMP sources

The generated `gmp_src_mgr/gmp_inc` and `gmp_src_mgr/gmp_src` directories in this project are ready to build without the repository. While developing inside a GMP checkout, regenerate them with:

```bat
set GMP_PRO_LOCATION=E:\path\to\gmp_pro
gmp_src_mgr\gmp_generate_all.bat
```

Regeneration intentionally uses the C28x CSP and selects the U16 Data Link backend through `GMP_PORT_DATA_SIZE_PER_BYTES == 2`.
