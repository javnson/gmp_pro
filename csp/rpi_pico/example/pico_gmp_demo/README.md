# GMP Raspberry Pi Pico example

This example connects a Raspberry Pi Pico or Pico 2 project to the GMP runtime.
It starts a 1 kHz Pico SDK repeating timer in `xplt.peripheral.c`. The timer
calls `MainISR()`. A 500 ms periodic task registered with the GMP function
scheduler toggles the board LED from the foreground loop. The controller
functions intentionally have empty bodies.

## Generated GMP sources

The `gmp_pico_src/gmp_src_mgr` directory selects `core/std`, the GMP function
scheduler, the base device interface, and `csp/rp_pico`. Run `gmp_config.bat`,
generate the source and header mirrors, and click **Generate CMake** if the
module selection changes. The manager writes
`gmp_pico_src/gmp_src_mgr/gmp_config.cmake`. The stable
`gmp_pico_src/gmp_config.cmake` wrapper includes that generated file and is the
only GMP integration file included by the Pico project.

The LED uses `PICO_DEFAULT_LED_PIN` from the selected Pico board definition and
is accessed through the GMP GPIO interface.

## Build

Install the Raspberry Pi Pico SDK and export both `PICO_SDK_PATH` and
`GMP_PRO_LOCATION`, then run:

```sh
cmake -S . -B build -G Ninja -DPICO_BOARD=pico
cmake --build build
```

Use `-DPICO_BOARD=pico2` for a Pico 2. The generated UF2 file is
`build/pico_gmp_demo.uf2`.

Source paths in the generated CMake file are relative to `gmp_src_mgr`. GMP
header paths are resolved from `GMP_PRO_LOCATION`, and the registered Pico SDK
libraries are linked by that file automatically.
