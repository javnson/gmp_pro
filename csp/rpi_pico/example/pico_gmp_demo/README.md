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
module selection changes. The generated `gmp_pico_src/gmp_config.cmake` is the
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

Source paths in `gmp_config.cmake` are relative to that file. GMP header paths
are resolved from the `GMP_PRO_LOCATION` environment variable.
