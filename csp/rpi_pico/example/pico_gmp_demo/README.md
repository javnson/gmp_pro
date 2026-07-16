# GMP Raspberry Pi Pico example

This example connects a Raspberry Pi Pico or Pico 2 project to the GMP runtime.
It demonstrates the GMP lifecycle, GPIO access, timeout-aware UART access, and
initialization of I2C0 and SPI0 for GMP device drivers.

## Pin assignment

| Peripheral | Pins |
| --- | --- |
| UART0 | TX GP0, RX GP1, 115200 baud |
| I2C0 | SDA GP4, SCL GP5, 400 kHz |
| SPI0 | RX GP16, CS GP17, SCK GP18, TX GP19, 1 MHz |
| LED | `PICO_DEFAULT_LED_PIN` from the selected board definition |

The UART prints a startup message and echoes received bytes. The LED toggles
every 500 ms through `gmp_hal_gpio_write()`.

Pico GPIO numbers must be converted with `GMP_RPI_PICO_GPIO(pin)` when passed
to a GMP GPIO API. The encoding keeps `NULL` available as the standard GMP
sentinel for an unassigned GPIO, while still allowing GP0 to be represented.

## Build

Install the Raspberry Pi Pico SDK and export `PICO_SDK_PATH`, then run:

```sh
cmake -S . -B build -G Ninja -DPICO_BOARD=pico
cmake --build build
```

Use `-DPICO_BOARD=pico2` for a Pico 2. The generated UF2 file is
`build/pico_gmp_demo.uf2`.

The CMake project derives the GMP repository root from its directory, so it
does not require a separate `GMP_PRO_LOCATION` setting.
