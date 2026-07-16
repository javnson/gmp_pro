# Raspberry Pi Pico GMP CSP

This chip support package connects RP2040 and RP2350 Pico SDK applications to
the GMP platform interfaces.

## Supported interfaces

- Monotonic millisecond system tick
- Per-core nested interrupt critical sections
- Optional hardware watchdog
- GPIO direction, read, and write
- Timeout-aware UART read and write
- 7-bit controller-mode I2C command, register, and memory transactions
- Timeout-aware 8-bit controller-mode SPI read, write, and full-duplex transfer

All source and header comments in this CSP use English Doxygen syntax.

## GPIO handles

GMP reserves `NULL` to mean an unassigned GPIO. A raw Pico GPIO number cannot
be used directly because GP0 would also have the value zero. Convert every pin
with `GMP_RPI_PICO_GPIO(pin)` before passing it to a GMP API:

```c
gpio_halt status_led = GMP_RPI_PICO_GPIO(25U);
gmp_hal_gpio_set_dir(status_led, GMP_HAL_GPIO_DIR_OUT);
gmp_hal_gpio_write(status_led, GMP_HAL_GPIO_HIGH);
```

## CMake integration

Select the `core/std` and `csp/rp_pico` modules in the GMP source manager,
generate the sources and headers, and then click **Generate CMake**. The source
manager writes `gmp_config.cmake` inside `gmp_src_mgr`.

Set the `GMP_PRO_LOCATION` environment variable to the GMP repository root.
After creating the Pico target, include the generated file:

```cmake
pico_sdk_init()
add_executable(your_target main.c)
include("${CMAKE_CURRENT_SOURCE_DIR}/gmp_src_mgr/gmp_config.cmake")
```

The generated source entries are relative to `gmp_config.cmake`. GMP include
directories are derived from `GMP_PRO_LOCATION`, and module-specific CMake
libraries are linked automatically. A project may expose a stable wrapper
outside `gmp_src_mgr`, as demonstrated by the Pico example. Set
`GMP_CMAKE_TARGET` before the include only when the CMake target name differs
from the project name.

## Watchdog

Define `GMP_RPI_PICO_WATCHDOG_TIMEOUT_MS` to a nonzero integer in
`xplt.config.h` to enable the watchdog during `gmp_csp_startup()`. The default
is zero, which leaves the watchdog disabled. `gmp_csp_loop()` reloads the
counter when the feature is enabled.

See `example/pico_gmp_demo` for a source-manager-based blink project using the
GMP function scheduler.
