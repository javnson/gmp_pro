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

Initialize the Pico SDK first, then add this directory and link the CSP target:

```cmake
pico_sdk_init()
set(GMP_XPLT_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
add_subdirectory(path/to/gmp_pro/csp/rpi_pico gmp_csp_rpi_pico)
target_link_libraries(your_target PRIVATE gmp_csp_rpi_pico)
```

The application must provide `xplt.config.h` and the standard GMP lifecycle
functions required by its selected configuration.

## Watchdog

Define `GMP_RPI_PICO_WATCHDOG_TIMEOUT_MS` to a nonzero integer in
`xplt.config.h` to enable the watchdog during `gmp_csp_startup()`. The default
is zero, which leaves the watchdog disabled. `gmp_csp_loop()` reloads the
counter when the feature is enabled.

See `example/pico_gmp_demo` for a complete Pico SDK project.
