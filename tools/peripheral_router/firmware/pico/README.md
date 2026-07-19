# GMP Pico Peripheral Router Firmware

This project turns an RP2040 or RP2350 board into a USB-connected peripheral
router. GPIO, UART, I2C and SPI are implemented. The capability response reports
zero CAN channels because Pico devices have no native CAN controller; the shared
protocol already reserves the complete asynchronous CAN operation and event set
for later CAN-capable boards.

Set `PICO_SDK_PATH`, select a board with `-DPICO_BOARD=...`, and build with CMake:

```text
cmake -S . -B build -DPICO_BOARD=pico
cmake --build build
```

The firmware uses USB stdio as a binary COBS-delimited transport. Do not enable
other USB-stdio logging on the same channel.
