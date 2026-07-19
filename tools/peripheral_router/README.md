# GMP Peripheral Router

The peripheral router makes physical GPIO, UART, I2C, SPI and future CAN
controllers available to Windows or Linux programs. A host can connect several
boards simultaneously and select both a device and a channel.

The repository is split into four stable layers:

- `protocol`: versioned C and Python wire encoding with CRC and COBS framing;
- `firmware/pico`: a complete Pico SDK USB router project;
- `host`: a Python API plus a loopback-only multi-device service;
- `ui`: a Tkinter diagnostic application grouped by peripheral family.

Install and run the host service from `host`:

```text
python -m pip install -e .
python -m gmp_router.service
```

Run the debug UI from the repository checkout:

```text
python ui/main.py
```

CAN commands and asynchronous event identifiers are fixed in protocol version 1.
Pico firmware intentionally reports zero CAN channels. A later CAN-capable board
can implement the same operations without changing host applications.
