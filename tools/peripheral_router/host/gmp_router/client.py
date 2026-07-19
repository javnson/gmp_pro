"""Typed peripheral API for GMP router devices."""

from __future__ import annotations

import struct

from .can import CanCapabilities, CanFrame, CanRxEvent, CanState, CanStateEvent, CanTxEvent
from .device import RouterDevice
from .protocol import CanOperation, Peripheral


class RouterClient:
    """Expose grouped GPIO, UART, I2C, SPI and CAN operations."""

    def __init__(self, device: RouterDevice, endpoint: int = 0):
        self.device = device
        self.endpoint = endpoint

    def hello(self) -> dict[str, object]:
        payload = self.device.request(Peripheral.SYSTEM, 1, endpoint=self.endpoint)
        version, gpio, uart, i2c, spi, can, uid_len = struct.unpack_from("<BBBBBBB", payload)
        uid = payload[7:7 + uid_len].decode("ascii", errors="replace")
        return {"protocol": version, "uid": uid, "gpio": gpio, "uart": uart,
                "i2c": i2c, "spi": spi, "can": can}

    def gpio_configure(self, pin: int, output: bool, pull: int = 0) -> None:
        self.device.request(Peripheral.GPIO, 1, endpoint=self.endpoint,
                            channel=pin, payload=struct.pack("<BB", output, pull))

    def gpio_write(self, pin: int, value: bool) -> None:
        self.device.request(Peripheral.GPIO, 2, endpoint=self.endpoint,
                            channel=pin, payload=bytes((bool(value),)))

    def gpio_read(self, pin: int) -> bool:
        return bool(self.device.request(Peripheral.GPIO, 3, endpoint=self.endpoint,
                                        channel=pin)[0])

    def uart_configure(self, channel: int, baudrate: int, tx_pin: int, rx_pin: int) -> None:
        self.device.request(Peripheral.UART, 1, endpoint=self.endpoint, channel=channel,
                            payload=struct.pack("<IHH", baudrate, tx_pin, rx_pin))

    def uart_write(self, channel: int, data: bytes) -> None:
        self.device.request(Peripheral.UART, 2, endpoint=self.endpoint,
                            channel=channel, payload=data)

    def uart_read(self, channel: int, length: int) -> bytes:
        return self.device.request(Peripheral.UART, 3, endpoint=self.endpoint,
                                   channel=channel, payload=struct.pack("<H", length))

    def i2c_configure(self, channel: int, frequency: int, sda_pin: int, scl_pin: int) -> None:
        self.device.request(Peripheral.I2C, 1, endpoint=self.endpoint, channel=channel,
                            payload=struct.pack("<IHH", frequency, sda_pin, scl_pin))

    def i2c_write(self, channel: int, address: int, data: bytes) -> None:
        self.device.request(Peripheral.I2C, 2, endpoint=self.endpoint, channel=channel,
                            payload=struct.pack("<H", address) + data)

    def i2c_read(self, channel: int, address: int, length: int) -> bytes:
        return self.device.request(Peripheral.I2C, 3, endpoint=self.endpoint, channel=channel,
                                   payload=struct.pack("<HH", address, length))

    def spi_configure(self, channel: int, frequency: int, sck: int, tx: int,
                      rx: int, mode: int = 0) -> None:
        self.device.request(Peripheral.SPI, 1, endpoint=self.endpoint, channel=channel,
                            payload=struct.pack("<IHHHB", frequency, sck, tx, rx, mode))

    def spi_transfer(self, channel: int, data: bytes) -> bytes:
        return self.device.request(Peripheral.SPI, 4, endpoint=self.endpoint,
                                   channel=channel, payload=data)

    def can_capabilities(self, channel: int) -> CanCapabilities:
        payload = self.device.request(Peripheral.CAN, CanOperation.GET_CAPABILITIES,
                                      endpoint=self.endpoint, channel=channel)
        return CanCapabilities.decode(payload)

    def can_configure(self, channel: int, nominal_bitrate: int, data_bitrate: int = 0,
                      nominal_sample_point: int = 875, data_sample_point: int = 800,
                      flags: int = 0, mode: int = 0) -> None:
        payload = struct.pack("<IIHHHH", nominal_bitrate, data_bitrate,
                              nominal_sample_point, data_sample_point, flags, mode)
        self.device.request(Peripheral.CAN, CanOperation.CONFIGURE, endpoint=self.endpoint,
                            channel=channel, payload=payload)

    def can_start(self, channel: int) -> None:
        self.device.request(Peripheral.CAN, CanOperation.START, endpoint=self.endpoint, channel=channel)

    def can_stop(self, channel: int) -> None:
        self.device.request(Peripheral.CAN, CanOperation.STOP, endpoint=self.endpoint, channel=channel)

    def can_transmit(self, channel: int, frame: CanFrame, token: int = 0) -> None:
        self.device.request(Peripheral.CAN, CanOperation.TRANSMIT, endpoint=self.endpoint,
                            channel=channel, payload=frame.encode(token))

    def can_set_filter(self, channel: int, slot: int, identifier: int, mask: int,
                       flags: int = 0, flags_mask: int = 0) -> None:
        payload = struct.pack("<HIIHH", slot, identifier, mask, flags, flags_mask)
        self.device.request(Peripheral.CAN, CanOperation.SET_FILTER, endpoint=self.endpoint,
                            channel=channel, payload=payload)

    def can_state(self, channel: int) -> CanState:
        payload = self.device.request(Peripheral.CAN, CanOperation.GET_STATE,
                                      endpoint=self.endpoint, channel=channel)
        return CanState.decode(payload)

    def can_recover(self, channel: int) -> None:
        self.device.request(Peripheral.CAN, CanOperation.RECOVER, endpoint=self.endpoint, channel=channel)

    def on_can_event(self, operation: CanOperation, handler) -> None:
        """Register a callback for RX, TX completion, or state events."""
        self.device.on_event(Peripheral.CAN, operation, handler)

    def on_can_receive(self, handler) -> None:
        self.on_can_event(CanOperation.RX_EVENT, lambda packet: handler(CanRxEvent.decode(packet.payload)))

    def on_can_tx_complete(self, handler) -> None:
        self.on_can_event(CanOperation.TX_EVENT, lambda packet: handler(CanTxEvent.decode(packet.payload)))

    def on_can_state_change(self, handler) -> None:
        self.on_can_event(CanOperation.STATE_EVENT, lambda packet: handler(CanStateEvent.decode(packet.payload)))
