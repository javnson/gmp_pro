"""Tkinter diagnostic console for GMP peripheral router boards."""

from __future__ import annotations

import pathlib
import sys
import tkinter as tk
from tkinter import messagebox, ttk

HOST = pathlib.Path(__file__).resolve().parents[1] / "host"
sys.path.insert(0, str(HOST))

from gmp_router import RouterClient, RouterDevice, SerialTransport, serial_ports  # noqa: E402


class RouterWindow(tk.Tk):
    """Small grouped UI for discovery and direct peripheral experiments."""

    def __init__(self) -> None:
        super().__init__()
        self.title("GMP Peripheral Router")
        self.geometry("760x520")
        self.device: RouterDevice | None = None
        self.client: RouterClient | None = None
        self.port = tk.StringVar()
        self.pin = tk.IntVar(value=25)
        self.value = tk.BooleanVar()
        self.channel = tk.IntVar(value=0)
        self.frequency = tk.IntVar(value=100000)
        self.tx_pin = tk.IntVar(value=0)
        self.rx_pin = tk.IntVar(value=1)
        self.aux_pin = tk.IntVar(value=2)
        self.address = tk.IntVar(value=0x50)
        self.length = tk.IntVar(value=1)
        self.hex_data = tk.StringVar(value="00")
        self._build()
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _build(self) -> None:
        bar = ttk.Frame(self, padding=8)
        bar.pack(fill="x")
        self.ports = ttk.Combobox(bar, textvariable=self.port, width=20)
        self.ports.pack(side="left")
        ttk.Button(bar, text="Refresh", command=self._refresh).pack(side="left", padx=4)
        ttk.Button(bar, text="Connect", command=self._connect).pack(side="left")
        self.status = ttk.Label(bar, text="Disconnected")
        self.status.pack(side="left", padx=12)
        tabs = ttk.Notebook(self)
        tabs.pack(fill="both", expand=True, padx=8, pady=4)
        device_tab = ttk.Frame(tabs, padding=12)
        gpio_tab = ttk.Frame(tabs, padding=12)
        tabs.add(device_tab, text="Device")
        tabs.add(gpio_tab, text="GPIO")
        uart_tab = ttk.Frame(tabs, padding=12)
        i2c_tab = ttk.Frame(tabs, padding=12)
        spi_tab = ttk.Frame(tabs, padding=12)
        can_tab = ttk.Frame(tabs, padding=12)
        tabs.add(uart_tab, text="UART")
        tabs.add(i2c_tab, text="I2C")
        tabs.add(spi_tab, text="SPI")
        tabs.add(can_tab, text="CAN")
        self.info = tk.Text(device_tab, height=12, width=72, state="disabled")
        self.info.pack(fill="both", expand=True)
        ttk.Label(gpio_tab, text="GPIO pin").grid(row=0, column=0, sticky="w")
        ttk.Entry(gpio_tab, textvariable=self.pin, width=8).grid(row=0, column=1, sticky="w")
        ttk.Checkbutton(gpio_tab, text="Level", variable=self.value).grid(row=1, column=0, sticky="w")
        ttk.Button(gpio_tab, text="Configure output", command=self._gpio_configure).grid(row=2, column=0, pady=8)
        ttk.Button(gpio_tab, text="Write", command=self._gpio_write).grid(row=2, column=1)
        ttk.Button(gpio_tab, text="Read", command=self._gpio_read).grid(row=2, column=2)
        self._common_fields(uart_tab)
        ttk.Button(uart_tab, text="Configure", command=self._uart_configure).grid(row=4, column=0, pady=8)
        ttk.Button(uart_tab, text="Write hex", command=self._uart_write).grid(row=4, column=1)
        ttk.Button(uart_tab, text="Read", command=self._uart_read).grid(row=4, column=2)
        self._common_fields(i2c_tab)
        ttk.Label(i2c_tab, text="Address").grid(row=4, column=0, sticky="w")
        ttk.Entry(i2c_tab, textvariable=self.address, width=12).grid(row=4, column=1, sticky="w")
        ttk.Button(i2c_tab, text="Configure", command=self._i2c_configure).grid(row=5, column=0, pady=8)
        ttk.Button(i2c_tab, text="Write hex", command=self._i2c_write).grid(row=5, column=1)
        ttk.Button(i2c_tab, text="Read", command=self._i2c_read).grid(row=5, column=2)
        self._common_fields(spi_tab)
        ttk.Button(spi_tab, text="Configure", command=self._spi_configure).grid(row=4, column=0, pady=8)
        ttk.Button(spi_tab, text="Transfer hex", command=self._spi_transfer).grid(row=4, column=1)
        ttk.Label(can_tab, text="Channel").grid(row=0, column=0, sticky="w")
        ttk.Entry(can_tab, textvariable=self.channel, width=10).grid(row=0, column=1, sticky="w")
        ttk.Button(can_tab, text="Query capabilities", command=self._can_capabilities).grid(row=1, column=0, pady=8)
        ttk.Label(can_tab, text="Pico reports zero CAN channels; this panel also works with future router boards.").grid(row=2, column=0, columnspan=3, sticky="w")
        self._refresh()

    def _common_fields(self, tab: ttk.Frame) -> None:
        labels = (("Channel", self.channel), ("Frequency/baud", self.frequency),
                  ("TX/SDA/SCK pin", self.tx_pin), ("RX/SCL/TX pin", self.rx_pin))
        for row, (label, variable) in enumerate(labels):
            ttk.Label(tab, text=label).grid(row=row, column=0, sticky="w")
            ttk.Entry(tab, textvariable=variable, width=14).grid(row=row, column=1, sticky="w")
        ttk.Label(tab, text="Hex data").grid(row=0, column=2, padx=(24, 4))
        ttk.Entry(tab, textvariable=self.hex_data, width=28).grid(row=0, column=3)
        ttk.Label(tab, text="Read length").grid(row=1, column=2, padx=(24, 4))
        ttk.Entry(tab, textvariable=self.length, width=10).grid(row=1, column=3, sticky="w")

    def _refresh(self) -> None:
        values = serial_ports()
        self.ports["values"] = values
        if values and not self.port.get():
            self.port.set(values[0])

    def _connect(self) -> None:
        try:
            if self.device:
                self.device.close()
            self.device = RouterDevice(SerialTransport(self.port.get()))
            self.client = RouterClient(self.device)
            metadata = self.client.hello()
            self.status.config(text=f"Connected: {metadata['uid']}")
            self.info.config(state="normal")
            self.info.delete("1.0", "end")
            self.info.insert("end", "\n".join(f"{key}: {value}" for key, value in metadata.items()))
            self.info.config(state="disabled")
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))

    def _require(self) -> RouterClient:
        if self.client is None:
            raise RuntimeError("Connect a router first")
        return self.client

    def _gpio_configure(self) -> None:
        try: self._require().gpio_configure(self.pin.get(), True)
        except Exception as exc: messagebox.showerror("GPIO", str(exc))

    def _gpio_write(self) -> None:
        try: self._require().gpio_write(self.pin.get(), self.value.get())
        except Exception as exc: messagebox.showerror("GPIO", str(exc))

    def _gpio_read(self) -> None:
        try: self.value.set(self._require().gpio_read(self.pin.get()))
        except Exception as exc: messagebox.showerror("GPIO", str(exc))

    def _uart_configure(self) -> None:
        try: self._require().uart_configure(self.channel.get(), self.frequency.get(), self.tx_pin.get(), self.rx_pin.get())
        except Exception as exc: messagebox.showerror("UART", str(exc))

    def _uart_write(self) -> None:
        try: self._require().uart_write(self.channel.get(), bytes.fromhex(self.hex_data.get()))
        except Exception as exc: messagebox.showerror("UART", str(exc))

    def _uart_read(self) -> None:
        try: self.hex_data.set(self._require().uart_read(self.channel.get(), self.length.get()).hex(" "))
        except Exception as exc: messagebox.showerror("UART", str(exc))

    def _i2c_configure(self) -> None:
        try: self._require().i2c_configure(self.channel.get(), self.frequency.get(), self.tx_pin.get(), self.rx_pin.get())
        except Exception as exc: messagebox.showerror("I2C", str(exc))

    def _i2c_write(self) -> None:
        try: self._require().i2c_write(self.channel.get(), self.address.get(), bytes.fromhex(self.hex_data.get()))
        except Exception as exc: messagebox.showerror("I2C", str(exc))

    def _i2c_read(self) -> None:
        try: self.hex_data.set(self._require().i2c_read(self.channel.get(), self.address.get(), self.length.get()).hex(" "))
        except Exception as exc: messagebox.showerror("I2C", str(exc))

    def _spi_configure(self) -> None:
        try: self._require().spi_configure(self.channel.get(), self.frequency.get(), self.tx_pin.get(), self.rx_pin.get(), self.aux_pin.get())
        except Exception as exc: messagebox.showerror("SPI", str(exc))

    def _spi_transfer(self) -> None:
        try: self.hex_data.set(self._require().spi_transfer(self.channel.get(), bytes.fromhex(self.hex_data.get())).hex(" "))
        except Exception as exc: messagebox.showerror("SPI", str(exc))

    def _can_capabilities(self) -> None:
        try: messagebox.showinfo("CAN capabilities", str(self._require().can_capabilities(self.channel.get())))
        except Exception as exc: messagebox.showerror("CAN", str(exc))

    def _close(self) -> None:
        if self.device:
            self.device.close()
        self.destroy()


if __name__ == "__main__":
    RouterWindow().mainloop()
