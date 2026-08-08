"""DSA trigger/scope waveform viewer backed by Data Link Memory Perspective."""

from __future__ import annotations

import struct

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from core_datalink import HermesDatalinkQt


class TabDsaScope(QWidget):
    """Acquire and display a coherent two-channel DSA snapshot."""

    CMD_SCOPE_INFO = 0x03
    CMD_SCOPE_ARM = 0x04
    CMD_MEMORY_READ = 0x50
    STATE_WAITING = 0
    STATE_CAPTURING = 1
    STATE_READY = 2
    FORMAT_F32 = 1
    MAX_FLOATS_PER_READ = 60

    def __init__(self, hermes: HermesDatalinkQt):
        super().__init__()
        self.hermes = hermes
        self.metadata = None
        self.memory_buffer = bytearray()
        self.read_offset = 0
        self.capture_active = False
        self.poll_count = 0

        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(50)
        self.poll_timer.timeout.connect(self._request_scope_info)
        self.hermes.sig_bus_event.connect(self.on_bus_event)

        self._setup_ui()

    def _setup_ui(self) -> None:
        """Build the capture controls and waveform plot."""
        layout = QVBoxLayout(self)

        controls = QGroupBox("DSA Trigger + Scope")
        controls_layout = QHBoxLayout(controls)
        self.capture_button = QPushButton("Arm and Capture")
        self.capture_button.setMinimumHeight(38)
        self.capture_button.clicked.connect(self.start_capture)
        controls_layout.addWidget(self.capture_button)

        self.status_label = QLabel("Idle")
        controls_layout.addWidget(self.status_label)
        controls_layout.addStretch()

        self.detail_label = QLabel("Waiting for target metadata")
        controls_layout.addWidget(self.detail_label)
        layout.addWidget(controls)

        self.plot = pg.PlotWidget()
        self.plot.setBackground("#F8F9FA")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "Time", units="ms")
        self.plot.setLabel("left", "Amplitude")
        self.plot.addLegend()
        self.sine_curve = self.plot.plot(
            [], [], pen=pg.mkPen("#1565C0", width=2), name="Sine"
        )
        self.cosine_curve = self.plot.plot(
            [], [], pen=pg.mkPen("#D84315", width=2), name="Cosine"
        )
        layout.addWidget(self.plot, stretch=1)

    def _log(self, message: str) -> None:
        """Forward a DSA-specific message to the shared system log."""
        self.hermes.sig_log_msg.emit(f"[DSA Scope] {message}")

    def start_capture(self) -> None:
        """Arm the target trigger and begin polling for a completed snapshot."""
        if not self.hermes.running or self.capture_active:
            return
        self.capture_active = True
        self.metadata = None
        self.memory_buffer = bytearray()
        self.read_offset = 0
        self.poll_count = 0
        self.capture_button.setEnabled(False)
        self.status_label.setText("Arming")
        self.hermes.send_frame(0x01, self.CMD_SCOPE_ARM, b"", priority=0)

    def _request_scope_info(self) -> None:
        """Request current DSA state and immutable snapshot metadata."""
        if not self.capture_active:
            self.poll_timer.stop()
            return
        self.poll_count += 1
        if self.poll_count > 100:
            self._finish_with_error("Capture timed out")
            return
        self.hermes.send_frame(0x01, self.CMD_SCOPE_INFO, b"", priority=0)

    def _request_next_memory_chunk(self) -> None:
        """Read the completed DSA buffer through Memory Perspective."""
        total_bytes = self.metadata["buffer_bytes"]
        if self.read_offset >= total_bytes:
            self._render_snapshot()
            return

        remaining_floats = (total_bytes - self.read_offset) // 4
        item_count = min(self.MAX_FLOATS_PER_READ, remaining_floats)
        address = self.metadata["buffer_address"] + self.read_offset
        payload = struct.pack("<IBH", address, 4, item_count)
        self.hermes.send_frame(0x01, self.CMD_MEMORY_READ, payload, priority=0)

    def _handle_scope_info(self, payload: bytes) -> None:
        """Validate metadata and start download once the scope is ready."""
        if len(payload) != 20:
            self._finish_with_error("Invalid scope metadata response")
            return

        (
            version,
            state,
            sample_format,
            channels,
            depth,
            sample_rate,
            buffer_address,
            buffer_bytes,
            generation,
        ) = struct.unpack("<BBBBHIIHI", payload)

        if version != 1 or sample_format != self.FORMAT_F32 or channels != 2:
            self._finish_with_error("Unsupported DSA snapshot format")
            return
        if buffer_bytes != channels * depth * 4:
            self._finish_with_error("Inconsistent DSA buffer size")
            return

        state_names = {
            self.STATE_WAITING: "Waiting for rising zero crossing",
            self.STATE_CAPTURING: "Capturing",
            self.STATE_READY: "Ready",
        }
        self.status_label.setText(state_names.get(state, f"State {state}"))
        self.detail_label.setText(
            f"{sample_rate} Hz, {depth} samples/channel, generation {generation}"
        )

        if state == self.STATE_READY:
            self.poll_timer.stop()
            self.metadata = {
                "channels": channels,
                "depth": depth,
                "sample_rate": sample_rate,
                "buffer_address": buffer_address,
                "buffer_bytes": buffer_bytes,
                "generation": generation,
            }
            self.memory_buffer = bytearray(buffer_bytes)
            self.read_offset = 0
            self.status_label.setText("Downloading snapshot")
            self._request_next_memory_chunk()

    def _handle_memory_data(self, payload: bytes) -> None:
        """Append one Memory Perspective response to the local snapshot."""
        if not self.capture_active or self.metadata is None or not payload:
            return
        if payload[0] != 0:
            self._finish_with_error("Memory Perspective rejected the DSA buffer read")
            return

        data = payload[1:]
        end = self.read_offset + len(data)
        if end > len(self.memory_buffer):
            self._finish_with_error("DSA memory response exceeded the snapshot size")
            return
        self.memory_buffer[self.read_offset:end] = data
        self.read_offset = end
        self._request_next_memory_chunk()

    def _render_snapshot(self) -> None:
        """Decode the SoA float buffer and update both waveform curves."""
        depth = self.metadata["depth"]
        sample_rate = self.metadata["sample_rate"]
        values = np.frombuffer(self.memory_buffer, dtype="<f4")
        if values.size != depth * 2:
            self._finish_with_error("Downloaded DSA sample count is invalid")
            return

        time_ms = np.arange(depth, dtype=np.float64) * (1000.0 / sample_rate)
        self.sine_curve.setData(time_ms, values[:depth])
        self.cosine_curve.setData(time_ms, values[depth:])
        self.plot.enableAutoRange()
        self.status_label.setText("Capture complete")
        self.capture_active = False
        self.capture_button.setEnabled(True)
        self._log(
            f"Displayed generation {self.metadata['generation']} "
            f"({depth} samples per channel)"
        )

    def _finish_with_error(self, message: str) -> None:
        """Stop the acquisition state machine and report an error."""
        self.poll_timer.stop()
        self.capture_active = False
        self.capture_button.setEnabled(True)
        self.status_label.setText(message)
        self._log(message)

    def on_bus_event(self, event: dict) -> None:
        """Consume only valid Data Link responses used by this page."""
        if (
            event.get("type") != "DL"
            or event.get("dir") != "RX"
            or not event.get("dl_crc_ok")
            or not self.capture_active
        ):
            return

        command = event["dl_cmd"]
        payload = event["dl_payload"]
        if command == self.CMD_SCOPE_ARM:
            if payload != b"\x00":
                self._finish_with_error("Target rejected the DSA arm command")
                return
            self.status_label.setText("Armed")
            self.poll_timer.start()
            self._request_scope_info()
        elif command == self.CMD_SCOPE_INFO:
            self._handle_scope_info(payload)
        elif command == self.CMD_MEMORY_READ:
            self._handle_memory_data(payload)
