"""Configurable oscilloscope page for the dedicated Data Link Scope service."""

from __future__ import annotations

import struct
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QComboBox,
    QCheckBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from core_datalink import HermesDatalinkQt


@dataclass(frozen=True)
class ScopeResource:
    """One target-reported Data Link Scope resource."""

    resource_id: int
    sample_type: int
    layout: int
    channels: int
    depth: int
    sample_rate_hz: int
    byte_length: int
    name: str


class TabDsaScope(QWidget):
    """Discover, configure, acquire, and plot target scope resources."""

    OP_DISCOVER = 0
    OP_CONFIGURE = 1
    OP_ARM = 2
    OP_STATUS = 3
    OP_READ = 4
    STATE_WAITING = 0
    STATE_CAPTURING = 1
    STATE_READY = 2
    MAX_BYTES_PER_READ = 180
    DISCOVERY_HEADER = struct.Struct("<BBBBBBBHIIIB")
    READ_HEADER = struct.Struct("<BBBIH")
    CURVE_COLORS = ["#1565C0", "#D84315", "#2E7D32", "#6A1B9A", "#00838F", "#C62828"]
    DATA_TYPES = {
        0: ("Raw unsigned 8-bit", np.dtype("u1")),
        1: ("Unsigned 8-bit", np.dtype("u1")),
        2: ("Signed 8-bit", np.dtype("i1")),
        3: ("Unsigned 16-bit", np.dtype("<u2")),
        4: ("Signed 16-bit", np.dtype("<i2")),
        5: ("Unsigned 32-bit", np.dtype("<u4")),
        6: ("Signed 32-bit", np.dtype("<i4")),
        7: ("32-bit float", np.dtype("<f4")),
        8: ("64-bit float", np.dtype("<f8")),
    }
    LAYOUT_NAMES = {0: "Linear", 1: "Structure of Arrays", 2: "Interleaved"}

    def __init__(self, hermes: HermesDatalinkQt) -> None:
        super().__init__()
        self.hermes = hermes
        self.base_command = 0x60
        self.resources: list[ScopeResource] = []
        self.discovery_next_id = 0
        self.discovery_total = 0
        self.discovery_active = False
        self.metadata: dict | None = None
        self.memory_buffer = bytearray()
        self.read_offset = 0
        self.capture_active = False
        self.repeat_after_capture = False
        self.poll_count = 0
        self.curves: list[pg.PlotDataItem] = []
        self.afterglow_groups: list[list[pg.PlotDataItem]] = []

        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(50)
        self.poll_timer.timeout.connect(self._request_scope_status)
        self.hermes.sig_bus_event.connect(self.on_bus_event)
        self._setup_ui()

    def _setup_ui(self) -> None:
        """Build resource, trigger, acquisition, and plotting controls."""
        layout = QVBoxLayout(self)

        resource_group = QGroupBox("Data Link Scope Resource")
        resource_layout = QGridLayout(resource_group)
        self.command_edit = QLineEdit("0x60")
        self.command_edit.setMaximumWidth(70)
        self.resource_combo = QComboBox()
        self.resource_combo.currentIndexChanged.connect(self._apply_selected_resource)
        self.refresh_button = QPushButton("Discover Scopes")
        self.refresh_button.clicked.connect(self.discover_scopes)
        resource_layout.addWidget(QLabel("Base command:"), 0, 0)
        resource_layout.addWidget(self.command_edit, 0, 1)
        resource_layout.addWidget(QLabel("Scope:"), 0, 2)
        resource_layout.addWidget(self.resource_combo, 0, 3, 1, 2)
        resource_layout.addWidget(self.refresh_button, 0, 5)

        self.type_label = QLabel("-")
        self.layout_label = QLabel("-")
        self.channels_label = QLabel("-")
        self.depth_label = QLabel("-")
        self.rate_label = QLabel("-")
        resource_layout.addWidget(QLabel("Data type:"), 1, 0)
        resource_layout.addWidget(self.type_label, 1, 1)
        resource_layout.addWidget(QLabel("Layout:"), 1, 2)
        resource_layout.addWidget(self.layout_label, 1, 3)
        resource_layout.addWidget(QLabel("Channels:"), 2, 0)
        resource_layout.addWidget(self.channels_label, 2, 1)
        resource_layout.addWidget(QLabel("Samples/channel:"), 2, 2)
        resource_layout.addWidget(self.depth_label, 2, 3)
        resource_layout.addWidget(QLabel("Sample rate:"), 2, 4)
        resource_layout.addWidget(self.rate_label, 2, 5)
        layout.addWidget(resource_group)

        trigger_group = QGroupBox("Trigger Configuration")
        trigger_layout = QGridLayout(trigger_group)
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Continuous / immediate", 0)
        self.mode_combo.addItem("Rising edge", 1)
        self.mode_combo.addItem("Falling edge", 2)
        self.mode_combo.addItem("Rising edge with auto timeout", 3)
        self.mode_combo.addItem("Falling edge with auto timeout", 4)
        self.trigger_channel_spin = QSpinBox()
        self.trigger_channel_spin.setRange(0, 0)
        self.level_spin = QDoubleSpinBox()
        self.level_spin.setRange(-1.0e9, 1.0e9)
        self.level_spin.setDecimals(6)
        self.position_spin = QDoubleSpinBox()
        self.position_spin.setRange(0.0, 99.9)
        self.position_spin.setValue(50.0)
        self.position_spin.setSuffix(" %")
        self.timeout_spin = QSpinBox()
        self.timeout_spin.setRange(1, 600000)
        self.timeout_spin.setValue(1000)
        self.timeout_spin.setSuffix(" ms")
        self.continuous_checkbox = QCheckBox("Continuous display (re-arm automatically)")
        self.continuous_checkbox.toggled.connect(self._on_continuous_toggled)
        self.repeat_delay_spin = QSpinBox()
        self.repeat_delay_spin.setRange(0, 5000)
        self.repeat_delay_spin.setValue(50)
        self.repeat_delay_spin.setSuffix(" ms between captures")
        self.afterglow_checkbox = QCheckBox("Waveform persistence (afterglow)")
        self.afterglow_checkbox.toggled.connect(self._on_afterglow_toggled)
        self.afterglow_depth_spin = QSpinBox()
        self.afterglow_depth_spin.setRange(1, 20)
        self.afterglow_depth_spin.setValue(5)
        self.afterglow_depth_spin.setSuffix(" history frames")
        self.afterglow_opacity_spin = QDoubleSpinBox()
        self.afterglow_opacity_spin.setRange(0.05, 0.90)
        self.afterglow_opacity_spin.setSingleStep(0.05)
        self.afterglow_opacity_spin.setValue(0.30)
        self.afterglow_opacity_spin.setSuffix(" max opacity")
        self.clear_afterglow_button = QPushButton("Clear Persistence")
        self.clear_afterglow_button.clicked.connect(self.clear_afterglow)
        trigger_layout.addWidget(QLabel("Mode:"), 0, 0)
        trigger_layout.addWidget(self.mode_combo, 0, 1)
        trigger_layout.addWidget(QLabel("Source channel:"), 0, 2)
        trigger_layout.addWidget(self.trigger_channel_spin, 0, 3)
        trigger_layout.addWidget(QLabel("Level:"), 0, 4)
        trigger_layout.addWidget(self.level_spin, 0, 5)
        trigger_layout.addWidget(QLabel("Trigger position:"), 1, 0)
        trigger_layout.addWidget(self.position_spin, 1, 1)
        trigger_layout.addWidget(QLabel("Auto timeout:"), 1, 2)
        trigger_layout.addWidget(self.timeout_spin, 1, 3)
        trigger_layout.addWidget(self.continuous_checkbox, 1, 4)
        trigger_layout.addWidget(self.repeat_delay_spin, 1, 5)
        trigger_layout.addWidget(self.afterglow_checkbox, 2, 0, 1, 2)
        trigger_layout.addWidget(self.afterglow_depth_spin, 2, 2)
        trigger_layout.addWidget(self.afterglow_opacity_spin, 2, 3)
        trigger_layout.addWidget(self.clear_afterglow_button, 2, 4, 1, 2)

        button_row = QHBoxLayout()
        self.capture_button = QPushButton("Configure, Arm, and Capture")
        self.capture_button.setMinimumHeight(38)
        self.capture_button.clicked.connect(self.start_capture)
        self.read_button = QPushButton("Read Current Snapshot")
        self.read_button.setMinimumHeight(38)
        self.read_button.clicked.connect(self.read_current_snapshot)
        self.status_label = QLabel("Idle")
        button_row.addWidget(self.capture_button)
        button_row.addWidget(self.read_button)
        button_row.addWidget(self.status_label)
        button_row.addStretch()
        trigger_layout.addLayout(button_row, 3, 0, 1, 6)
        layout.addWidget(trigger_group)

        self.plot = pg.PlotWidget()
        self.plot.setBackground("#F8F9FA")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "Time", units="ms")
        self.plot.setLabel("left", "Amplitude")
        self.plot.addLegend()
        self.trigger_line = pg.InfiniteLine(angle=90, pen=pg.mkPen("#7B1FA2", width=1))
        self.plot.addItem(self.trigger_line)
        layout.addWidget(self.plot, stretch=1)

    def _log(self, message: str) -> None:
        """Forward a scope-specific message to the shared system log."""
        self.hermes.sig_log_msg.emit(f"[Data Link Scope] {message}")

    def _sync_command(self) -> bool:
        """Parse and validate the editable Scope command identifier."""
        try:
            self.base_command = int(self.command_edit.text(), 0)
        except ValueError:
            self._log("The Scope command must be an integer such as 0x60.")
            return False
        if not 0 <= self.base_command <= 0xFF:
            self._log("The Scope command must be between 0x00 and 0xFF.")
            return False
        return True

    def discover_scopes(self) -> None:
        """Start indexed discovery for the dedicated Scope service."""
        if not self.hermes.running or not self._sync_command():
            return
        self.resources = []
        self.discovery_next_id = 0
        self.discovery_total = 0
        self.discovery_active = True
        self.refresh_button.setEnabled(False)
        self.status_label.setText("Discovering scopes")
        self._send_discovery_query()

    def _send_discovery_query(self) -> None:
        """Request one indexed scope descriptor."""
        self.hermes.send_frame(
            0x01,
            self.base_command,
            bytes((self.OP_DISCOVER, self.discovery_next_id)),
            priority=0,
        )

    def _selected_resource(self) -> ScopeResource | None:
        """Return the resource currently selected by the user."""
        index = self.resource_combo.currentIndex()
        return self.resources[index] if 0 <= index < len(self.resources) else None

    def _apply_selected_resource(self) -> None:
        """Display immutable target-reported resource metadata."""
        resource = self._selected_resource()
        if resource is None:
            return
        self.type_label.setText(self.DATA_TYPES.get(resource.sample_type, ("Unknown", None))[0])
        self.layout_label.setText(self.LAYOUT_NAMES.get(resource.layout, "Unknown"))
        self.channels_label.setText(str(resource.channels))
        self.depth_label.setText(str(resource.depth))
        self.rate_label.setText(f"{resource.sample_rate_hz} Hz")
        self.trigger_channel_spin.setMaximum(max(0, resource.channels - 1))

    def start_capture(self) -> None:
        """Configure and arm the selected target scope resource."""
        resource = self._selected_resource()
        if not self.hermes.running or self.capture_active or resource is None:
            self._log("Discover and select a Scope resource first.")
            return
        self.capture_active = True
        self.repeat_after_capture = self.continuous_checkbox.isChecked()
        self.poll_count = 0
        self._set_acquisition_controls(False)
        self.status_label.setText("Configuring")
        payload = struct.pack(
            "<BBBBHfI",
            self.OP_CONFIGURE,
            resource.resource_id,
            self.mode_combo.currentData(),
            self.trigger_channel_spin.value(),
            int(round(self.position_spin.value() * 10.0)),
            self.level_spin.value(),
            self.timeout_spin.value(),
        )
        self.hermes.send_frame(0x01, self.base_command, payload, priority=0)

    def read_current_snapshot(self) -> None:
        """Read the selected scope buffer without changing target capture state."""
        resource = self._selected_resource()
        if not self.hermes.running or self.capture_active or resource is None:
            return
        self.capture_active = True
        self.repeat_after_capture = False
        self._set_acquisition_controls(False)
        self._prepare_download(resource, generation=None)

    def _request_scope_status(self) -> None:
        """Poll the target capture state until a snapshot is ready."""
        resource = self._selected_resource()
        if not self.capture_active or resource is None:
            self.poll_timer.stop()
            return
        self.poll_count += 1
        if self.poll_count > 100:
            self._finish_with_error("Capture timed out.")
            return
        self.hermes.send_frame(
            0x01,
            self.base_command,
            bytes((self.OP_STATUS, resource.resource_id)),
            priority=0,
        )

    def _prepare_download(self, resource: ScopeResource, generation: int | None) -> None:
        """Allocate a local snapshot and begin Scope-native chunked reads."""
        dtype_entry = self.DATA_TYPES.get(resource.sample_type)
        if dtype_entry is None:
            self._finish_with_error("The selected scope uses an unsupported sample type.")
            return
        expected_bytes = resource.channels * resource.depth * dtype_entry[1].itemsize
        if expected_bytes > resource.byte_length:
            self._finish_with_error("Scope metadata exceeds the registered buffer capacity.")
            return
        self.metadata = {"resource": resource, "generation": generation}
        self.memory_buffer = bytearray(expected_bytes)
        self.read_offset = 0
        self.status_label.setText("Downloading snapshot")
        self._request_next_chunk()

    def _request_next_chunk(self) -> None:
        """Read the next byte chunk through the dedicated Scope service."""
        resource = self.metadata["resource"]
        if self.read_offset >= len(self.memory_buffer):
            self._render_snapshot()
            return
        length = min(self.MAX_BYTES_PER_READ, len(self.memory_buffer) - self.read_offset)
        payload = struct.pack(
            "<BBIH", self.OP_READ, resource.resource_id, self.read_offset, length
        )
        self.hermes.send_frame(0x01, self.base_command, payload, priority=0)

    def _handle_discovery(self, payload: bytes) -> None:
        """Parse one descriptor and continue the indexed discovery sequence."""
        if len(payload) < 5:
            self._finish_discovery("Scope discovery response is too short.")
            return
        operation, status, version, total, resource_id = payload[:5]
        if operation != self.OP_DISCOVER or version != 1 or status != 0:
            self._finish_discovery("Target rejected Scope discovery.")
            return
        if len(payload) < self.DISCOVERY_HEADER.size:
            self._finish_discovery("Scope descriptor is truncated.")
            return
        fields = self.DISCOVERY_HEADER.unpack_from(payload)
        name_end = self.DISCOVERY_HEADER.size + fields[11]
        if name_end > len(payload) or resource_id != self.discovery_next_id:
            self._finish_discovery("Scope descriptor sequence is invalid.")
            return
        name = payload[self.DISCOVERY_HEADER.size:name_end].decode("utf-8", errors="replace")
        self.resources.append(
            ScopeResource(
                resource_id=fields[4],
                sample_type=fields[5],
                layout=fields[6],
                channels=fields[7],
                depth=fields[8],
                sample_rate_hz=fields[9],
                byte_length=fields[10],
                name=name or f"Scope {fields[4]}",
            )
        )
        self.discovery_total = total
        self.discovery_next_id += 1
        if self.discovery_next_id < self.discovery_total:
            QTimer.singleShot(0, self._send_discovery_query)
        else:
            self.discovery_active = False
            self.refresh_button.setEnabled(True)
            self.resource_combo.clear()
            for resource in self.resources:
                self.resource_combo.addItem(resource.name, resource.resource_id)
            if self.resources:
                self.resource_combo.setCurrentIndex(0)
                self._apply_selected_resource()
            self.status_label.setText(f"Discovered {len(self.resources)} scope(s)")
            self._log(f"Discovered {len(self.resources)} Scope resources.")

    def _finish_discovery(self, error: str) -> None:
        """Stop a failed discovery sequence and report its reason."""
        self.discovery_active = False
        self.refresh_button.setEnabled(True)
        self.status_label.setText(error)
        self._log(error)

    def _handle_status(self, payload: bytes) -> None:
        """Process a capture-state response and start reading when ready."""
        if len(payload) != 8:
            self._finish_with_error("Invalid Scope status response.")
            return
        operation, status, resource_id, state, generation = struct.unpack("<BBBBI", payload)
        resource = self._selected_resource()
        if operation != self.OP_STATUS or status != 0 or resource is None or resource_id != resource.resource_id:
            self._finish_with_error("Target rejected the Scope status query.")
            return
        state_names = {
            self.STATE_WAITING: "Waiting for trigger",
            self.STATE_CAPTURING: "Capturing",
            self.STATE_READY: "Ready",
        }
        self.status_label.setText(state_names.get(state, f"State {state}"))
        if state == self.STATE_READY:
            self.poll_timer.stop()
            self._prepare_download(resource, generation)

    def _handle_read(self, payload: bytes) -> None:
        """Append one valid Scope-native buffer chunk."""
        if len(payload) < self.READ_HEADER.size:
            self._finish_with_error("Scope read response is truncated.")
            return
        operation, status, resource_id, offset, length = self.READ_HEADER.unpack_from(payload)
        resource = self.metadata["resource"] if self.metadata else None
        data = payload[self.READ_HEADER.size:]
        if (
            operation != self.OP_READ
            or status != 0
            or resource is None
            or resource_id != resource.resource_id
            or offset != self.read_offset
            or length != len(data)
            or offset + length > len(self.memory_buffer)
        ):
            self._finish_with_error("Target returned an invalid Scope data chunk.")
            return
        self.memory_buffer[offset:offset + length] = data
        self.read_offset += length
        self._request_next_chunk()

    def _ensure_curves(self, channels: int) -> None:
        """Create exactly one plot curve for each decoded channel."""
        if len(self.curves) == channels:
            return
        self.clear_afterglow()
        for curve in self.curves:
            self.plot.removeItem(curve)
        self.curves = [
            self.plot.plot(
                [], [],
                pen=pg.mkPen(self.CURVE_COLORS[index % len(self.CURVE_COLORS)], width=2),
                name=f"Channel {index}",
            )
            for index in range(channels)
        ]

    def _capture_afterglow(self) -> None:
        """Preserve the current curves as one age-faded persistence frame."""
        if not self.afterglow_checkbox.isChecked() or not self.curves:
            return
        group = []
        for index, curve in enumerate(self.curves):
            x_values, y_values = curve.getData()
            if x_values is None or y_values is None or len(x_values) == 0:
                return
            item = self.plot.plot(x_values.copy(), y_values.copy())
            group.append(item)
        self.afterglow_groups.append(group)
        maximum_groups = self.afterglow_depth_spin.value()
        while len(self.afterglow_groups) > maximum_groups:
            oldest = self.afterglow_groups.pop(0)
            for item in oldest:
                self.plot.removeItem(item)
        group_count = len(self.afterglow_groups)
        maximum_alpha = int(round(self.afterglow_opacity_spin.value() * 255.0))
        for age_index, history_group in enumerate(self.afterglow_groups):
            age_ratio = (age_index + 1) / group_count
            alpha = max(8, int(round(maximum_alpha * age_ratio)))
            for channel_index, item in enumerate(history_group):
                color = pg.mkColor(self.CURVE_COLORS[channel_index % len(self.CURVE_COLORS)])
                color.setAlpha(alpha)
                item.setPen(pg.mkPen(color, width=1))

    def clear_afterglow(self) -> None:
        """Remove every retained persistence frame from the plot."""
        for group in self.afterglow_groups:
            for item in group:
                self.plot.removeItem(item)
        self.afterglow_groups = []

    def _on_afterglow_toggled(self, enabled: bool) -> None:
        """Clear retained frames when waveform persistence is disabled."""
        if not enabled:
            self.clear_afterglow()

    def _render_snapshot(self) -> None:
        """Decode the reported layout and update all waveform curves."""
        resource = self.metadata["resource"]
        dtype = self.DATA_TYPES[resource.sample_type][1]
        values = np.frombuffer(self.memory_buffer, dtype=dtype)
        if values.size != resource.channels * resource.depth:
            self._finish_with_error("Downloaded Scope sample count is inconsistent.")
            return
        if resource.layout == 2:
            channel_data = values.reshape(resource.depth, resource.channels).T
        else:
            channel_data = values.reshape(resource.channels, resource.depth)
        time_ms = np.arange(resource.depth, dtype=np.float64) * (1000.0 / resource.sample_rate_hz)
        self._ensure_curves(resource.channels)
        self._capture_afterglow()
        for index, curve in enumerate(self.curves):
            curve.setData(time_ms, channel_data[index])
        trigger_sample = int(round(resource.depth * self.position_spin.value() / 100.0))
        self.trigger_line.setValue(trigger_sample * 1000.0 / resource.sample_rate_hz)
        self.plot.enableAutoRange()
        generation = self.metadata["generation"]
        suffix = "" if generation is None else f", generation {generation}"
        should_repeat = self.repeat_after_capture and self.continuous_checkbox.isChecked()
        self.status_label.setText("Capture complete; re-arming" if should_repeat else "Capture complete")
        self._finish_capture()
        self._log(f"Displayed {resource.depth} samples on {resource.channels} channels{suffix}.")
        if should_repeat:
            QTimer.singleShot(self.repeat_delay_spin.value(), self._repeat_capture_if_enabled)

    def _repeat_capture_if_enabled(self) -> None:
        """Start the next snapshot only while continuous display remains selected."""
        if self.continuous_checkbox.isChecked() and not self.capture_active:
            self.start_capture()

    def _on_continuous_toggled(self, enabled: bool) -> None:
        """Apply a continuous-display change immediately to the active sequence."""
        if self.capture_active:
            self.repeat_after_capture = enabled
        elif not enabled and self.status_label.text().endswith("re-arming"):
            self.status_label.setText("Capture complete")

    def _set_acquisition_controls(self, enabled: bool) -> None:
        """Enable or disable controls that can start another acquisition."""
        self.capture_button.setEnabled(enabled)
        self.read_button.setEnabled(enabled)
        self.refresh_button.setEnabled(enabled and not self.discovery_active)

    def _finish_capture(self) -> None:
        """Release acquisition controls after a completed operation."""
        self.poll_timer.stop()
        self.capture_active = False
        self._set_acquisition_controls(True)

    def _finish_with_error(self, message: str) -> None:
        """Stop acquisition and report a user-visible error."""
        self._finish_capture()
        self.status_label.setText(message)
        self._log(message)

    def on_bus_event(self, event: dict) -> None:
        """Consume valid responses belonging to this Scope service command."""
        if (
            event.get("type") != "DL"
            or event.get("dir") != "RX"
            or not event.get("dl_crc_ok")
            or event.get("dl_cmd") != self.base_command
        ):
            return
        payload = event.get("dl_payload", b"")
        if not payload:
            return
        operation = payload[0]
        if operation == self.OP_DISCOVER and self.discovery_active:
            self._handle_discovery(payload)
        elif operation == self.OP_CONFIGURE and self.capture_active:
            resource = self._selected_resource()
            if len(payload) != 3 or payload[1] != 0 or resource is None or payload[2] != resource.resource_id:
                self._finish_with_error("Target rejected the Scope configuration.")
                return
            self.status_label.setText("Arming")
            self.hermes.send_frame(
                0x01,
                self.base_command,
                bytes((self.OP_ARM, resource.resource_id)),
                priority=0,
            )
        elif operation == self.OP_ARM and self.capture_active:
            resource = self._selected_resource()
            if len(payload) != 3 or payload[1] != 0 or resource is None or payload[2] != resource.resource_id:
                self._finish_with_error("Target rejected the Scope arm request.")
                return
            self.status_label.setText("Armed")
            self.poll_timer.start()
            self._request_scope_status()
        elif operation == self.OP_STATUS and self.capture_active:
            self._handle_status(payload)
        elif operation == self.OP_READ and self.capture_active:
            self._handle_read(payload)
