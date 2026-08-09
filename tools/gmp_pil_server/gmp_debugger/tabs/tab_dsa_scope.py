"""Configurable oscilloscope page for the dedicated Data Link Scope service."""

from __future__ import annotations

import csv
import struct
import time
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QComboBox,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
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
    protocol_version: int
    sample_type: int
    layout: int
    channels: int
    depth: int
    sample_rate_hz: int
    byte_length: int
    name: str


@dataclass(frozen=True)
class ScopeFrame:
    """One decoded Scope frame retained for plotting or CSV export."""

    time_ms: np.ndarray
    channel_data: np.ndarray
    generation: int | None


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
    PHASE_IDLE = "idle"
    PHASE_CONFIGURING = "configuring"
    PHASE_ARMING = "arming"
    PHASE_POLLING = "polling"
    PHASE_DOWNLOADING = "downloading"
    AUTO_TIMEOUT_MS = 1000
    CONTINUOUS_REARM_DELAY_MS = 50
    REQUEST_TIMEOUT_MS = 500
    MAX_REQUEST_RETRIES = 2
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
        self.capture_phase = self.PHASE_IDLE
        self.capture_mode = 0
        self.repeat_after_capture = False
        self.status_request_pending = False
        self.status_request_started = 0.0
        self.queued_configuration: bytes | None = None
        self.pending_request: bytes | None = None
        self.pending_request_retries = 0
        self.poll_count = 0
        self.curves: list[pg.PlotDataItem] = []
        self.afterglow_groups: list[list[pg.PlotDataItem]] = []
        self.persistence_frames: list[ScopeFrame] = []
        self.current_frame: ScopeFrame | None = None
        self.active_sample_dividers: dict[int, int] = {}
        self.pending_sample_divider = 0

        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(50)
        self.poll_timer.timeout.connect(self._request_scope_status)
        self.repeat_timer = QTimer(self)
        self.repeat_timer.setSingleShot(True)
        self.repeat_timer.setInterval(self.CONTINUOUS_REARM_DELAY_MS)
        self.repeat_timer.timeout.connect(self._repeat_capture_if_enabled)
        self.request_timer = QTimer(self)
        self.request_timer.setSingleShot(True)
        self.request_timer.setInterval(self.REQUEST_TIMEOUT_MS)
        self.request_timer.timeout.connect(self._retry_pending_request)
        self.hermes.sig_bus_event.connect(self.on_bus_event)
        self.hermes.sig_conn_state.connect(self._on_connection_state)
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
        resource_layout.addWidget(QLabel("Scope:"), 0, 0)
        resource_layout.addWidget(self.resource_combo, 0, 1, 1, 4)
        resource_layout.addWidget(self.refresh_button, 0, 5)

        self.type_label = QLabel("-")
        self.layout_label = QLabel("-")
        self.channels_label = QLabel("-")
        self.depth_label = QLabel("-")
        self.rate_label = QLabel("-")
        resource_layout.addWidget(QLabel("Base command:"), 1, 0)
        resource_layout.addWidget(self.command_edit, 1, 1)
        resource_layout.addWidget(QLabel("Data type:"), 1, 2)
        resource_layout.addWidget(self.type_label, 1, 3)
        resource_layout.addWidget(QLabel("Layout:"), 1, 4)
        resource_layout.addWidget(self.layout_label, 1, 5)
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
        self.mode_combo.addItem("Rising edge (1 s auto)", 3)
        self.mode_combo.addItem("Falling edge (1 s auto)", 4)
        control_height = self.mode_combo.sizeHint().height()
        self.refresh_button.setFixedHeight(control_height)
        self.trigger_channel_spin = QSpinBox()
        self.trigger_channel_spin.setRange(0, 0)
        self.level_spin = QDoubleSpinBox()
        self.level_spin.setRange(-1.0e9, 1.0e9)
        self.level_spin.setDecimals(6)
        self.position_spin = QDoubleSpinBox()
        self.position_spin.setRange(0.0, 99.9)
        self.position_spin.setValue(50.0)
        self.position_spin.setSuffix(" %")
        position_tooltip = (
            "The target continuously stores pre-trigger samples in a circular history "
            "buffer. When the selected edge occurs, it preserves the requested history "
            "and records the remaining samples after the trigger. For example, 50% "
            "places the trigger event at the center of the displayed record."
        )
        position_label = QLabel("Trigger position")
        position_label.setToolTip(position_tooltip)
        self.position_spin.setToolTip(position_tooltip)
        trigger_controls = (
            (QLabel("Mode"), self.mode_combo),
            (QLabel("Source channel"), self.trigger_channel_spin),
            (QLabel("Level"), self.level_spin),
            (position_label, self.position_spin),
        )
        for column, (label, control) in enumerate(trigger_controls):
            trigger_layout.addWidget(label, 0, column)
            trigger_layout.addWidget(control, 1, column)
            trigger_layout.setColumnStretch(column, 1)
        layout.addWidget(trigger_group)

        acquisition_group = QGroupBox("Acquisition")
        acquisition_layout = QGridLayout(acquisition_group)
        self.sample_divider_spin = QSpinBox()
        self.sample_divider_spin.setRange(0, 65535)
        self.sample_divider_spin.setValue(0)
        self.sample_divider_spin.setToolTip(
            "The target records one sample every divider + 1 control ticks. "
            "A value of 0 disables division and samples every control tick."
        )
        self.sample_divider_spin.valueChanged.connect(self._update_rate_label)
        self.continuous_checkbox = QCheckBox("Enabled")
        self.continuous_checkbox.setToolTip(
            "Enable to configure and capture immediately, then re-arm after each completed "
            "snapshot. Disable to stop future re-arming without starting another capture."
        )
        self.continuous_checkbox.toggled.connect(self._on_continuous_toggled)
        self.capture_button = QPushButton("Configure && Capture")
        self.capture_button.setFixedHeight(control_height)
        self.capture_button.clicked.connect(self.start_capture)
        self.read_button = QPushButton("Read Current Snapshot")
        self.read_button.setFixedHeight(control_height)
        self.read_button.clicked.connect(self.read_current_snapshot)
        self.status_label = QLabel("Idle")
        self.status_label.setFixedHeight(control_height)
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet(
            "background: #ECEFF1; border: 1px solid #CFD8DC; border-radius: 3px;"
        )
        acquisition_controls = (
            (QLabel("Sampling divider"), self.sample_divider_spin),
            (QLabel("Continuous display"), self.continuous_checkbox),
            (QLabel("Acquisition"), self.capture_button),
            (QLabel("Snapshot"), self.read_button),
        )
        for column, (label, control) in enumerate(acquisition_controls):
            acquisition_layout.addWidget(label, 0, column)
            acquisition_layout.addWidget(control, 1, column)
            acquisition_layout.setColumnStretch(column, 1)
        acquisition_layout.addWidget(self.status_label, 2, 0, 1, 4)
        layout.addWidget(acquisition_group)

        persistence_group = QGroupBox("Waveform Persistence")
        persistence_layout = QGridLayout(persistence_group)
        self.afterglow_checkbox = QCheckBox("Enabled")
        self.afterglow_checkbox.toggled.connect(self._on_afterglow_toggled)
        self.afterglow_depth_spin = QSpinBox()
        self.afterglow_depth_spin.setRange(1, 20)
        self.afterglow_depth_spin.setValue(5)
        self.afterglow_depth_spin.setSuffix(" history frames")
        self.afterglow_depth_spin.valueChanged.connect(self._trim_persistence)
        self.afterglow_opacity_spin = QDoubleSpinBox()
        self.afterglow_opacity_spin.setRange(0.05, 0.90)
        self.afterglow_opacity_spin.setSingleStep(0.05)
        self.afterglow_opacity_spin.setValue(0.30)
        self.afterglow_opacity_spin.setSuffix(" max opacity")
        self.afterglow_opacity_spin.valueChanged.connect(self._refresh_afterglow_opacity)
        self.clear_afterglow_button = QPushButton("Clear Persistence")
        self.clear_afterglow_button.setFixedHeight(control_height)
        self.clear_afterglow_button.clicked.connect(self.clear_afterglow)
        persistence_controls = (
            (QLabel("Afterglow"), self.afterglow_checkbox),
            (QLabel("History depth"), self.afterglow_depth_spin),
            (QLabel("Maximum opacity"), self.afterglow_opacity_spin),
            (QLabel("History"), self.clear_afterglow_button),
        )
        for column, (label, control) in enumerate(persistence_controls):
            persistence_layout.addWidget(label, 0, column)
            persistence_layout.addWidget(control, 1, column)
            persistence_layout.setColumnStretch(column, 1)
        layout.addWidget(persistence_group)

        export_group = QGroupBox("Waveform Export")
        export_layout = QGridLayout(export_group)
        self.save_current_button = QPushButton("Save Current Frame")
        self.save_current_button.setFixedHeight(control_height)
        self.save_current_button.setEnabled(False)
        self.save_current_button.clicked.connect(self.save_current_frame)
        self.save_persistence_button = QPushButton("Save Persistence Frames")
        self.save_persistence_button.setFixedHeight(control_height)
        self.save_persistence_button.setEnabled(False)
        self.save_persistence_button.setToolTip(
            "Save every retained afterglow frame and the current bold frame to one CSV file."
        )
        self.save_persistence_button.clicked.connect(self.save_persistence_frames)
        export_layout.addWidget(self.save_current_button, 0, 0, 1, 2)
        export_layout.addWidget(self.save_persistence_button, 0, 2, 1, 2)
        for column in range(4):
            export_layout.setColumnStretch(column, 1)
        layout.addWidget(export_group)

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
        self.hermes.emit_log("Scope", message)

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
        if self.current_frame is not None:
            self.current_frame = None
            self.clear_afterglow()
            for curve in self.curves:
                curve.setData([], [])
            self.save_current_button.setEnabled(False)
            self.save_persistence_button.setEnabled(False)
        self.type_label.setText(self.DATA_TYPES.get(resource.sample_type, ("Unknown", None))[0])
        self.layout_label.setText(self.LAYOUT_NAMES.get(resource.layout, "Unknown"))
        self.channels_label.setText(str(resource.channels))
        self.depth_label.setText(str(resource.depth))
        self.sample_divider_spin.setEnabled(resource.protocol_version >= 2)
        self.sample_divider_spin.setValue(
            self.active_sample_dividers.get(resource.resource_id, 0)
            if resource.protocol_version >= 2 else 0
        )
        self._update_rate_label()
        self.trigger_channel_spin.setMaximum(max(0, resource.channels - 1))

    @staticmethod
    def _effective_sample_rate_hz(resource: ScopeResource, divider: int) -> float:
        """Return the configured per-channel sample rate."""
        return resource.sample_rate_hz / float(divider + 1)

    def _update_rate_label(self, *_args) -> None:
        """Show the base and configured Scope sample rates."""
        resource = self._selected_resource()
        if resource is None:
            self.rate_label.setText("-")
            return
        divider = self.sample_divider_spin.value() if resource.protocol_version >= 2 else 0
        effective_rate = self._effective_sample_rate_hz(resource, divider)
        if divider == 0:
            self.rate_label.setText(f"{resource.sample_rate_hz} Hz (no division)")
        else:
            self.rate_label.setText(
                f"{resource.sample_rate_hz} Hz / {divider + 1} = {effective_rate:g} Hz"
            )

    def start_capture(self) -> None:
        """Configure and arm the selected target scope resource."""
        resource = self._selected_resource()
        if not self.hermes.running or resource is None:
            self._log("Discover and select a Scope resource first.")
            return
        payload = self._build_configuration_payload(resource)
        if self.capture_active:
            if self.capture_phase == self.PHASE_POLLING:
                self.queued_configuration = payload
                self.poll_timer.stop()
                self.capture_button.setEnabled(False)
                self.status_label.setText("Trigger update queued")
                if not self.status_request_pending:
                    self._apply_queued_configuration()
                else:
                    QTimer.singleShot(300, self._apply_queued_configuration)
            else:
                self._log("Wait for the current Scope transfer to complete before reconfiguring.")
            return
        self._begin_configuration(payload)

    def _build_configuration_payload(self, resource: ScopeResource) -> bytes:
        """Build a target configuration from the currently editable controls."""
        fields = (
            self.OP_CONFIGURE,
            resource.resource_id,
            int(self.mode_combo.currentData()),
            self.trigger_channel_spin.value(),
            int(round(self.position_spin.value() * 10.0)),
            self.level_spin.value(),
            self.AUTO_TIMEOUT_MS,
        )
        if resource.protocol_version >= 2:
            return struct.pack("<BBBBHfIH", *fields, self.sample_divider_spin.value())
        return struct.pack("<BBBBHfI", *fields)

    def _begin_configuration(self, payload: bytes) -> None:
        """Start or restart configuration after all serial activity is quiescent."""
        self.capture_active = True
        self.capture_phase = self.PHASE_CONFIGURING
        self.capture_mode = payload[2]
        self.pending_sample_divider = (
            struct.unpack_from("<H", payload, 14)[0] if len(payload) >= 16 else 0
        )
        self.repeat_after_capture = self.continuous_checkbox.isChecked()
        self.repeat_timer.stop()
        self.poll_timer.stop()
        self.queued_configuration = None
        self.status_request_pending = False
        self.status_request_started = 0.0
        self.poll_count = 0
        self._set_acquisition_controls(False)
        self.status_label.setText("Configuring")
        self._send_capture_request(payload)

    def _apply_queued_configuration(self) -> None:
        """Apply a trigger edit queued while the target was waiting for an edge."""
        if self.queued_configuration is None or self.capture_phase != self.PHASE_POLLING:
            return
        if self.status_request_pending and time.monotonic() - self.status_request_started < 0.25:
            QTimer.singleShot(100, self._apply_queued_configuration)
            return
        payload = self.queued_configuration
        self.status_request_pending = False
        self._begin_configuration(payload)

    def _send_capture_request(self, payload: bytes, retry: bool = False) -> None:
        """Send one capture transaction and arm its packet-loss watchdog."""
        if not retry:
            self.pending_request = payload
            self.pending_request_retries = 0
        self.hermes.send_frame(0x01, self.base_command, payload, priority=0)
        self.request_timer.start()

    def _complete_capture_request(self, operation: int) -> None:
        """Acknowledge the response matching the outstanding operation."""
        if self.pending_request is not None and self.pending_request[0] == operation:
            self.request_timer.stop()
            self.pending_request = None
            self.pending_request_retries = 0

    def _retry_pending_request(self) -> None:
        """Retry a lost capture packet or release the UI after a bounded wait."""
        if not self.capture_active or self.pending_request is None:
            return
        if self.pending_request_retries >= self.MAX_REQUEST_RETRIES:
            self._finish_with_error("Scope request timed out; controls were released safely.")
            return
        self.pending_request_retries += 1
        self._log(
            f"Retrying Scope operation {self.pending_request[0]} "
            f"({self.pending_request_retries}/{self.MAX_REQUEST_RETRIES})."
        )
        self._send_capture_request(self.pending_request, retry=True)

    def read_current_snapshot(self) -> None:
        """Read the selected scope buffer without changing target capture state."""
        resource = self._selected_resource()
        if not self.hermes.running or self.capture_active or resource is None:
            return
        self.capture_active = True
        self.capture_phase = self.PHASE_DOWNLOADING
        self.repeat_after_capture = False
        self.repeat_timer.stop()
        self.status_request_pending = False
        self.status_request_started = 0.0
        self._set_acquisition_controls(False)
        self._prepare_download(resource, generation=None)

    def _request_scope_status(self) -> None:
        """Poll the target capture state until a snapshot is ready."""
        resource = self._selected_resource()
        if (
            not self.capture_active
            or self.capture_phase != self.PHASE_POLLING
            or resource is None
        ):
            self.poll_timer.stop()
            return
        if self.status_request_pending:
            if time.monotonic() - self.status_request_started < 0.25:
                return
            self.status_request_pending = False
        self.poll_count += 1
        if self.capture_mode == 0 and self.poll_count > 100:
            self._finish_with_error("Capture timed out.")
            return
        if self.capture_mode in (3, 4) and self.poll_count > 80:
            self._finish_with_error("Automatic trigger timed out without a completed snapshot.")
            return
        self.status_request_pending = True
        self.status_request_started = time.monotonic()
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
        self.metadata = {
            "resource": resource,
            "generation": generation,
            "sample_rate_hz": self._effective_sample_rate_hz(
                resource, self.active_sample_dividers.get(resource.resource_id, 0)
            ),
        }
        self.capture_phase = self.PHASE_DOWNLOADING
        self.status_request_pending = False
        self.status_request_started = 0.0
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
        self._send_capture_request(payload)

    def _handle_discovery(self, payload: bytes) -> None:
        """Parse one descriptor and continue the indexed discovery sequence."""
        if len(payload) < 5:
            self._finish_discovery("Scope discovery response is too short.")
            return
        operation, status, version, total, resource_id = payload[:5]
        if operation != self.OP_DISCOVER or version not in (1, 2) or status != 0:
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
                protocol_version=version,
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
            if self.continuous_checkbox.isChecked():
                QTimer.singleShot(0, self._start_continuous_if_enabled)

    def _finish_discovery(self, error: str) -> None:
        """Stop a failed discovery sequence and report its reason."""
        self.discovery_active = False
        self.refresh_button.setEnabled(True)
        self.status_label.setText(error)
        self._log(error)

    def _handle_status(self, payload: bytes) -> None:
        """Process a capture-state response and start reading when ready."""
        self.status_request_pending = False
        self.status_request_started = 0.0
        if len(payload) != 8:
            self._finish_with_error("Invalid Scope status response.")
            return
        operation, status, resource_id, state, generation = struct.unpack("<BBBBI", payload)
        resource = self._selected_resource()
        if operation != self.OP_STATUS or status != 0 or resource is None or resource_id != resource.resource_id:
            self._finish_with_error("Target rejected the Scope status query.")
            return
        if self.queued_configuration is not None:
            self._apply_queued_configuration()
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
        if operation == self.OP_READ and offset < self.read_offset:
            # A delayed response to a retried read is harmless and can be ignored.
            return
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
        self._complete_capture_request(self.OP_READ)
        self.memory_buffer[offset:offset + length] = data
        self.read_offset += length
        self._request_next_chunk()

    def _ensure_curves(self, channels: int) -> None:
        """Create exactly one plot curve for each decoded channel."""
        if len(self.curves) == channels:
            return
        self.clear_afterglow()
        self.current_frame = None
        self.save_current_button.setEnabled(False)
        self.save_persistence_button.setEnabled(False)
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
        """Preserve the current decoded frame as age-faded persistence data."""
        if not self.afterglow_checkbox.isChecked() or self.current_frame is None:
            return
        group = []
        for channel_data in self.current_frame.channel_data:
            item = self.plot.plot(self.current_frame.time_ms, channel_data)
            group.append(item)
        self.afterglow_groups.append(group)
        self.persistence_frames.append(self.current_frame)
        self._trim_persistence()
        self._refresh_afterglow_opacity()

    def _trim_persistence(self, *_args) -> None:
        """Trim retained plot items and export data to the configured history depth."""
        maximum_groups = self.afterglow_depth_spin.value()
        while len(self.afterglow_groups) > maximum_groups:
            oldest = self.afterglow_groups.pop(0)
            for item in oldest:
                self.plot.removeItem(item)
            self.persistence_frames.pop(0)

    def _refresh_afterglow_opacity(self, *_args) -> None:
        """Apply an age-dependent opacity to every retained waveform frame."""
        group_count = len(self.afterglow_groups)
        if group_count == 0:
            return
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
        self.persistence_frames = []

    def _on_afterglow_toggled(self, enabled: bool) -> None:
        """Clear retained frames when waveform persistence is disabled."""
        if not enabled:
            self.clear_afterglow()

    def _suggest_export_name(self, suffix: str) -> str:
        """Return a filesystem-friendly default name for a Scope CSV export."""
        resource = self._selected_resource()
        base_name = resource.name if resource is not None else "scope"
        safe_name = "_".join(base_name.lower().split())
        safe_name = "".join(character for character in safe_name if character.isalnum() or character == "_")
        return f"{safe_name or 'scope'}_{suffix}.csv"

    def save_current_frame(self) -> None:
        """Save the currently displayed bold waveform frame to CSV."""
        if self.current_frame is None:
            self._log("No Scope frame is available to save.")
            return
        path, _selected_filter = QFileDialog.getSaveFileName(
            self,
            "Save Current Scope Frame",
            self._suggest_export_name("current"),
            "CSV Files (*.csv)",
        )
        if path:
            self._write_frames_csv(path, [self.current_frame])

    def save_persistence_frames(self) -> None:
        """Save retained afterglow frames and the current frame to one CSV file."""
        if self.current_frame is None:
            self._log("No Scope frames are available to save.")
            return
        frames = [*self.persistence_frames, self.current_frame]
        path, _selected_filter = QFileDialog.getSaveFileName(
            self,
            "Save Scope Persistence Frames",
            self._suggest_export_name("persistence"),
            "CSV Files (*.csv)",
        )
        if path:
            self._write_frames_csv(path, frames)

    def _write_frames_csv(self, path: str, frames: list[ScopeFrame]) -> None:
        """Write one or more compatible Scope frames in long-form CSV rows."""
        channel_count = frames[0].channel_data.shape[0]
        if any(frame.channel_data.shape[0] != channel_count for frame in frames):
            self._log("Persistence frames have incompatible channel counts.")
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(
                    ["frame_index", "generation", "time_ms"] +
                    [f"channel_{index}" for index in range(channel_count)]
                )
                for frame_index, frame in enumerate(frames):
                    generation = "" if frame.generation is None else frame.generation
                    for sample_index, time_value in enumerate(frame.time_ms):
                        writer.writerow(
                            [frame_index, generation, float(time_value)] +
                            [float(frame.channel_data[channel, sample_index])
                             for channel in range(channel_count)]
                        )
        except OSError as error:
            self._log(f"Failed to save Scope CSV: {error}")
            return
        self._log(f"Saved {len(frames)} Scope frame(s) to {path}.")

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
        sample_rate_hz = self.metadata["sample_rate_hz"]
        time_ms = np.arange(resource.depth, dtype=np.float64) * (1000.0 / sample_rate_hz)
        self._ensure_curves(resource.channels)
        self._capture_afterglow()
        generation = self.metadata["generation"]
        self.current_frame = ScopeFrame(
            time_ms=time_ms.copy(),
            channel_data=channel_data.copy(),
            generation=generation,
        )
        for index, curve in enumerate(self.curves):
            curve.setData(time_ms, channel_data[index])
        trigger_sample = int(round(resource.depth * self.position_spin.value() / 100.0))
        self.trigger_line.setValue(trigger_sample * 1000.0 / sample_rate_hz)
        self.plot.enableAutoRange()
        self.save_current_button.setEnabled(True)
        self.save_persistence_button.setEnabled(True)
        suffix = "" if generation is None else f", generation {generation}"
        should_repeat = self.repeat_after_capture and self.continuous_checkbox.isChecked()
        self.status_label.setText("Capture complete; re-arming" if should_repeat else "Capture complete")
        self._finish_capture()
        self._log(f"Displayed {resource.depth} samples on {resource.channels} channels{suffix}.")
        if should_repeat:
            self.repeat_timer.start()

    def _repeat_capture_if_enabled(self) -> None:
        """Start the next snapshot only while continuous display remains selected."""
        if self.continuous_checkbox.isChecked() and not self.capture_active:
            self.start_capture()

    def _on_continuous_toggled(self, enabled: bool) -> None:
        """Start continuous capture on enable and stop future re-arming on disable."""
        self.repeat_after_capture = enabled
        if enabled:
            QTimer.singleShot(0, self._start_continuous_if_enabled)
            return
        self.repeat_timer.stop()
        if self.status_label.text().endswith("re-arming"):
            self.status_label.setText("Capture complete")

    def _start_continuous_if_enabled(self) -> None:
        """Start the first continuous acquisition when configuration is available."""
        if (
            self.continuous_checkbox.isChecked()
            and not self.capture_active
            and self.capture_button.isEnabled()
            and self.hermes.running
            and self._selected_resource() is not None
        ):
            self.start_capture()

    def _on_connection_state(self, connected: bool) -> None:
        """Cancel timers and release controls when the transport disconnects."""
        if connected:
            return
        self.poll_timer.stop()
        self.repeat_timer.stop()
        self.request_timer.stop()
        self.capture_active = False
        self.capture_phase = self.PHASE_IDLE
        self.status_request_pending = False
        self.status_request_started = 0.0
        self.queued_configuration = None
        self.pending_request = None
        self.pending_request_retries = 0
        self.repeat_after_capture = False
        self._set_acquisition_controls(True)

    def _set_acquisition_controls(self, enabled: bool) -> None:
        """Enable or disable controls that can start another acquisition."""
        self.capture_button.setEnabled(enabled)
        self.read_button.setEnabled(enabled)
        self.refresh_button.setEnabled(enabled and not self.discovery_active)

    def _finish_capture(self) -> None:
        """Release acquisition controls after a completed operation."""
        self.poll_timer.stop()
        self.request_timer.stop()
        self.capture_active = False
        self.capture_phase = self.PHASE_IDLE
        self.status_request_pending = False
        self.status_request_started = 0.0
        self.queued_configuration = None
        self.pending_request = None
        self.pending_request_retries = 0
        self._set_acquisition_controls(True)

    def _finish_with_error(self, message: str) -> None:
        """Stop acquisition and report a user-visible error."""
        self.repeat_after_capture = False
        self.repeat_timer.stop()
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
        elif operation == self.OP_CONFIGURE and self.capture_phase == self.PHASE_CONFIGURING:
            resource = self._selected_resource()
            if len(payload) != 3 or payload[1] != 0 or resource is None or payload[2] != resource.resource_id:
                self._finish_with_error("Target rejected the Scope configuration.")
                return
            self._complete_capture_request(self.OP_CONFIGURE)
            self.active_sample_dividers[resource.resource_id] = self.pending_sample_divider
            self.status_label.setText("Arming")
            self.capture_phase = self.PHASE_ARMING
            self._send_capture_request(bytes((self.OP_ARM, resource.resource_id)))
        elif operation == self.OP_ARM and self.capture_phase == self.PHASE_ARMING:
            resource = self._selected_resource()
            if len(payload) != 3 or payload[1] != 0 or resource is None or payload[2] != resource.resource_id:
                self._finish_with_error("Target rejected the Scope arm request.")
                return
            self._complete_capture_request(self.OP_ARM)
            self.status_label.setText("Armed")
            self.capture_phase = self.PHASE_POLLING
            self.capture_button.setEnabled(True)
            self.poll_timer.start()
            self._request_scope_status()
        elif operation == self.OP_STATUS and self.capture_phase == self.PHASE_POLLING:
            self._handle_status(payload)
        elif operation == self.OP_READ and self.capture_phase == self.PHASE_DOWNLOADING:
            self._handle_read(payload)
