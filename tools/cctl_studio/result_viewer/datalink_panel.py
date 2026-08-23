"""Embedded GMP Data Link Studio pages for a managed CCTL simulator."""

from __future__ import annotations

import html
import os
import sys
from datetime import datetime
from pathlib import Path

from PyQt5 import QtCore, QtWidgets


def _studio_directory() -> Path:
    configured_root = os.environ.get("GMP_PRO_LOCATION")
    root = Path(configured_root) if configured_root else Path(__file__).resolve().parents[3]
    return root / "tools" / "gmp_datalink" / "datalink_studio"


STUDIO_DIRECTORY = _studio_directory()
if str(STUDIO_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(STUDIO_DIRECTORY))

from core_datalink import HermesDatalinkQt  # noqa: E402
from resource_discovery import ResourceDiscovery  # noqa: E402
from tabs.tab_ascii import TabAscii  # noqa: E402
from tabs.tab_chronos import TabChronosManager  # noqa: E402
from tabs.tab_dsa_scope import TabDsaScope  # noqa: E402
from tabs.tab_mem_persp import TabMemPersp  # noqa: E402
from tabs.tab_raw import TabRaw  # noqa: E402
from tabs.tab_tunable import TabTunableManager  # noqa: E402


class DataLinkPanel(QtWidgets.QWidget):
    """Reuse Data Link Studio pages over the supervised CCTL byte transport."""

    def __init__(self, simulation, parent=None) -> None:
        super().__init__(parent)
        self.simulation = simulation
        self.hermes = HermesDatalinkQt()
        self.discovery = ResourceDiscovery(self.hermes)

        layout = QtWidgets.QVBoxLayout(self)
        status_row = QtWidgets.QHBoxLayout()
        self.connection_status = QtWidgets.QLabel("CCTL Data Link: disconnected")
        self.connection_status.setStyleSheet("font-weight: 600; color: #9E2A2B;")
        status_row.addWidget(self.connection_status)
        status_row.addStretch(1)
        discover = QtWidgets.QPushButton("Discover target facilities")
        discover.clicked.connect(self._discover_resources)
        status_row.addWidget(discover)
        layout.addLayout(status_row)

        self.tabs = QtWidgets.QTabWidget()
        self.tab_raw = TabRaw(self.hermes)
        self.tab_ascii = TabAscii(self.hermes)
        self.tab_tunable = TabTunableManager(self.hermes, self.discovery)
        self.tab_mem_persp = TabMemPersp(self.hermes, self.discovery)
        self.tab_chronos = TabChronosManager(self.hermes)
        self.tab_dsa_scope = TabDsaScope(self.hermes)
        for widget, title in (
            (self.tab_raw, "Raw"),
            (self.tab_ascii, "Echo"),
            (self.tab_tunable, "Tunable"),
            (self.tab_mem_persp, "Memory"),
            (self.tab_chronos, "Chronos"),
            (self.tab_dsa_scope, "Data Link Scope"),
        ):
            self.tabs.addTab(widget, title)
        layout.addWidget(self.tabs, 1)

        self.log = QtWidgets.QTextBrowser()
        self.log.setMaximumHeight(110)
        self.log.setPlaceholderText("Data Link events")
        layout.addWidget(self.log)

        self.hermes.sig_log_event.connect(self._append_log)
        self.hermes.sig_conn_state.connect(self._connection_changed)
        self.simulation.datalink_received.connect(self.hermes.feed_transport)
        self.simulation.state_changed.connect(self._simulation_state_changed)

    @QtCore.pyqtSlot(str)
    def _simulation_state_changed(self, state: str) -> None:
        connected_states = {"ready", "paused", "running", "stopping"}
        if state in connected_states and not self.hermes.running:
            self.hermes.connect_transport(
                self.simulation.send_datalink, "CCTL managed Data Link"
            )
        elif state in {"stopped", "completed", "failed"} and self.hermes.running:
            self.hermes.close()

    @QtCore.pyqtSlot()
    def _discover_resources(self) -> None:
        self.discovery.discover_tunables()
        self.discovery.discover_memory()

    @QtCore.pyqtSlot(bool)
    def _connection_changed(self, connected: bool) -> None:
        if connected:
            self.connection_status.setText("CCTL Data Link: connected (u8 target)")
            self.connection_status.setStyleSheet("font-weight: 600; color: #217346;")
        else:
            self.connection_status.setText("CCTL Data Link: disconnected")
            self.connection_status.setStyleSheet("font-weight: 600; color: #9E2A2B;")

    @QtCore.pyqtSlot(str, str)
    def _append_log(self, source: str, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log.append(
            f"<span style='color:#657786'>{timestamp}</span> "
            f"<b>{html.escape(source)}</b>: {html.escape(message)}"
        )

    def shutdown(self) -> None:
        self.hermes.close()
