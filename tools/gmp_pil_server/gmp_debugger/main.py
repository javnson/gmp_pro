import html
import os
import sys
from datetime import datetime

import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QTabWidget, QGroupBox, QComboBox,
                             QPushButton, QTextBrowser, QLabel, QFormLayout,
                             QProgressBar, QSizePolicy, QMenu, QToolButton)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal

# Shared communication engine.
from core_datalink import HermesDatalinkQt
from resource_discovery import ResourceDiscovery

# Decoupled feature pages.
from tabs.tab_ascii import TabAscii
from tabs.tab_raw import TabRaw
from tabs.tab_sim import TabSim
from tabs.tab_pil import TabPilBridge
from tabs.tab_tunable import TabTunableManager
from tabs.tab_mem_persp import TabMemPersp
from tabs.tab_chronos import TabChronosManager
from tabs.tab_dsa_scope import TabDsaScope

DATA_BITS_MAP = {'8': serial.EIGHTBITS, '7': serial.SEVENBITS, '6': serial.SIXBITS, '5': serial.FIVEBITS}
STOP_BITS_MAP = {'1': serial.STOPBITS_ONE, '1.5': serial.STOPBITS_ONE_POINT_FIVE, '2': serial.STOPBITS_TWO}
PARITY_MAP = {
    'None': serial.PARITY_NONE, 'Even': serial.PARITY_EVEN, 
    'Odd': serial.PARITY_ODD, 'Mark': serial.PARITY_MARK, 'Space': serial.PARITY_SPACE
}

LOG_SOURCE_COLORS = {
    "System": "#455A64",
    "Serial Terminal": "#546E7A",
    "Loop Test": "#00897B",
    "PIL Simulation": "#3949AB",
    "PIL Bridge": "#7B1FA2",
    "Tunable": "#2E7D32",
    "Memory": "#EF6C00",
    "Chronos": "#0277BD",
    "Scope": "#C62828",
}


class LogFilterMenu(QToolButton):
    """Dropdown checklist controlling which page logs remain visible."""

    visibility_changed = pyqtSignal()

    def __init__(self, sources: list[str], parent=None) -> None:
        super().__init__(parent)
        self.setPopupMode(QToolButton.InstantPopup)
        self.setToolButtonStyle(Qt.ToolButtonTextOnly)
        self._actions = {}
        menu = QMenu(self)
        for source in sources:
            action = menu.addAction(source)
            action.setCheckable(True)
            action.setChecked(True)
            action.toggled.connect(self._on_filter_toggled)
            self._actions[source] = action
        self.setMenu(menu)
        self._update_caption()

    def visible_sources(self) -> set[str]:
        """Return the source names currently selected for display."""
        return {name for name, action in self._actions.items() if action.isChecked()}

    def _on_filter_toggled(self, _checked: bool) -> None:
        """Refresh the summary caption and notify the log view."""
        self._update_caption()
        self.visibility_changed.emit()

    def _update_caption(self) -> None:
        """Summarize the number of visible page sources."""
        visible_count = len(self.visible_sources())
        self.setText(f"Sources: {visible_count}/{len(self._actions)}  ▾")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        target_unit_bytes = int(os.environ.get("GMP_DATALINK_TARGET_UNIT_BYTES", "1"))
        if target_unit_bytes not in (1, 2):
            target_unit_bytes = 1
        self.setWindowTitle(
            f"GMP Data Link Debugger / PIL Server — u{target_unit_bytes * 8} Target Profile"
        )
        self.resize(1100, 650)
        
        # Create the single shared communication and discovery services.
        self.hermes = HermesDatalinkQt()
        self.discovery = ResourceDiscovery(self.hermes)
        self.hermes.sig_log_event.connect(self.log_message)
        self.hermes.sig_log_msg.connect(lambda message: self.log_message("System", message))
        self.hermes.sig_conn_state.connect(self.update_ui_connection_state)
        self.discovery.discovery_error.connect(
            lambda message: self.log_message("System", message)
        )
        self.log_entries: list[tuple[str, str, str]] = []
        
        self.total_tx_bytes = 0
        self.total_rx_bytes = 0
        self.last_tx_bytes = 0
        self.last_rx_bytes = 0
        self.hermes.sig_bus_event.connect(self._on_bus_event_for_stats)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs, stretch=4)

        # Mount feature pages and inject their shared services.
        self.tab_raw = TabRaw(self.hermes)
        self.tabs.addTab(self.tab_raw, "1. Serial Terminal (RAW)")
        
        self.tab_ascii = TabAscii(self.hermes)
        self.tabs.addTab(self.tab_ascii, "2. Data Link Loop Test (ECHO)")

        self.tab_sim = TabSim(self.hermes)
        self.tabs.addTab(self.tab_sim, "3. PIL Simulation Engine")

        self.tab_pil_bridge = TabPilBridge(self.hermes, self.tab_sim)
        self.tabs.addTab(self.tab_pil_bridge, "4. Simulink-PIL Bridge")

        self.tab_tunable = TabTunableManager(self.hermes, self.discovery)
        self.tabs.addTab(self.tab_tunable, "5. Online Tunable Workbench")

        self.tab_mem_persp = TabMemPersp(self.hermes, self.discovery)
        self.tabs.addTab(self.tab_mem_persp, "6. Argos Memory Perspective")

        self.tab_chronos = TabChronosManager(self.hermes)
        self.tabs.addTab(self.tab_chronos, "7. Chronos Waveform Recorder")

        self.tab_dsa_scope = TabDsaScope(self.hermes)
        self.tabs.addTab(self.tab_dsa_scope, "8. Data Link Scope")

        # Cross-page bus ownership signals.
        self.tab_pil_bridge.sig_rx_parsed.connect(self.tab_sim.update_rx_ui_from_bridge)
        self.tab_tunable.sig_global_bus_busy.connect(self.tab_pil_bridge.set_bus_preempted)
        
        # Global serial controls, statistics, and log panel.
        right_panel = QVBoxLayout()
        main_layout.addLayout(right_panel, stretch=1)
        
        # Fixed-height serial configuration.
        self._build_serial_panel(right_panel)
        # Fixed-height transmit and receive statistics.
        self._build_stats_panel(right_panel) 
        
        self._build_log_panel(right_panel)

        self.update_ui_connection_state(False)

        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self._update_bus_stats)
        self.stats_timer.start(1000)

    # ---------------------------------------------------------
    # User interface construction and basic interaction.
    # ---------------------------------------------------------
    def _build_serial_panel(self, layout: QVBoxLayout):
        group_box = QGroupBox("Serial Configuration")
        # Keep the compact control group at its preferred height.
        group_box.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum) 
        
        form_layout = QFormLayout()
        form_layout.setLabelAlignment(Qt.AlignRight)
        
        port_hlayout = QHBoxLayout()
        self.cb_ports = QComboBox()
        self.cb_ports.setMinimumWidth(150)
        self.cb_ports.view().setMinimumWidth(350) 
        
        self.btn_refresh = QPushButton("↻")
        self.btn_refresh.setMaximumWidth(30)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        port_hlayout.addWidget(self.cb_ports)
        port_hlayout.addWidget(self.btn_refresh)
        
        self.cb_baud = QComboBox()
        self.cb_baud.setEditable(True) 
        self.cb_baud.addItems(["9600", "115200", "256000", "460800", "921600", "2000000"])
        self.cb_baud.setCurrentText("921600") 
        
        self.cb_data_bits = QComboBox()
        self.cb_data_bits.addItems(list(DATA_BITS_MAP.keys()))
        self.cb_data_bits.setCurrentText("8")
        
        self.cb_stop_bits = QComboBox()
        self.cb_stop_bits.addItems(list(STOP_BITS_MAP.keys()))
        self.cb_stop_bits.setCurrentText("1")
        
        self.cb_parity = QComboBox()
        self.cb_parity.addItems(list(PARITY_MAP.keys()))
        self.cb_parity.setCurrentText("None")

        form_layout.addRow("Port:", port_hlayout)
        form_layout.addRow("Baud rate:", self.cb_baud)
        form_layout.addRow("Data bits:", self.cb_data_bits)
        form_layout.addRow("Stop bits:", self.cb_stop_bits)
        form_layout.addRow("Parity:", self.cb_parity)
        
        self.btn_connect = QPushButton("Open Port")
        self.btn_connect.setMinimumHeight(40)
        font = self.btn_connect.font()
        font.setBold(True)
        self.btn_connect.setFont(font)
        self.btn_connect.clicked.connect(self.toggle_connection)
        
        vbox = QVBoxLayout()
        vbox.addLayout(form_layout)
        vbox.addSpacing(5)
        vbox.addWidget(self.btn_connect)
        
        group_box.setLayout(vbox)
        layout.addWidget(group_box)
        
        self.refresh_ports()

    def _build_stats_panel(self, layout: QVBoxLayout):
        """Build independent transmit and receive bus-load indicators."""
        stats_group = QGroupBox("Bus Load")
        # Keep the statistics group compact.
        stats_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        vbox = QVBoxLayout()
        vbox.setSpacing(8)
        
        # Transfer-rate labels.
        h_speeds = QHBoxLayout()
        self.lbl_speed_tx = QLabel("TX: 0.0 kB/s")
        self.lbl_speed_rx = QLabel("RX: 0.0 kB/s")
        self.lbl_speed_tx.setStyleSheet("color: #4527A0; font-weight: bold;")
        self.lbl_speed_rx.setStyleSheet("color: #00796B; font-weight: bold;")
        h_speeds.addWidget(self.lbl_speed_tx)
        h_speeds.addWidget(self.lbl_speed_rx)
        vbox.addLayout(h_speeds)
        
        # Transmit utilization.
        self.bar_tx = QProgressBar()
        self.bar_tx.setFixedHeight(18)
        self.bar_tx.setRange(0, 100)
        self.bar_tx.setValue(0)
        self.bar_tx.setFormat("TX load: %p%")
        self.bar_tx.setStyleSheet(self._get_bar_stylesheet("#E0E0E0", "gray"))
        vbox.addWidget(self.bar_tx)

        # Receive utilization.
        self.bar_rx = QProgressBar()
        self.bar_rx.setFixedHeight(18)
        self.bar_rx.setRange(0, 100)
        self.bar_rx.setValue(0)
        self.bar_rx.setFormat("RX load: %p%")
        self.bar_rx.setStyleSheet(self._get_bar_stylesheet("#E0E0E0", "gray"))
        vbox.addWidget(self.bar_rx)
        
        stats_group.setLayout(vbox)
        layout.addWidget(stats_group)

    def _build_log_panel(self, layout: QVBoxLayout) -> None:
        """Build the source-colored system log and its page filter menu."""
        title_row = QHBoxLayout()
        title_row.addWidget(QLabel("<b>System Log:</b>"))
        title_row.addStretch()
        self.log_filter = LogFilterMenu(list(LOG_SOURCE_COLORS))
        self.log_filter.setToolTip("Uncheck a page to hide its messages from System Log.")
        self.log_filter.visibility_changed.connect(self._refresh_system_log)
        title_row.addWidget(self.log_filter)
        clear_button = QPushButton("Clear")
        clear_button.setMaximumWidth(55)
        clear_button.clicked.connect(self._clear_system_log)
        title_row.addWidget(clear_button)
        layout.addLayout(title_row)

        self.sys_log = QTextBrowser()
        self.sys_log.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.sys_log.setStyleSheet(
            "QTextBrowser { background: #FAFBFC; border: 1px solid #CFD8DC; "
            "color: #263238; selection-background-color: #B0BEC5; }"
        )
        layout.addWidget(self.sys_log, stretch=1)

    # ---------------------------------------------------------
    # Serial control and data-rate accounting.
    # ---------------------------------------------------------
    def _on_bus_event_for_stats(self, ev: dict):
        if ev['dir'] == 'TX':
            self.total_tx_bytes += len(ev['data'])
        else:
            self.total_rx_bytes += len(ev['data'])

    def _get_bar_stylesheet(self, color: str, text_color: str = "black") -> str:
        """Return a progress-bar style for the requested load color."""
        return f"""
            QProgressBar {{
                border: 1px solid #BDBDBD;
                border-radius: 4px;
                text-align: center;
                font-weight: bold;
                background-color: #F5F5F5;
                color: {text_color};
                font-size: 11px;
            }}
            QProgressBar::chunk {{ background-color: {color}; width: 6px; margin: 0.5px; }}
        """

    def _update_bus_stats(self):
        """Update independent transmit and receive rates once per second."""
        if not self.hermes.running:
            self.lbl_speed_tx.setText("TX: 0.0 kB/s")
            self.lbl_speed_rx.setText("RX: 0.0 kB/s")
            self.bar_tx.setValue(0)
            self.bar_rx.setValue(0)
            self.bar_tx.setStyleSheet(self._get_bar_stylesheet("#E0E0E0", "gray"))
            self.bar_rx.setStyleSheet(self._get_bar_stylesheet("#E0E0E0", "gray"))
            return

        tx_diff = self.total_tx_bytes - self.last_tx_bytes
        rx_diff = self.total_rx_bytes - self.last_rx_bytes
        
        self.last_tx_bytes = self.total_tx_bytes
        self.last_rx_bytes = self.total_rx_bytes

        tx_kbs = tx_diff / 1024.0
        rx_kbs = rx_diff / 1024.0

        self.lbl_speed_tx.setText(f"TX: {tx_kbs:.1f} kB/s")
        self.lbl_speed_rx.setText(f"RX: {rx_kbs:.1f} kB/s")

        # Approximate UART capacity as ten wire bits per payload byte.
        try:
            baud_rate = int(self.cb_baud.currentText())
            max_bytes_per_sec = baud_rate / 10.0
            
            tx_util = (tx_diff / max_bytes_per_sec) * 100.0 if max_bytes_per_sec > 0 else 0.0
            rx_util = (rx_diff / max_bytes_per_sec) * 100.0 if max_bytes_per_sec > 0 else 0.0
        except ValueError:
            tx_util, rx_util = 0.0, 0.0

        # Clamp display values to the progress-bar range.
        tx_int = int(min(100, max(0, tx_util)))
        rx_int = int(min(100, max(0, rx_util)))

        self.bar_tx.setValue(tx_int)
        self.bar_rx.setValue(rx_int)

        # Apply warning colors as utilization increases.
        tx_color = "#E53935" if tx_int > 80 else ("#FF9800" if tx_int > 50 else "#7E57C2")
        rx_color = "#E53935" if rx_int > 80 else ("#FF9800" if rx_int > 50 else "#26A69A")

        self.bar_tx.setStyleSheet(self._get_bar_stylesheet(tx_color))
        self.bar_rx.setStyleSheet(self._get_bar_stylesheet(rx_color))

    def refresh_ports(self):
        self.cb_ports.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            desc = p.description.replace(f"({p.device})", "").strip()
            display_text = f"{p.device}: {desc}" if desc else p.device
            self.cb_ports.addItem(display_text, userData=p.device)

    def toggle_connection(self):
        if not self.hermes.running:
            port = self.cb_ports.currentData()
            if not port:
                self.log_message("System", "Select a valid serial port first.")
                return
            baud = int(self.cb_baud.currentText())
            data_bits = DATA_BITS_MAP[self.cb_data_bits.currentText()]
            stop_bits = STOP_BITS_MAP[self.cb_stop_bits.currentText()]
            parity = PARITY_MAP[self.cb_parity.currentText()]
            
            self.hermes.connect_serial(port, baud, data_bits, parity, stop_bits)
        else:
            self.hermes.close()

    def update_ui_connection_state(self, is_connected: bool):
        if is_connected:
            self.btn_connect.setText("Close Port")
            self.btn_connect.setStyleSheet("""
                QPushButton { border-left: 6px solid #4CAF50; background-color: #E8F5E9; border-radius: 3px; }
                QPushButton:hover { background-color: #C8E6C9; }
            """)
            self._set_combos_enabled(False)
        else:
            self.btn_connect.setText("Open Port")
            self.btn_connect.setStyleSheet("""
                QPushButton { border-left: 6px solid #F44336; background-color: #FAFAFA; border-radius: 3px; }
                QPushButton:hover { background-color: #EEEEEE; }
            """)
            self._set_combos_enabled(True)

    def _set_combos_enabled(self, state: bool):
        self.cb_ports.setEnabled(state)
        self.cb_baud.setEnabled(state)
        self.cb_data_bits.setEnabled(state)
        self.cb_stop_bits.setEnabled(state)
        self.cb_parity.setEnabled(state)
        self.btn_refresh.setEnabled(state)

    def log_message(self, source: str, message: str) -> None:
        """Store and render one plain-text message using its page accent color."""
        if source not in LOG_SOURCE_COLORS:
            message = f"[{source}] {message}"
            source = "System"
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_entries.append((timestamp, source, str(message)))
        if len(self.log_entries) > 2000:
            del self.log_entries[:len(self.log_entries) - 2000]
        if source in self.log_filter.visible_sources():
            self._append_log_entry(timestamp, source, str(message))

    def _append_log_entry(self, timestamp: str, source: str, message: str) -> None:
        """Append one escaped and consistently styled entry to System Log."""
        color = LOG_SOURCE_COLORS[source]
        safe_message = html.escape(message).replace("\n", "<br>")
        self.sys_log.append(
            f"<span style='color:#90A4AE;'>{timestamp}</span> "
            f"<span style='color:{color}; font-weight:600;'>[{source}]</span> "
            f"<span style='color:#263238;'>{safe_message}</span>"
        )
        scrollbar = self.sys_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _refresh_system_log(self) -> None:
        """Rebuild the log when one or more page filters change."""
        visible_sources = self.log_filter.visible_sources()
        self.sys_log.clear()
        for timestamp, source, message in self.log_entries:
            if source in visible_sources:
                self._append_log_entry(timestamp, source, message)

    def _clear_system_log(self) -> None:
        """Discard the retained log history and clear the view."""
        self.log_entries.clear()
        self.sys_log.clear()

    def closeEvent(self, event):
        self.log_message("System", "Closing the communication engine...")
        
        if hasattr(self.tab_tunable, 'tab_widget'):
            for i in range(self.tab_tunable.tab_widget.count()):
                widget = self.tab_tunable.tab_widget.widget(i)
                if hasattr(widget, 'stop_timers'):
                    widget.stop_timers()
                    
        if hasattr(self.tab_pil_bridge, 'running') and self.tab_pil_bridge.running:
            self.tab_pil_bridge.toggle_bridge()
            
        if self.hermes.running:
            self.hermes.close()
            
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion") 
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
