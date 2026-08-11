import re
import html
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTextBrowser, 
                             QPlainTextEdit, QPushButton, QLabel, QRadioButton, QLineEdit)
from PyQt5.QtCore import Qt, QTimer
from core_datalink import HermesDatalinkQt

class TabAscii(QWidget):
    def __init__(self, hermes: HermesDatalinkQt):
        super().__init__()
        self.hermes = hermes
        # Consume shared bus events and retain only Data Link frames.
        self.hermes.sig_bus_event.connect(self.on_bus_event)
        
        self.history = []
        self.rx_total_bytes = 0
        self.tx_total_bytes = 0
        
        self._needs_update = False
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._render_html)
        self.update_timer.start(50)
        
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Receive controls.
        rx_ctrl_layout = QHBoxLayout()
        rx_ctrl_layout.addWidget(QLabel("<b>Protocol Payload Log:</b>"))
        
        self.rb_rx_ascii = QRadioButton("ASCII")
        self.rb_rx_hex = QRadioButton("HEX")
        self.rb_rx_ascii.setChecked(True)
        self.rb_rx_ascii.toggled.connect(self.request_render)
        
        rx_ctrl_layout.addSpacing(20)
        rx_ctrl_layout.addWidget(QLabel("View:"))
        rx_ctrl_layout.addWidget(self.rb_rx_ascii)
        rx_ctrl_layout.addWidget(self.rb_rx_hex)
        rx_ctrl_layout.addStretch()
        
        self.lbl_counters = QLabel("RX Payload: 0 B  |  TX Payload: 0 B")
        self.lbl_counters.setStyleSheet("color: blue; font-weight: bold;")
        rx_ctrl_layout.addWidget(self.lbl_counters)
        
        self.btn_clear_rx = QPushButton("Clear")
        self.btn_clear_rx.clicked.connect(self.clear_history)
        rx_ctrl_layout.addWidget(self.btn_clear_rx)
        layout.addLayout(rx_ctrl_layout)
        
        # Receive display.
        self.rx_view = QTextBrowser()
        self.rx_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.rx_view.setStyleSheet("background-color: #FAFAFA; font-size: 13px;")
        layout.addWidget(self.rx_view, stretch=3)
        
        # Transmit controls and format options.
        tx_ctrl_layout = QHBoxLayout()
        self.rb_tx_ascii = QRadioButton("ASCII payload")
        self.rb_tx_hex = QRadioButton("HEX payload")
        self.rb_tx_ascii.setChecked(True)
        self.rb_tx_ascii.toggled.connect(self.update_tx_status)
        self.rb_tx_hex.toggled.connect(self.update_tx_status)
        
        tx_ctrl_layout.addWidget(self.rb_tx_ascii)
        tx_ctrl_layout.addWidget(self.rb_tx_hex)
        tx_ctrl_layout.addSpacing(20)
        
        # Sequence identifier.
        tx_ctrl_layout.addWidget(QLabel("Seq/ID:"))
        self.tx_id_input = QLineEdit("0x01")
        self.tx_id_input.setMaximumWidth(50)
        tx_ctrl_layout.addWidget(self.tx_id_input)

        tx_ctrl_layout.addSpacing(10)

        # Command identifier.
        tx_ctrl_layout.addWidget(QLabel("CMD:"))
        self.tx_cmd_input = QLineEdit("0x00")  # ECHO is the convenient default.
        self.tx_cmd_input.setMaximumWidth(50)
        tx_ctrl_layout.addWidget(self.tx_cmd_input)
        
        tx_ctrl_layout.addStretch()
        
        self.lbl_tx_len = QLabel("Ready")
        tx_ctrl_layout.addWidget(self.lbl_tx_len)
        layout.addLayout(tx_ctrl_layout)
        
        # Payload editor and send action.
        tx_input_layout = QHBoxLayout()
        self.tx_input = QPlainTextEdit()
        self.tx_input.setPlaceholderText("Enter payload data...")
        self.tx_input.setMaximumHeight(100)
        self.tx_input.textChanged.connect(self.update_tx_status)
        
        self.btn_send = QPushButton("Send Framed\n(Data Link)")
        self.btn_send.setMinimumHeight(70)
        self.btn_send.setMinimumWidth(120)
        self.btn_send.setStyleSheet("background-color: #E8F5E9; font-weight: bold;")
        self.btn_send.clicked.connect(self.send_data)
        
        tx_input_layout.addWidget(self.tx_input)
        tx_input_layout.addWidget(self.btn_send)
        layout.addLayout(tx_input_layout, stretch=1)

    def _parse_friendly_hex(self, text: str) -> bytes:
        clean_str = re.sub(r'(0[xX])|[^0-9a-fA-F]', '', text)
        if not clean_str: return b''
        if len(clean_str) % 2 != 0: return None
        return bytes.fromhex(clean_str)

    def update_tx_status(self):
        text = self.tx_input.toPlainText()
        if not text:
            self.lbl_tx_len.setText("0 B")
            self.btn_send.setEnabled(True)
            return

        if self.rb_tx_ascii.isChecked():
            length = len(text.encode('utf-8'))
            self.lbl_tx_len.setText(f"{length} B")
            self.lbl_tx_len.setStyleSheet("color: green;" if length <= 256 else "color: red;")
            self.btn_send.setEnabled(length <= 256)
        else:
            payload = self._parse_friendly_hex(text)
            if payload is None:
                self.lbl_tx_len.setText("Incomplete HEX")
                self.lbl_tx_len.setStyleSheet("color: red;")
                self.btn_send.setEnabled(False)
            else:
                length = len(payload)
                self.lbl_tx_len.setText(f"{length} B")
                self.lbl_tx_len.setStyleSheet("color: green;" if length <= 256 else "color: red;")
                self.btn_send.setEnabled(length <= 256)

    def send_data(self):
        text = self.tx_input.toPlainText()
        
        # Empty payloads are valid for command-only requests and acknowledgements.
        if not text:
            payload = b''
        else:
            payload = text.encode('utf-8') if self.rb_tx_ascii.isChecked() else self._parse_friendly_hex(text)
            if payload is None: return
        
        # Parse the user-supplied sequence identifier.
        try:
            seq_id = int(self.tx_id_input.text(), 16)
        except ValueError:
            seq_id = 0x01
            self.tx_id_input.setText("0x01")

        # Parse the user-supplied command identifier.
        try:
            cmd = int(self.tx_cmd_input.text(), 16)
        except ValueError:
            cmd = 0x00
            self.tx_cmd_input.setText("0x00")
            
        # Manual protocol tests use background priority.
        self.hermes.send_frame(target_id=seq_id, cmd=cmd, payload=payload, priority=2)
        self.hermes.emit_log(
            "Loop Test",
            f"Sent command 0x{cmd:02X} with {len(payload)} payload bytes.",
        )

    def on_bus_event(self, ev: dict):
        # This page consumes framed traffic only.
        if ev['type'] != 'DL': return
        
        # Report payload bytes without framing overhead.
        self.rx_total_bytes += len(ev['dl_payload']) if ev['dir'] == 'RX' else 0
        self.tx_total_bytes += len(ev['dl_payload']) if ev['dir'] == 'TX' else 0
        
        self.history.append(ev)
        if len(self.history) > 300: self.history = self.history[-300:]
        self.request_render()

    def clear_history(self):
        self.history.clear()
        self.rx_total_bytes = 0
        self.tx_total_bytes = 0
        self.request_render()

    def request_render(self):
        self._needs_update = True

    def _render_html(self):
        if not self._needs_update: return
        self._needs_update = False
        
        self.lbl_counters.setText(f"RX Payload: {self.rx_total_bytes} B  |  TX Payload: {self.tx_total_bytes} B")
        is_hex = self.rb_rx_hex.isChecked()
        html_parts = []
        
        for ev in self.history:
            payload_bytes = ev['dl_payload']
            if is_hex: 
                text_str = payload_bytes.hex(' ').upper()
            else:      
                text_str = html.escape(payload_bytes.decode('utf-8', errors='replace')).replace('\n', '<br>')
            
            if ev['dir'] == 'TX':
                color = '#4527A0'  # Purple transmit highlight.
                header = f"[{ev['dir']}: {ev['time']} | Payload -> Seq:0x{ev['dl_target']:02X} CMD:0x{ev['dl_cmd']:02X}] >>>"
            else:
                crc_str = "OK" if ev['dl_crc_ok'] else "FAIL"
                
                # Decode and highlight NACK responses.
                if ev['dl_cmd'] == 0x01:
                    color = '#E91E63'
                    header = f"[{ev['dir']}: {ev['time']} | Payload <- Seq:0x{ev['dl_target']:02X} CMD:0x{ev['dl_cmd']:02X} (NACK) | CRC:{crc_str}] >>>"
                    
                    # NACK payload: rejected command followed by the error code.
                    if len(payload_bytes) >= 2:
                        rejected_cmd = payload_bytes[0]
                        err_code = payload_bytes[1]
                        nack_notice = f"<b>[System] NACK rejected command 0x{rejected_cmd:02X} (error 0x{err_code:02X}).</b><br>"
                        text_str = nack_notice + text_str
                    else:
                        text_str = "<b>[System] NACK received without a decoded reason.</b><br>" + text_str
                else:
                    color = '#E65100' if ev['dl_crc_ok'] else '#D32F2F'
                    header = f"[{ev['dir']}: {ev['time']} | Payload <- Seq:0x{ev['dl_target']:02X} CMD:0x{ev['dl_cmd']:02X} | CRC:{crc_str}] >>>"
                
            html_parts.append(f"<div style='color:{color}; font-family:Consolas, monospace; margin-bottom:8px; line-height: 1.4;'><b>{header}</b><br>{text_str}</div>")
                
        self.rx_view.setHtml("".join(html_parts))
        scrollbar = self.rx_view.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
