import re
import html
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTextBrowser, 
                             QPlainTextEdit, QPushButton, QLabel, QRadioButton, QCheckBox, QButtonGroup)
from PyQt5.QtCore import Qt, QTimer
from core_datalink import HermesDatalinkQt

class TabRaw(QWidget):
    def __init__(self, hermes: HermesDatalinkQt):
        super().__init__()
        self.hermes = hermes
        self.hermes.sig_bus_event.connect(self.on_bus_event)
        
        self.history = []
        self.rx_total_bytes = 0
        self.tx_total_bytes = 0
        
        self._needs_update = False
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._render_html)
        self.update_timer.start(50) # 20 FPS 防卡死渲染
        
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        # --- 接收控制栏 ---
        rx_ctrl_layout = QHBoxLayout()
        rx_ctrl_layout.addWidget(QLabel("<b>全局总线监控:</b>"))
        
        self.rb_rx_ascii = QRadioButton("ASCII")
        self.rb_rx_hex = QRadioButton("HEX")
        self.rb_rx_ascii.setChecked(True)
        self.rb_rx_ascii.toggled.connect(self.request_render)
        
        self.chk_raw_rx = QCheckBox("RAW RX")
        self.chk_raw_tx = QCheckBox("RAW TX")
        self.chk_dl     = QCheckBox("DL 协议帧")
        self.chk_raw_rx.setChecked(True)
        self.chk_raw_tx.setChecked(True)
        self.chk_dl.setChecked(True)
        for cb in (self.chk_raw_rx, self.chk_raw_tx, self.chk_dl):
            cb.stateChanged.connect(self.request_render)
        
        rx_ctrl_layout.addSpacing(10)
        rx_ctrl_layout.addWidget(QLabel("视图:"))
        rx_ctrl_layout.addWidget(self.rb_rx_ascii)
        rx_ctrl_layout.addWidget(self.rb_rx_hex)
        rx_ctrl_layout.addSpacing(15)
        rx_ctrl_layout.addWidget(QLabel("筛选:"))
        rx_ctrl_layout.addWidget(self.chk_raw_rx)
        rx_ctrl_layout.addWidget(self.chk_raw_tx)
        rx_ctrl_layout.addWidget(self.chk_dl)
        rx_ctrl_layout.addStretch()
        
        self.lbl_counters = QLabel("RX: 0 B  |  TX: 0 B")
        self.lbl_counters.setStyleSheet("color: blue; font-weight: bold;")
        rx_ctrl_layout.addWidget(self.lbl_counters)
        
        self.btn_clear_rx = QPushButton("清空")
        self.btn_clear_rx.clicked.connect(self.clear_history)
        rx_ctrl_layout.addWidget(self.btn_clear_rx)
        layout.addLayout(rx_ctrl_layout)
        
        # --- 接收显示区 ---
        self.rx_view = QTextBrowser()
        self.rx_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.rx_view.setStyleSheet("background-color: #FAFAFA; font-size: 13px;")
        layout.addWidget(self.rx_view, stretch=3)
        
        # --- 发送控制栏 ---
        tx_ctrl_layout = QHBoxLayout()
        self.rb_tx_ascii = QRadioButton("盲发 ASCII")
        self.rb_tx_hex = QRadioButton("盲发 HEX")
        self.rb_tx_ascii.setChecked(True)
        self.rb_tx_ascii.toggled.connect(self.update_tx_status)
        self.rb_tx_hex.toggled.connect(self.update_tx_status)
        
        tx_ctrl_layout.addWidget(self.rb_tx_ascii)
        tx_ctrl_layout.addWidget(self.rb_tx_hex)
        tx_ctrl_layout.addStretch()
        
        self.lbl_tx_len = QLabel("Ready")
        tx_ctrl_layout.addWidget(self.lbl_tx_len)
        layout.addLayout(tx_ctrl_layout)
        
        # --- 发送输入区 ---
        tx_input_layout = QHBoxLayout()
        self.tx_input = QPlainTextEdit()
        self.tx_input.setPlaceholderText("原生串口盲发 (不带协议封装)。\nHEX 模式兼容性极强: 1A, 0x2b, 3C 4d 均可识别。")
        self.tx_input.setMaximumHeight(100)
        self.tx_input.textChanged.connect(self.update_tx_status)
        
        self.btn_send = QPushButton("盲发 RAW\n(不带头部)")
        self.btn_send.setMinimumHeight(70)
        self.btn_send.setMinimumWidth(120)
        self.btn_send.setStyleSheet("background-color: #E3F2FD; font-weight: bold;")
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
            self.lbl_tx_len.setText(f"{len(text.encode('utf-8'))} B")
            self.lbl_tx_len.setStyleSheet("color: green;")
            self.btn_send.setEnabled(True)
        else:
            payload = self._parse_friendly_hex(text)
            if payload is None:
                self.lbl_tx_len.setText("HEX 长度必须为偶数!")
                self.lbl_tx_len.setStyleSheet("color: red;")
                self.btn_send.setEnabled(False)
            else:
                self.lbl_tx_len.setText(f"解析成功: {len(payload)} B")
                self.lbl_tx_len.setStyleSheet("color: green;")
                self.btn_send.setEnabled(True)

    def send_data(self):
        text = self.tx_input.toPlainText()
        if not text: return
        payload = text.encode('utf-8') if self.rb_tx_ascii.isChecked() else self._parse_friendly_hex(text)
        if not payload: return
        
        # 【核心优化】：明确赋予底层引擎 priority=2（最低优先级）
        # 确保你在页面上狂点发送时，绝对不会阻塞后台正跑得火热的 PIL 或 Tunable 任务
        self.hermes.send_raw(payload, priority=2)

    def on_bus_event(self, ev: dict):
        if ev['dir'] == 'RX': self.rx_total_bytes += len(ev['data'])
        else: self.tx_total_bytes += len(ev['data'])
        
        # 智能合并连续同向 RAW 字节流 (终端级效果的核心)
        if self.history and self.history[-1]['type'] == 'RAW' and ev['type'] == 'RAW' and self.history[-1]['dir'] == ev['dir']:
            self.history[-1]['data'] += ev['data']
        else:
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
        
        self.lbl_counters.setText(f"RX: {self.rx_total_bytes} B  |  TX: {self.tx_total_bytes} B")
        
        is_hex = self.rb_rx_hex.isChecked()
        show_raw_rx = self.chk_raw_rx.isChecked()
        show_raw_tx = self.chk_raw_tx.isChecked()
        show_dl = self.chk_dl.isChecked()
        
        html_parts = []
        for ev in self.history:
            if ev['type'] == 'RAW' and ev['dir'] == 'RX' and not show_raw_rx: continue
            if ev['type'] == 'RAW' and ev['dir'] == 'TX' and not show_raw_tx: continue
            if ev['type'] == 'DL' and not show_dl: continue
            
            if is_hex: text_str = ev['data'].hex(' ').upper()
            else:      text_str = html.escape(ev['data'].decode('utf-8', errors='replace')).replace('\n', '<br>')
                
            if ev['type'] == 'RAW':
                # 绿色 和 蓝色
                color = '#00796B' if ev['dir'] == 'RX' else '#1976D2' 
                header = f"[{ev['dir']}: {ev['time']}] >>>"
            else:
                # 深紫 和 橙色 (特殊高亮协议帧)
                color = '#E65100' if ev['dir'] == 'RX' else '#4527A0' 
                crc_str = "OK" if ev['dl_crc_ok'] else f"FAIL ({ev['error']})"
                header = f"[{ev['dir']} DL_FRAME: {ev['time']} | CMD:0x{ev['dl_cmd']:02X} | Index:0x{ev['dl_target']:02X} | CRC:{crc_str}] >>>"
                
            html_parts.append(f"<div style='color:{color}; font-family:Consolas, monospace; margin-bottom:8px; line-height: 1.4;'><b>{header}</b><br>{text_str}</div>")

        self.rx_view.setHtml("".join(html_parts))
        scrollbar = self.rx_view.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())