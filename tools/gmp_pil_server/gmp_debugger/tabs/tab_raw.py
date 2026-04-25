import re
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTextBrowser, 
                             QPlainTextEdit, QPushButton, QLabel, QRadioButton, QButtonGroup)
from PyQt5.QtCore import Qt
from core_datalink import HermesDatalinkQt

class TabRaw(QWidget):
    def __init__(self, hermes: HermesDatalinkQt):
        super().__init__()
        self.hermes = hermes
        # 订阅纯粹的原始字节流
        self.hermes.sig_raw_rx.connect(self.on_raw_received)
        
        # 数据流履历，格式: [{'dir': 'RX'/'TX', 'data': bytes}]
        self.history = [] 
        self.rx_total_bytes = 0
        self.tx_total_bytes = 0
        
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        # --- 接收控制栏 ---
        rx_ctrl_layout = QHBoxLayout()
        rx_ctrl_layout.addWidget(QLabel("<b>接收数据记录:</b>"))
        
        self.rb_rx_ascii = QRadioButton("ASCII")
        self.rb_rx_hex = QRadioButton("HEX")
        self.rb_rx_ascii.setChecked(True)
        self.rb_rx_ascii.toggled.connect(self.refresh_display)
        
        rx_ctrl_layout.addSpacing(20)
        rx_ctrl_layout.addWidget(QLabel("视图:"))
        rx_ctrl_layout.addWidget(self.rb_rx_ascii)
        rx_ctrl_layout.addWidget(self.rb_rx_hex)
        rx_ctrl_layout.addStretch()
        
        self.lbl_counters = QLabel("RX: 0 Bytes  |  TX: 0 Bytes")
        self.lbl_counters.setStyleSheet("color: blue; font-weight: bold;")
        rx_ctrl_layout.addWidget(self.lbl_counters)
        
        self.btn_clear_rx = QPushButton("清空记录")
        self.btn_clear_rx.clicked.connect(self.clear_history)
        rx_ctrl_layout.addWidget(self.btn_clear_rx)
        
        layout.addLayout(rx_ctrl_layout)
        
        # --- 接收显示区 ---
        self.rx_view = QTextBrowser()
        self.rx_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.rx_view.setStyleSheet("background-color: #f8f9fa; font-family: Consolas;")
        layout.addWidget(self.rx_view, stretch=3)
        
        # --- 发送控制栏 ---
        tx_ctrl_layout = QHBoxLayout()
        self.rb_tx_ascii = QRadioButton("发送 ASCII")
        self.rb_tx_hex = QRadioButton("发送 HEX (自动过滤非法字符)")
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
        self.tx_input.setPlaceholderText("支持多行发送...\nHEX 模式下可随意输入如: 0x1A, 2b 3C\\n 4d")
        self.tx_input.setMaximumHeight(120)
        self.tx_input.textChanged.connect(self.update_tx_status)
        
        self.btn_send = QPushButton("发送数据\n(RAW)")
        self.btn_send.setMinimumHeight(80)
        self.btn_send.setMinimumWidth(120)
        self.btn_send.setStyleSheet("background-color: #e3f2fd; font-weight: bold;")
        self.btn_send.clicked.connect(self.send_data)
        
        tx_input_layout.addWidget(self.tx_input)
        tx_input_layout.addWidget(self.btn_send)
        layout.addLayout(tx_input_layout, stretch=1)

    def _parse_friendly_hex(self, text: str) -> bytes:
        """极其宽容的 HEX 解析器"""
        # 1. 使用正则剔除所有 '0x', '0X', 以及非 16 进制字符
        clean_str = re.sub(r'(0[xX])|[^0-9a-fA-F]', '', text)
        if not clean_str: return b''
        # 2. 检查奇偶，给出提示而不是闪退
        if len(clean_str) % 2 != 0:
            return None
        return bytes.fromhex(clean_str)

    def update_tx_status(self):
        text = self.tx_input.toPlainText()
        if not text:
            self.lbl_tx_len.setText("0 Bytes")
            self.btn_send.setEnabled(True)
            return

        if self.rb_tx_ascii.isChecked():
            payload = text.encode('utf-8')
            self.lbl_tx_len.setText(f"{len(payload)} Bytes")
            self.lbl_tx_len.setStyleSheet("color: green;")
            self.btn_send.setEnabled(True)
        else:
            payload = self._parse_friendly_hex(text)
            if payload is None:
                self.lbl_tx_len.setText("HEX 长度必须为偶数!")
                self.lbl_tx_len.setStyleSheet("color: red;")
                self.btn_send.setEnabled(False)
            else:
                self.lbl_tx_len.setText(f"解析成功: {len(payload)} Bytes")
                self.lbl_tx_len.setStyleSheet("color: green;")
                self.btn_send.setEnabled(True)

    def send_data(self):
        text = self.tx_input.toPlainText()
        if not text: return
        
        payload = text.encode('utf-8') if self.rb_tx_ascii.isChecked() else self._parse_friendly_hex(text)
        if not payload: return
        
        self.hermes.send_raw(payload)
        self.tx_total_bytes += len(payload)
        
        self.history.append({'dir': 'TX', 'data': payload})
        self._limit_history()
        self.refresh_display()

    def on_raw_received(self, data: bytes):
        self.rx_total_bytes += len(data)
        self.history.append({'dir': 'RX', 'data': data})
        self._limit_history()
        self.refresh_display()

    def _limit_history(self):
        # 限制最大保存 500 条交互，防止内存/渲染卡顿
        if len(self.history) > 500:
            self.history = self.history[-500:]

    def clear_history(self):
        self.history.clear()
        self.rx_total_bytes = 0
        self.tx_total_bytes = 0
        self.refresh_display()

    def refresh_display(self):
        """【核心】根据当前的 Radio 按钮彻底重新渲染所有履历"""
        self.lbl_counters.setText(f"RX: {self.rx_total_bytes} Bytes  |  TX: {self.tx_total_bytes} Bytes")
        
        is_hex = self.rb_rx_hex.isChecked()
        html = ""
        
        for item in self.history:
            raw_bytes = item['data']
            
            if is_hex:
                text_str = raw_bytes.hex(' ').upper()
            else:
                text_str = raw_bytes.decode('utf-8', errors='replace').replace('\n', '<br>')
            
            if item['dir'] == 'TX':
                html += f"<span style='color:blue;'>[TX] {text_str}</span><br>"
            else:
                html += f"<span style='color:#00796b;'>[RX] {text_str}</span><br>"
                
        self.rx_view.setHtml(html)
        # 滚动到底部
        scrollbar = self.rx_view.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())