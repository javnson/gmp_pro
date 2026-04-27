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
        # 统一使用总线事件，只过滤 DL 帧
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
        
        # --- RX 控制区 ---
        rx_ctrl_layout = QHBoxLayout()
        rx_ctrl_layout.addWidget(QLabel("<b>协议 Payload 记录:</b>"))
        
        self.rb_rx_ascii = QRadioButton("ASCII")
        self.rb_rx_hex = QRadioButton("HEX")
        self.rb_rx_ascii.setChecked(True)
        self.rb_rx_ascii.toggled.connect(self.request_render)
        
        rx_ctrl_layout.addSpacing(20)
        rx_ctrl_layout.addWidget(QLabel("视图:"))
        rx_ctrl_layout.addWidget(self.rb_rx_ascii)
        rx_ctrl_layout.addWidget(self.rb_rx_hex)
        rx_ctrl_layout.addStretch()
        
        self.lbl_counters = QLabel("RX Payload: 0 B  |  TX Payload: 0 B")
        self.lbl_counters.setStyleSheet("color: blue; font-weight: bold;")
        rx_ctrl_layout.addWidget(self.lbl_counters)
        
        self.btn_clear_rx = QPushButton("清空")
        self.btn_clear_rx.clicked.connect(self.clear_history)
        rx_ctrl_layout.addWidget(self.btn_clear_rx)
        layout.addLayout(rx_ctrl_layout)
        
        # --- RX 显示区 ---
        self.rx_view = QTextBrowser()
        self.rx_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.rx_view.setStyleSheet("background-color: #FAFAFA; font-size: 13px;")
        layout.addWidget(self.rx_view, stretch=3)
        
        # --- TX 控制与选项区 ---
        tx_ctrl_layout = QHBoxLayout()
        self.rb_tx_ascii = QRadioButton("Payload 填 ASCII")
        self.rb_tx_hex = QRadioButton("Payload 填 HEX")
        self.rb_tx_ascii.setChecked(True)
        self.rb_tx_ascii.toggled.connect(self.update_tx_status)
        self.rb_tx_hex.toggled.connect(self.update_tx_status)
        
        tx_ctrl_layout.addWidget(self.rb_tx_ascii)
        tx_ctrl_layout.addWidget(self.rb_tx_hex)
        tx_ctrl_layout.addSpacing(20)
        
        # 【新增】Seq/ID 输入框
        tx_ctrl_layout.addWidget(QLabel("Seq/ID:"))
        self.tx_id_input = QLineEdit("0x01")
        self.tx_id_input.setMaximumWidth(50)
        tx_ctrl_layout.addWidget(self.tx_id_input)

        tx_ctrl_layout.addSpacing(10)

        # CMD 输入框
        tx_ctrl_layout.addWidget(QLabel("CMD:"))
        self.tx_cmd_input = QLineEdit("0x00") # 默认 0x00 方便测试 ECHO
        self.tx_cmd_input.setMaximumWidth(50)
        tx_ctrl_layout.addWidget(self.tx_cmd_input)
        
        tx_ctrl_layout.addStretch()
        
        self.lbl_tx_len = QLabel("Ready")
        tx_ctrl_layout.addWidget(self.lbl_tx_len)
        layout.addLayout(tx_ctrl_layout)
        
        # --- TX 输入与发送区 ---
        tx_input_layout = QHBoxLayout()
        self.tx_input = QPlainTextEdit()
        self.tx_input.setPlaceholderText("输入要装入 Payload 的数据...")
        self.tx_input.setMaximumHeight(100)
        self.tx_input.textChanged.connect(self.update_tx_status)
        
        self.btn_send = QPushButton("封包发送\n(加入协议头尾)")
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
                self.lbl_tx_len.setText("HEX 不完整")
                self.lbl_tx_len.setStyleSheet("color: red;")
                self.btn_send.setEnabled(False)
            else:
                length = len(payload)
                self.lbl_tx_len.setText(f"{length} B")
                self.lbl_tx_len.setStyleSheet("color: green;" if length <= 256 else "color: red;")
                self.btn_send.setEnabled(length <= 256)

    def send_data(self):
        text = self.tx_input.toPlainText()
        
        # 允许发送空包（比如纯命令或 ACK）
        if not text:
            payload = b''
        else:
            payload = text.encode('utf-8') if self.rb_tx_ascii.isChecked() else self._parse_friendly_hex(text)
            if payload is None: return
        
        # 安全获取用户输入的自定义 ID (Seq)
        try:
            seq_id = int(self.tx_id_input.text(), 16)
        except ValueError:
            seq_id = 0x01
            self.tx_id_input.setText("0x01")

        # 安全获取用户输入的自定义 CMD
        try:
            cmd = int(self.tx_cmd_input.text(), 16)
        except ValueError:
            cmd = 0x00
            self.tx_cmd_input.setText("0x00")
            
        self.hermes.send_frame(target_id=seq_id, cmd=cmd, payload=payload)

    def on_bus_event(self, ev: dict):
        # DL 页面只关心 DL 帧
        if ev['type'] != 'DL': return
        
        # 仅统计 Payload 字节
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
                color = '#4527A0' # 深紫
                header = f"[{ev['dir']}: {ev['time']} | Payload -> Seq:0x{ev['dl_target']:02X} CMD:0x{ev['dl_cmd']:02X}] >>>"
            else:
                crc_str = "OK" if ev['dl_crc_ok'] else "FAIL"
                
                # 【核心修改】单独识别并高亮 NACK 消息 (CMD == 0x01)
                if ev['dl_cmd'] == 0x01:
                    color = '#E91E63' # 醒目的粉红色/紫红色
                    header = f"[{ev['dir']}: {ev['time']} | Payload <- Seq:0x{ev['dl_target']:02X} CMD:0x{ev['dl_cmd']:02X} (NACK) | CRC:{crc_str}] >>>"
                    
                    # 尝试解析 NACK 的具体内容 (Byte 0: 被拒 CMD, Byte 1: 错误码)
                    if len(payload_bytes) >= 2:
                        rejected_cmd = payload_bytes[0]
                        err_code = payload_bytes[1]
                        nack_notice = f"<b>[系统提示] 收到 NACK！对方拒绝了指令 0x{rejected_cmd:02X} (错误码: 0x{err_code:02X})</b><br>"
                        text_str = nack_notice + text_str
                    else:
                        text_str = "<b>[系统提示] 收到 NACK (未知原因)</b><br>" + text_str
                else:
                    color = '#E65100' if ev['dl_crc_ok'] else '#D32F2F' # 橙色正常，红色报错
                    header = f"[{ev['dir']}: {ev['time']} | Payload <- Seq:0x{ev['dl_target']:02X} CMD:0x{ev['dl_cmd']:02X} | CRC:{crc_str}] >>>"
                
            html_parts.append(f"<div style='color:{color}; font-family:Consolas, monospace; margin-bottom:8px; line-height: 1.4;'><b>{header}</b><br>{text_str}</div>")
                
        self.rx_view.setHtml("".join(html_parts))
        scrollbar = self.rx_view.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())