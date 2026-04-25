from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTextBrowser, 
                             QPlainTextEdit, QPushButton, QLabel, QRadioButton, QButtonGroup)
from PyQt5.QtCore import Qt
from core_datalink import HermesDatalinkQt

CMD_ECHO = 0x99

class TabAscii(QWidget):
    def __init__(self, hermes: HermesDatalinkQt):
        super().__init__()
        self.hermes = hermes
        self.hermes.sig_frame_received.connect(self.on_frame_received)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        # 1. 接收显示区
        self.rx_view = QTextBrowser()
        self.rx_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        layout.addWidget(QLabel("接收数据记录:"))
        layout.addWidget(self.rx_view, stretch=3) # 分配更多空间给显示
        
        # 2. 发送设置栏 (格式切换 & 字节统计)
        ctrl_layout = QHBoxLayout()
        
        self.rb_ascii = QRadioButton("ASCII (UTF-8)")
        self.rb_hex = QRadioButton("HEX 格式")
        self.rb_ascii.setChecked(True)
        
        self.format_group = QButtonGroup()
        self.format_group.addButton(self.rb_ascii)
        self.format_group.addButton(self.rb_hex)
        self.rb_ascii.toggled.connect(self.update_byte_count)
        self.rb_hex.toggled.connect(self.update_byte_count)

        self.lbl_byte_count = QLabel("Payload: 0 Bytes")
        self.lbl_byte_count.setStyleSheet("color: gray; font-weight: bold;")
        
        ctrl_layout.addWidget(self.rb_ascii)
        ctrl_layout.addWidget(self.rb_hex)
        ctrl_layout.addStretch()
        ctrl_layout.addWidget(self.lbl_byte_count)
        
        layout.addLayout(ctrl_layout)
        
        # 3. 发送输入区 (多行文本 + 滚动条)
        tx_layout = QHBoxLayout()
        self.tx_input = QPlainTextEdit()
        self.tx_input.setPlaceholderText("在此输入要发送的文本或 HEX 数据(以空格隔开, 如: 1A 2B 3C)...")
        self.tx_input.setMaximumHeight(120) # 限制最大高度
        self.tx_input.textChanged.connect(self.update_byte_count)
        
        self.btn_send = QPushButton("发送\n(CMD:0x99)")
        self.btn_send.setMinimumHeight(80)
        self.btn_send.setMinimumWidth(100)
        self.btn_send.clicked.connect(self.send_text)
        
        tx_layout.addWidget(self.tx_input)
        tx_layout.addWidget(self.btn_send)
        layout.addLayout(tx_layout, stretch=1)

    def _get_payload_bytes(self) -> bytes:
        """根据当前选项解析输入框的内容，如果解析失败返回 None"""
        text = self.tx_input.toPlainText()
        if not text: return b''
        
        if self.rb_ascii.isChecked():
            return text.encode('utf-8')
        else:
            # HEX 模式：过滤掉空格和回车，尝试转换
            hex_str = text.replace(' ', '').replace('\n', '').replace('\r', '')
            try:
                return bytes.fromhex(hex_str)
            except ValueError:
                return None

    def update_byte_count(self):
        """实时更新当前输入框的数据长度"""
        payload = self._get_payload_bytes()
        if payload is None:
            self.lbl_byte_count.setText("Payload: 格式错误!")
            self.lbl_byte_count.setStyleSheet("color: red; font-weight: bold;")
            self.btn_send.setEnabled(False)
        else:
            length = len(payload)
            if length > 256: # 假设咱们单片机的 MTU 是 256
                self.lbl_byte_count.setText(f"Payload: {length} Bytes (超长!)")
                self.lbl_byte_count.setStyleSheet("color: red; font-weight: bold;")
            else:
                self.lbl_byte_count.setText(f"Payload: {length} Bytes")
                self.lbl_byte_count.setStyleSheet("color: green; font-weight: bold;")
                self.btn_send.setEnabled(True)

    def send_text(self):
        payload = self._get_payload_bytes()
        if not payload: 
            return # 空包或格式错误不发
        
        # 调用引擎发送
        self.hermes.send_frame(target_id=0x01, cmd=CMD_ECHO, payload=payload)
        
        # 在界面上留痕
        if self.rb_ascii.isChecked():
            display_text = payload.decode('utf-8', errors='replace')
        else:
            display_text = payload.hex(' ').upper()
            
        self.rx_view.append(f"<font color='blue'>[TX: {len(payload)}B] {display_text}</font>")
        # 发生后可以选择不清空，方便多次测试修改
        # self.tx_input.clear() 

    def on_frame_received(self, target_id: int, cmd: int, payload: bytes):
        if cmd == CMD_ECHO:
            if self.rb_ascii.isChecked():
                try:
                    text = payload.decode('utf-8')
                    self.rx_view.append(f"<font color='green'>[RX 回显: {len(payload)}B] {text}</font>")
                except UnicodeDecodeError:
                    self.rx_view.append(f"<font color='red'>[RX 乱码] {payload.hex(' ').upper()}</font>")
            else:
                # HEX 模式展示
                text = payload.hex(' ').upper()
                self.rx_view.append(f"<font color='#009688'>[RX HEX: {len(payload)}B] {text}</font>")