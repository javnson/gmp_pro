import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QTabWidget, QGroupBox, QComboBox, 
                             QPushButton, QTextBrowser, QLabel, QFormLayout)
from PyQt5.QtCore import Qt

from core_datalink import HermesDatalinkQt
from tabs.tab_ascii import TabAscii

DATA_BITS_MAP = {'8': serial.EIGHTBITS, '7': serial.SEVENBITS, '6': serial.SIXBITS, '5': serial.FIVEBITS}
STOP_BITS_MAP = {'1': serial.STOPBITS_ONE, '1.5': serial.STOPBITS_ONE_POINT_FIVE, '2': serial.STOPBITS_TWO}
PARITY_MAP = {
    'None': serial.PARITY_NONE, 'Even': serial.PARITY_EVEN, 
    'Odd': serial.PARITY_ODD, 'Mark': serial.PARITY_MARK, 'Space': serial.PARITY_SPACE
}

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GMP Datalink Debugger / PIL Server")
        self.resize(1100, 650) # 稍微加宽一点窗口
        
        self.hermes = HermesDatalinkQt()
        self.hermes.sig_log_msg.connect(self.log_message)
        # 【新增】订阅连接状态的改变
        self.hermes.sig_conn_state.connect(self.update_ui_connection_state) 
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs, stretch=4)
        
        self.tab_ascii = TabAscii(self.hermes)
        self.tabs.addTab(self.tab_ascii, "综合收发测试区")
        
        right_panel = QVBoxLayout()
        main_layout.addLayout(right_panel, stretch=1)
        
        self._build_serial_panel(right_panel)
        
        self.sys_log = QTextBrowser()
        self.sys_log.setMaximumHeight(200)
        # 【新增】强制显示系统日志的滚动条
        self.sys_log.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn) 
        right_panel.addWidget(QLabel("系统日志:"))
        right_panel.addWidget(self.sys_log)

        # 初始状态设置（红条断开状态）
        self.update_ui_connection_state(False)

    def _build_serial_panel(self, layout: QVBoxLayout):
        group_box = QGroupBox("串口配置")
        form_layout = QFormLayout()
        form_layout.setLabelAlignment(Qt.AlignRight)
        
        port_hlayout = QHBoxLayout()
        self.cb_ports = QComboBox()
        self.cb_ports.setMinimumWidth(150)
        # 【新增】关键：让下拉列表弹出时可以突破框本身的宽度，显示超长设备名！
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

        form_layout.addRow("串口选择:", port_hlayout)
        form_layout.addRow("波特率:", self.cb_baud)
        form_layout.addRow("数据位:", self.cb_data_bits)
        form_layout.addRow("停止位:", self.cb_stop_bits)
        form_layout.addRow("校验位:", self.cb_parity)
        
        self.btn_connect = QPushButton("打开串口")
        self.btn_connect.setMinimumHeight(45)
        # 字体加粗
        font = self.btn_connect.font()
        font.setBold(True)
        self.btn_connect.setFont(font)
        self.btn_connect.clicked.connect(self.toggle_connection)
        
        vbox = QVBoxLayout()
        vbox.addLayout(form_layout)
        vbox.addSpacing(15)
        vbox.addWidget(self.btn_connect)
        vbox.addStretch()
        
        group_box.setLayout(vbox)
        layout.addWidget(group_box)
        
        self.refresh_ports()

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
                self.log_message("⚠️ 请先选择一个有效的串口")
                return
            baud = int(self.cb_baud.currentText())
            data_bits = DATA_BITS_MAP[self.cb_data_bits.currentText()]
            stop_bits = STOP_BITS_MAP[self.cb_stop_bits.currentText()]
            parity = PARITY_MAP[self.cb_parity.currentText()]
            
            # connect_serial 成功后会自动发射 sig_conn_state(True)
            self.hermes.connect_serial(port, baud, data_bits, parity, stop_bits)
        else:
            # close 会自动发射 sig_conn_state(False)
            self.hermes.close()

    def update_ui_connection_state(self, is_connected: bool):
        """【新增】统一管理连接状态 UI，防止状态不一致"""
        if is_connected:
            self.btn_connect.setText("关闭串口")
            # 绿色左边框指示器，浅绿背景
            self.btn_connect.setStyleSheet("""
                QPushButton {
                    border-left: 6px solid #4CAF50;
                    background-color: #E8F5E9;
                    border-radius: 3px;
                }
                QPushButton:hover { background-color: #C8E6C9; }
            """)
            self._set_combos_enabled(False)
        else:
            self.btn_connect.setText("打开串口")
            # 红色左边框指示器，标准背景
            self.btn_connect.setStyleSheet("""
                QPushButton {
                    border-left: 6px solid #F44336;
                    background-color: #FAFAFA;
                    border-radius: 3px;
                }
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

    def log_message(self, msg: str):
        self.sys_log.append(msg)
        scrollbar = self.sys_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

if __name__ == "__main__":
    app = QApplication(sys.argv)
    # 给整体应用换个顺眼的风格
    app.setStyle("Fusion") 
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())