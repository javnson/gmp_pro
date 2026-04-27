import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QTabWidget, QGroupBox, QComboBox, 
                             QPushButton, QTextBrowser, QLabel, QFormLayout,
                             QProgressBar, QSizePolicy) 
from PyQt5.QtCore import Qt, QTimer

# 导入核心通信引擎
from core_datalink import HermesDatalinkQt

# 导入各个解耦的业务子页面
from tabs.tab_ascii import TabAscii
from tabs.tab_raw import TabRaw
from tabs.tab_sim import TabSim
from tabs.tab_pil import TabPilBridge
from tabs.tab_tunable import TabTunableManager
from tabs.tab_mem_persp import TabMemPersp
from tabs.tab_chronos import TabChronosManager

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
        self.resize(1100, 650)
        
        # =========================================================
        # 1. 实例化全局唯一的通信内核 (Hermes Engine)
        # =========================================================
        self.hermes = HermesDatalinkQt()
        self.hermes.sig_log_msg.connect(self.log_message)
        self.hermes.sig_conn_state.connect(self.update_ui_connection_state) 
        
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

        # =========================================================
        # 2. 页面挂载与依赖注入
        # =========================================================
        self.tab_raw = TabRaw(self.hermes)
        self.tabs.addTab(self.tab_raw, "1. 标准串口调试助手 (RAW)")
        
        self.tab_ascii = TabAscii(self.hermes)
        self.tabs.addTab(self.tab_ascii, "2. GMP DL协议在环测试 (ECHO)")

        self.tab_sim = TabSim(self.hermes)
        self.tabs.addTab(self.tab_sim, "3. PIL 在环仿真引擎")

        self.tab_pil_bridge = TabPilBridge(self.hermes, self.tab_sim)
        self.tabs.addTab(self.tab_pil_bridge, "4. Simulink-PIL 网桥")

        self.tab_tunable = TabTunableManager(self.hermes)
        self.tabs.addTab(self.tab_tunable, "5. 参数在线整定工作台")

        self.tab_mem_persp = TabMemPersp(self.hermes)
        self.tabs.addTab(self.tab_mem_persp, "6. Argos 内存透视分析仪")

        self.tab_chronos = TabChronosManager(self.hermes)
        self.tabs.addTab(self.tab_chronos, "7. Chronos 波形记录仪")

        # =========================================================
        # 3. 跨模块信号互锁连线
        # =========================================================
        self.tab_pil_bridge.sig_rx_parsed.connect(self.tab_sim.update_rx_ui_from_bridge)
        self.tab_tunable.sig_global_bus_busy.connect(self.tab_pil_bridge.set_bus_preempted)
        
        # =========================================================
        # 4. 右侧全局控制与日志面板 (弹性高度布局重构)
        # =========================================================
        right_panel = QVBoxLayout()
        main_layout.addLayout(right_panel, stretch=1)
        
        # 顶部：串口配置 (固定高度)
        self._build_serial_panel(right_panel)
        # 中部：状态监视 (TX/RX 分离，固定高度)
        self._build_stats_panel(right_panel) 
        
        # 底部：系统日志 (无限制高度，吃掉剩余空间)
        self.sys_log = QTextBrowser()
        self.sys_log.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn) 
        
        right_panel.addWidget(QLabel("<b>系统日志:</b>"))
        # 【核心调整】：加入 stretch=1，让日志框弹满剩余所有高度
        right_panel.addWidget(self.sys_log, stretch=1)

        self.update_ui_connection_state(False)

        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self._update_bus_stats)
        self.stats_timer.start(1000)

    # ---------------------------------------------------------
    # 界面构建与基础交互
    # ---------------------------------------------------------
    def _build_serial_panel(self, layout: QVBoxLayout):
        group_box = QGroupBox("串口配置")
        # 锁定组件不随窗口拉伸而变形
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

        form_layout.addRow("串口选择:", port_hlayout)
        form_layout.addRow("波特率:", self.cb_baud)
        form_layout.addRow("数据位:", self.cb_data_bits)
        form_layout.addRow("停止位:", self.cb_stop_bits)
        form_layout.addRow("校验位:", self.cb_parity)
        
        self.btn_connect = QPushButton("打开串口")
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
        """构建 TX/RX 分轨总线监控面板"""
        stats_group = QGroupBox("总线利用率监控 (Bus Load)")
        # 锁定组件高度
        stats_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        vbox = QVBoxLayout()
        vbox.setSpacing(8)
        
        # 1. 速率显示标签
        h_speeds = QHBoxLayout()
        self.lbl_speed_tx = QLabel("TX: 0.0 kB/s")
        self.lbl_speed_rx = QLabel("RX: 0.0 kB/s")
        self.lbl_speed_tx.setStyleSheet("color: #4527A0; font-weight: bold;")
        self.lbl_speed_rx.setStyleSheet("color: #00796B; font-weight: bold;")
        h_speeds.addWidget(self.lbl_speed_tx)
        h_speeds.addWidget(self.lbl_speed_rx)
        vbox.addLayout(h_speeds)
        
        # 2. TX 负载条 (固定较小的高度，美观精炼)
        self.bar_tx = QProgressBar()
        self.bar_tx.setFixedHeight(18)
        self.bar_tx.setRange(0, 100)
        self.bar_tx.setValue(0)
        self.bar_tx.setFormat("TX 占用: %p%")
        self.bar_tx.setStyleSheet(self._get_bar_stylesheet("#E0E0E0", "gray"))
        vbox.addWidget(self.bar_tx)

        # 3. RX 负载条
        self.bar_rx = QProgressBar()
        self.bar_rx.setFixedHeight(18)
        self.bar_rx.setRange(0, 100)
        self.bar_rx.setValue(0)
        self.bar_rx.setFormat("RX 占用: %p%")
        self.bar_rx.setStyleSheet(self._get_bar_stylesheet("#E0E0E0", "gray"))
        vbox.addWidget(self.bar_rx)
        
        stats_group.setLayout(vbox)
        layout.addWidget(stats_group)

    # ---------------------------------------------------------
    # 串口控制与数据统计核心逻辑
    # ---------------------------------------------------------
    def _on_bus_event_for_stats(self, ev: dict):
        if ev['dir'] == 'TX':
            self.total_tx_bytes += len(ev['data'])
        else:
            self.total_rx_bytes += len(ev['data'])

    def _get_bar_stylesheet(self, color: str, text_color: str = "black") -> str:
        """辅助函数：动态生成 ProgressBar 样式"""
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
        """1Hz 触发，分别计算 TX 和 RX 的通信速率和负载率"""
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

        # 串口物理带宽 (1 Byte = 10 bits: 1起始 + 8数据 + 1停止)
        try:
            baud_rate = int(self.cb_baud.currentText())
            max_bytes_per_sec = baud_rate / 10.0
            
            tx_util = (tx_diff / max_bytes_per_sec) * 100.0 if max_bytes_per_sec > 0 else 0.0
            rx_util = (rx_diff / max_bytes_per_sec) * 100.0 if max_bytes_per_sec > 0 else 0.0
        except ValueError:
            tx_util, rx_util = 0.0, 0.0

        # 限定到 0-100
        tx_int = int(min(100, max(0, tx_util)))
        rx_int = int(min(100, max(0, rx_util)))

        self.bar_tx.setValue(tx_int)
        self.bar_rx.setValue(rx_int)

        # TX 预警颜色 (深紫主色调)
        tx_color = "#E53935" if tx_int > 80 else ("#FF9800" if tx_int > 50 else "#7E57C2")
        # RX 预警颜色 (青绿主色调)
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
                self.log_message("⚠️ 请先选择一个有效的串口")
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
            self.btn_connect.setText("关闭串口")
            self.btn_connect.setStyleSheet("""
                QPushButton { border-left: 6px solid #4CAF50; background-color: #E8F5E9; border-radius: 3px; }
                QPushButton:hover { background-color: #C8E6C9; }
            """)
            self._set_combos_enabled(False)
        else:
            self.btn_connect.setText("打开串口")
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

    def log_message(self, msg: str):
        self.sys_log.append(msg)
        scrollbar = self.sys_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def closeEvent(self, event):
        self.log_message("⏳ 正在安全关闭通信内核引擎...")
        
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