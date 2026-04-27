import struct
import socket
import threading
import time
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QGroupBox, QLineEdit, 
                             QPushButton, QLabel, QFormLayout, QCheckBox)
from PyQt5.QtCore import pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QPainter, QPen, QColor
from core_datalink import HermesDatalinkQt

# =========================================================
# 现代化浅色系曲线绘制组件
# =========================================================
class TickPlotter(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(140)
        self.history = [0] * 100
        self.max_val = 10

    def add_value(self, val):
        self.history.pop(0)
        self.history.append(val)
        if val > self.max_val:
            self.max_val = val * 1.2 
        elif max(self.history) < self.max_val * 0.5 and self.max_val > 10:
            self.max_val *= 0.9
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        
        painter.fillRect(self.rect(), QColor("#FFFFFF"))
        
        painter.setPen(QPen(QColor("#E0E0E0"), 1, Qt.SolidLine))
        painter.drawRect(0, 0, w-1, h-1)
        painter.setPen(QPen(QColor("#EEEEEE"), 1, Qt.DashLine))
        painter.drawLine(0, int(h * 0.25), w, int(h * 0.25))
        painter.drawLine(0, int(h * 0.5), w, int(h * 0.5))
        painter.drawLine(0, int(h * 0.75), w, int(h * 0.75))

        if not self.history: return

        non_zeros = [v for v in self.history if v > 0]
        current = self.history[-1]
        avg = sum(non_zeros) / len(non_zeros) if non_zeros else 0
        max_v = max(self.history)
        min_v = min(non_zeros) if non_zeros else 0

        if self.max_val > 0 and avg > 0:
            y_avg = h - (avg / self.max_val * h)
            painter.setPen(QPen(QColor("#FF9800"), 1.5, Qt.DashLine))
            painter.drawLine(0, int(y_avg), w, int(y_avg))
            painter.setPen(QColor("#FF9800"))
            painter.drawText(w - 80, int(y_avg) - 5, f"Avg: {avg:.1f} Hz")

        pen = QPen(QColor("#1976D2"), 2)
        painter.setPen(pen)
        dx = w / (len(self.history) - 1)

        for i in range(len(self.history) - 1):
            x1 = i * dx
            y1 = h - (self.history[i] / self.max_val * h) if self.max_val > 0 else h
            x2 = (i + 1) * dx
            y2 = h - (self.history[i+1] / self.max_val * h) if self.max_val > 0 else h
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))
            
        painter.setPen(QColor("#424242"))
        font = painter.font()
        font.setPointSize(9)
        font.setBold(True)
        painter.setFont(font)
        text = f" Cur: {current} Hz\n Max: {max_v} Hz\n Min: {min_v} Hz"
        painter.drawText(10, 10, 150, 100, Qt.AlignTop | Qt.AlignLeft, text)

# =========================================================
# PIL 桥接页面主控
# =========================================================
class TabPilBridge(QWidget):
    sig_rx_parsed = pyqtSignal(dict)

    def __init__(self, hermes: HermesDatalinkQt, tab_sim):
        super().__init__()
        self.hermes = hermes
        self.tab_sim = tab_sim 
        
        self.running = False
        self.sock = None
        self.thread = None
        
        self.CMD_BASE = 0x10
        self.CMD_STEP = self.CMD_BASE + 2
        self.MATLAB_RX_SIZE = 264
        self.MATLAB_TX_SIZE = 200

        self.last_step_payload = None
        self.last_step_time = 0
        self.is_waiting_ack = False
        self.last_matlab_rx_time = 0
        self.matlab_timeout_flag = False
        
        self.total_rx_bytes = 0
        self.total_tx_bytes = 0
        self.last_rx_bytes = 0
        self.last_tx_bytes = 0
        self.tick_counter = 0
        
        self.stat_rx_pkts = 0       
        self.stat_retransmits = 0   
        
        # 【新增】UI 刷新节流阀：控制向 GUI 发信号的频率
        self.last_ui_emit_time = 0  
        
        self.watchdog_timer = QTimer()
        self.watchdog_timer.timeout.connect(self._check_timeouts)
        self.watchdog_timer.start(50) 
        
        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self._update_stats)
        self.stats_timer.start(1000)

        self._setup_ui()
        self.hermes.sig_bus_event.connect(self.on_serial_rx)

        self.is_bus_preempted = False 

    def set_bus_preempted(self, state: bool):
        self.is_bus_preempted = state

    def _setup_ui(self):
        layout = QVBoxLayout(self)

        net_group = QGroupBox("MATLAB / UDP 节点与双重容错配置")
        net_layout = QFormLayout()
        
        self.edit_ip = QLineEdit("127.0.0.1")
        self.edit_recv_port = QLineEdit("12501")
        self.edit_trans_port = QLineEdit("12500") 
        self.edit_step_size = QLineEdit("0.0001")
        self.edit_mcu_timeout = QLineEdit("0.2") 
        self.edit_matlab_timeout = QLineEdit("5.0") 
        
        net_layout.addRow("MATLAB 目标 IP:", self.edit_ip)
        net_layout.addRow("本地监听端口 (MATLAB TX):", self.edit_recv_port)
        net_layout.addRow("目标发送端口 (MATLAB RX):", self.edit_trans_port)
        net_layout.addRow("仿真步长 (Step Size, s):", self.edit_step_size)
        net_layout.addRow("MCU 重传触发阈值 (s):", self.edit_mcu_timeout)
        net_layout.addRow("MATLAB UI挂起警告阈值 (s):", self.edit_matlab_timeout)
        
        info_label = QLabel("ℹ️ 提示：启动前请确保 3 号页面的 MASK 已同步，启动后将自动接管其界面。")
        info_label.setStyleSheet("color: #1565C0; font-style: italic;")
        net_layout.addRow("", info_label)
        
        self.cb_test_mode = QCheckBox("开启 UDP 测试模式 (旁路串口直接回环，不检查 Mask 锁定)")
        self.cb_test_mode.setStyleSheet("color: #E65100; font-weight: bold;")
        net_layout.addRow("", self.cb_test_mode)
        
        net_group.setLayout(net_layout)
        layout.addWidget(net_group)

        stats_group = QGroupBox("在环实时监控指标")
        stats_layout = QVBoxLayout()
        
        self.lbl_traffic = QLabel("流量: RX 0.0 kB (0.0 kB/s) | TX 0.0 kB (0.0 kB/s)")
        self.lbl_pkts = QLabel("质量: 收到 0 包 | 重传 0 包 | 丢包率 0.00%")
        
        self.lbl_traffic.setStyleSheet("font-weight: bold; color: #283593;")
        self.lbl_pkts.setStyleSheet("font-weight: bold; color: #D32F2F;")
        
        stats_layout.addWidget(self.lbl_traffic)
        stats_layout.addWidget(self.lbl_pkts)
        
        self.plotter = TickPlotter()
        stats_layout.addWidget(self.plotter)
        
        stats_group.setLayout(stats_layout)
        layout.addWidget(stats_group)

        self.btn_toggle = QPushButton("🚀 启动 PIL 桥接服务")
        self.btn_toggle.setMinimumHeight(60)
        self.btn_toggle.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.btn_toggle.clicked.connect(self.toggle_bridge)
        layout.addWidget(self.btn_toggle)
        
        layout.addStretch()

    def log(self, msg, color="black"):
        if hasattr(self.hermes, 'sig_log_msg'):
            self.hermes.sig_log_msg.emit(f"<span style='color:{color}; font-weight:bold;'>[桥接器] {msg}</span>")
        else:
            print(msg)
            
    def _set_ui_enabled(self, enabled: bool):
        self.edit_ip.setEnabled(enabled)
        self.edit_recv_port.setEnabled(enabled)
        self.edit_trans_port.setEnabled(enabled)
        self.edit_step_size.setEnabled(enabled)
        self.edit_mcu_timeout.setEnabled(enabled)
        self.edit_matlab_timeout.setEnabled(enabled)
        self.cb_test_mode.setEnabled(enabled)

    def toggle_bridge(self):
        if not self.running:
            if not self.cb_test_mode.isChecked() and not self.tab_sim.is_mask_synced:
                self.log("❌ 启动被拒绝：请先在 '3. PIL 在环仿真引擎' 页面同步 Mask！", "red")
                return

            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock.bind(('0.0.0.0', int(self.edit_recv_port.text())))
                self.sock.settimeout(0.5)
                
                self.target_addr = (self.edit_ip.text(), int(self.edit_trans_port.text()))
                self.step_size = float(self.edit_step_size.text())
                self.mcu_timeout_th = float(self.edit_mcu_timeout.text())
                self.matlab_timeout_th = float(self.edit_matlab_timeout.text())
                
                self.last_matlab_rx_time = 0
                self.matlab_timeout_flag = False
                
                self.running = True
                self.thread = threading.Thread(target=self._bridge_worker, daemon=True)
                self.thread.start()
                
                if not self.cb_test_mode.isChecked():
                    self.tab_sim.set_action_buttons_enabled(False)
                    self.log("🔒 已接管并锁定 3 号页面的手动控制按钮", "orange")
                self._set_ui_enabled(False) 
                
                self.btn_toggle.setText("🛑 停止 PIL 桥接服务")
                self.btn_toggle.setStyleSheet("background-color: #FFEBEE; color: #D32F2F; font-weight: bold;")
                self.log(f"服务已启动，监听 UDP 端口: {self.edit_recv_port.text()}", "green")
                
                dummy_tx = bytes(self.MATLAB_TX_SIZE)
                self.sock.sendto(dummy_tx, self.target_addr)
                self.total_tx_bytes += self.MATLAB_TX_SIZE
                self.log("已发出标准的初始空包，准备迎接同步信号", "blue")
                
            except Exception as e:
                self.log(f"启动错误: {str(e)}", "red")
        else:
            self.running = False
            if self.thread: self.thread.join()
            if self.sock: self.sock.close()
            
            self.tab_sim.set_action_buttons_enabled(True)
            self._set_ui_enabled(True)
            self.log("🔓 已归还 3 号页面的手动控制权限", "green")
            
            self.btn_toggle.setText("🚀 启动 PIL 桥接服务")
            self.btn_toggle.setStyleSheet("")
            self.log("服务已正常关闭", "gray")

    def _update_stats(self):
        if not self.running: return
        rx_speed = (self.total_rx_bytes - self.last_rx_bytes) / 1024.0
        tx_speed = (self.total_tx_bytes - self.last_tx_bytes) / 1024.0
        
        self.lbl_traffic.setText(f"流量: 收到(MATLAB) {self.total_rx_bytes/1024:.1f} kB ({rx_speed:.1f} kB/s)  |  "
                                 f"发给(MATLAB) {self.total_tx_bytes/1024:.1f} kB ({tx_speed:.1f} kB/s)")
        
        loss_rate = 0.0
        if (self.stat_rx_pkts + self.stat_retransmits) > 0:
            loss_rate = self.stat_retransmits / (self.stat_rx_pkts + self.stat_retransmits) * 100.0
            
        self.lbl_pkts.setText(f"质量: 解析通过 {self.stat_rx_pkts} 包  |  "
                              f"超时重传 {self.stat_retransmits} 次  |  "
                              f"丢包率 {loss_rate:.2f}%")
        
        self.plotter.add_value(self.tick_counter)
        
        self.last_rx_bytes = self.total_rx_bytes
        self.last_tx_bytes = self.total_tx_bytes
        self.tick_counter = 0

    def _check_timeouts(self):
        if not self.running: return
        now = time.time()

        if self.is_bus_preempted and self.is_waiting_ack:
            self.last_step_time = now 
            return

        if self.is_waiting_ack and self.last_step_payload:
            elapsed_mcu = now - self.last_step_time
            if elapsed_mcu > self.mcu_timeout_th:
                self.stat_retransmits += 1 
                self.last_step_time = now
                if self.hermes.running:
                    self.hermes.send_frame(0x01, self.CMD_STEP, self.last_step_payload)
                    self.log(f"⚠️ 串口应答超时，触发自动重传", "#FF9800")

        if self.last_matlab_rx_time > 0:
            elapsed_matlab = now - self.last_matlab_rx_time
            if elapsed_matlab > self.matlab_timeout_th and not self.matlab_timeout_flag:
                self.matlab_timeout_flag = True
                self.log(f"⏸️ MATLAB 超过 {self.matlab_timeout_th}s 未响应，可能在 UI 拖拽...", "red")
            elif elapsed_matlab <= self.matlab_timeout_th and self.matlab_timeout_flag:
                self.matlab_timeout_flag = False
                self.log(f"▶️ MATLAB 恢复，重新开始接收仿真推进信号", "green")

    def _bridge_worker(self):
        fmt = '<d24I16d8i'
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                
                if len(data) == self.MATLAB_RX_SIZE:
                    self.total_rx_bytes += len(data)
                    self.tick_counter += 1
                    self.stat_rx_pkts += 1
                    current_time = time.time()
                    self.last_matlab_rx_time = current_time
                    
                    unpacked = struct.unpack(fmt, data)
                    isr_ticks = int(unpacked[0] / self.step_size)
                    digital_in = int(unpacked[41])
                    adc_list = [int(v) & 0xFFFF for v in unpacked[1:25]]
                    panel_list = [float(v) for v in unpacked[25:33]]
                    
                    current_mask_rx = self.tab_sim.current_mask_rx
                    
                    serial_pld = bytearray()
                    serial_pld.extend(struct.pack('<II', isr_ticks, digital_in))
                    for i in range(24):
                        if (current_mask_rx >> i) & 1:
                            serial_pld.extend(struct.pack('<H', adc_list[i]))
                    for i in range(8):
                        if (current_mask_rx >> (24 + i)) & 1:
                            serial_pld.extend(struct.pack('<f', panel_list[i]))
                    
                    self.last_step_payload = bytes(serial_pld)
                    self.last_step_time = current_time
                    self.is_waiting_ack = True
                    
                    # 【核心优化】：GUI 节流阀，最大 20fps 刷新 3 号监视窗口，防止 GUI 卡死
                    if current_time - self.last_ui_emit_time > 0.05:
                        self.sig_rx_parsed.emit({
                            'isr_ticks': isr_ticks, 'dig_in': digital_in,
                            'adc': adc_list, 'panel': panel_list
                        })
                        self.last_ui_emit_time = current_time
                    
                    # 仲裁锁：使用 time.sleep(0) 释放 CPU，提高抢占恢复速度
                    while self.is_bus_preempted and self.running:
                        time.sleep(0) 
                    
                    if self.hermes.running:
                        # 【核心修复】：移除了多余的重复发送指令
                        self.hermes.send_frame(0x01, self.CMD_STEP, self.last_step_payload)
                        
                    if self.cb_test_mode.isChecked():
                        dummy_tx = bytes(self.MATLAB_TX_SIZE)
                        self.sock.sendto(dummy_tx, self.target_addr)
                        self.total_tx_bytes += len(dummy_tx)
                        self.is_waiting_ack = False 
                        
            except ConnectionResetError:
                continue
            except socket.timeout: 
                continue
            except OSError as e:
                if hasattr(e, 'winerror') and e.winerror == 10054:
                    continue
                if self.running: self.log(f"网络底层错误: {e}", "red")

    def on_serial_rx(self, ev: dict):
        if not self.running or ev['type'] != 'DL' or ev['dir'] != 'RX' or not ev['dl_crc_ok']: return
        
        if ev['dl_cmd'] == self.CMD_STEP:
            self.is_waiting_ack = False
            
            payload = ev['dl_payload']
            if len(payload) < 4 or self.cb_test_mode.isChecked(): return
            
            try:
                current_mask_tx = self.tab_sim.current_mask_tx
                
                dig_out = struct.unpack_from('<I', payload, 0)[0]
                idx = 4
                
                pwm_full = [0] * 8
                dac_full = [0] * 8
                mon_full = [0.0] * 16
                
                for i in range(8):
                    if (current_mask_tx >> i) & 1:
                        pwm_full[i] = struct.unpack_from('<H', payload, idx)[0]
                        idx += 2
                for i in range(8):
                    if (current_mask_tx >> (8 + i)) & 1:
                        dac_full[i] = struct.unpack_from('<H', payload, idx)[0]
                        idx += 2
                for i in range(16):
                    if (current_mask_tx >> (16 + i)) & 1:
                        mon_full[i] = struct.unpack_from('<f', payload, idx)[0]
                        idx += 4

                tx_fmt = '<d8I8I16d'
                udp_data = struct.pack(tx_fmt, float(dig_out), *pwm_full, *dac_full, *mon_full)
                
                self.sock.sendto(udp_data, self.target_addr)
                self.total_tx_bytes += len(udp_data)
                
            except struct.error:
                self.log(f"串口解包越界，请确保已在 '3. PIL 页面' 将 Mask 同步至下位机！", "red")
            except Exception as e:
                self.log(f"数据回传格式异常: {e}", "red")