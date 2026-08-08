import os
import random
import struct
import time
import csv
import json
import re
import numpy as np
from collections import deque
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                             QPushButton, QLabel, QComboBox, QLineEdit, 
                             QScrollArea, QSplitter, QColorDialog, QFrame, 
                             QInputDialog, QMessageBox, QTabWidget, QCheckBox, QFileDialog)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor

import pyqtgraph as pg
from core_datalink import HermesDatalinkQt

# 全局 pyqtgraph 样式配置 (工业风白底黑字)
pg.setConfigOption('background', '#F8F9FA')
pg.setConfigOption('foreground', '#212529')
pg.setConfigOption('antialias', True)

# 支持的类型解析字典
TYPE_FORMATS = {
    'F32': ('<f', 4), 'I32': ('<i', 4), 'U32': ('<I', 4),
    'I16': ('<h', 2), 'U16': ('<H', 2), 'I8': ('<b', 1), 'U8': ('<B', 1)
}

# =========================================================
# 第三层：单个波形通道配置 (Waveform Configuration)
# =========================================================
class WaveformConfigWidget(QFrame):
    sig_removed = pyqtSignal(object)
    sig_name_changed = pyqtSignal()
    sig_target_changed_attempt = pyqtSignal(object)

    def __init__(self, plot_id, wave_id):
        super().__init__()
        self.plot_id = plot_id
        self.wave_id = wave_id
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("QFrame { background-color: #FFFFFF; border-radius: 4px; border: 1px solid #E0E0E0; margin-bottom: 2px; }")
        
        self.color = QColor(random.randint(0, 200), random.randint(0, 200), random.randint(0, 200))
        
        # 轮询状态机参数
        self.last_req_time = 0.0
        self.waiting_for_ack = False 
        
        # 状态备份（用于用户取消修改时回滚）
        self._committed_state = {}
        
        self._setup_ui()
        self.commit_target() # 初始状态锁定

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)

        # 第一行：颜色、名字、删除按钮
        row1 = QHBoxLayout()
        self.btn_color = QPushButton()
        self.btn_color.setFixedSize(14, 14)
        self._update_color_btn()
        self.btn_color.clicked.connect(self._choose_color)
        row1.addWidget(self.btn_color)

        self.edit_name = QLineEdit(f"CH {self.wave_id}")
        self.edit_name.setPlaceholderText("波形名称")
        self.edit_name.setStyleSheet("border: none; font-weight: bold; background: transparent;")
        self.edit_name.editingFinished.connect(self.sig_name_changed.emit)
        row1.addWidget(self.edit_name)
        
        self.btn_del = QPushButton("❌")
        self.btn_del.setFixedSize(20, 20)
        self.btn_del.setStyleSheet("border: none; font-size: 10px;")
        self.btn_del.clicked.connect(lambda: self.sig_removed.emit(self))
        row1.addWidget(self.btn_del)
        layout.addLayout(row1)

        # 第二行：数据源配置
        row2 = QHBoxLayout()
        self.cb_mode = QComboBox()
        self.cb_mode.addItems(["变量字典 (Tunable)", "绝对地址 (Memory)"])
        self.cb_mode.currentIndexChanged.connect(self._on_target_ui_interacted)
        row2.addWidget(self.cb_mode)

        self.edit_target = QLineEdit()
        self.edit_target.setPlaceholderText("ID/Addr(Hex)")
        self.edit_target.setMaximumWidth(80)
        self.edit_target.editingFinished.connect(self._on_target_ui_interacted)
        row2.addWidget(self.edit_target)

        self.cb_type = QComboBox()
        self.cb_type.addItems(list(TYPE_FORMATS.keys()))
        self.cb_type.setCurrentText("F32")
        self.cb_type.setMaximumWidth(60)
        self.cb_type.currentIndexChanged.connect(self._on_target_ui_interacted)
        row2.addWidget(self.cb_type)
        layout.addLayout(row2)
        
        # 第三行：指令与频率 (提升UI饱满度)
        row3 = QHBoxLayout()
        row3.addWidget(QLabel("CMD:"))
        self.edit_cmd = QLineEdit("0x30")
        row3.addWidget(self.edit_cmd, stretch=1)
        
        row3.addWidget(QLabel("频率:"))
        self.cb_rate = QComboBox()
        self.cb_rate.setEditable(True) # 允许手动输入
        self.cb_rate.addItems(["连续实时", "1000 Hz", "500 Hz", "300 Hz", "250 Hz", "200 Hz", "100 Hz", "50 Hz", "20 Hz", "10 Hz", "5 Hz", "1 Hz"])
        row3.addWidget(self.cb_rate, stretch=2)
        layout.addLayout(row3)

        self._update_mode_ui_state()

    def _on_target_ui_interacted(self):
        """核心防丢锁：捕捉配置变更请求"""
        self._update_mode_ui_state()
        self.sig_target_changed_attempt.emit(self)

    def commit_target(self):
        """确认修改，保存当前状态为安全基准"""
        self._committed_state = {
            'mode': self.cb_mode.currentIndex(),
            'target': self.edit_target.text(),
            'type': self.cb_type.currentIndex()
        }

    def revert_target(self):
        """用户取消修改，回滚UI到安全基准，防止误发信号死循环"""
        self.cb_mode.blockSignals(True)
        self.edit_target.blockSignals(True)
        self.cb_type.blockSignals(True)
        
        self.cb_mode.setCurrentIndex(self._committed_state['mode'])
        self.edit_target.setText(self._committed_state['target'])
        self.cb_type.setCurrentIndex(self._committed_state['type'])
        
        self.cb_mode.blockSignals(False)
        self.edit_target.blockSignals(False)
        self.cb_type.blockSignals(False)
        self._update_mode_ui_state()

    def _update_mode_ui_state(self):
        is_mem = self.cb_mode.currentIndex() == 1
        self.cb_type.setEnabled(is_mem)
        # 如果用户未自定义过CMD，自动切
        if self.edit_cmd.text() in ["0x30", "0x50"]:
            self.edit_cmd.setText("0x50" if is_mem else "0x30")

    def _choose_color(self):
        color = QColorDialog.getColor(self.color, self, "选择波形颜色")
        if color.isValid():
            self.color = color
            self._update_color_btn()
            self.sig_name_changed.emit()

    def _update_color_btn(self):
        self.btn_color.setStyleSheet(f"background-color: {self.color.name()}; border: 1px solid #9E9E9E; border-radius: 7px;")

    def get_rate_ms(self):
        txt = self.cb_rate.currentText()
        if "连续" in txt or "实时" in txt: return 0
        
        # 智能正则解析用户输入的任意数字 (提取 Hz 或 ms)
        nums = re.findall(r"[-+]?\d*\.\d+|\d+", txt)
        if nums:
            val = float(nums[0])
            if val <= 0: return 100
            if "ms" in txt.lower(): return val
            return 1000.0 / val # 默认按 Hz 算
        return 100

    def serialize(self):
        return {
            'wave_id': self.wave_id, 'name': self.edit_name.text(),
            'color': self.color.name(), 'mode': self.cb_mode.currentIndex(),
            'target': self.edit_target.text(), 'type': self.cb_type.currentIndex(),
            'cmd': self.edit_cmd.text(), 'rate': self.cb_rate.currentText()
        }

    def deserialize(self, data):
        self.edit_name.setText(data.get('name', f"CH {self.wave_id}"))
        self.color = QColor(data.get('color', '#000000'))
        self._update_color_btn()
        
        self.cb_mode.setCurrentIndex(data.get('mode', 0))
        self.edit_target.setText(data.get('target', ''))
        self.cb_type.setCurrentIndex(data.get('type', 0))
        self.edit_cmd.setText(data.get('cmd', '0x30'))
        self.cb_rate.setCurrentText(data.get('rate', '100 Hz'))
        self.commit_target()


# =========================================================
# 第二层：单个 Plot 窗口配置 (Plot Window Configuration)
# =========================================================
class PlotConfigWidget(QGroupBox):
    sig_plot_removed = pyqtSignal(object)
    sig_structure_changed = pyqtSignal()
    sig_wave_target_changed = pyqtSignal(object)

    def __init__(self, plot_id):
        super().__init__(f"📊 示波器窗口 {plot_id}")
        self.plot_id = plot_id
        self.waveforms = []
        self.wave_counter = 0
        self._setup_ui()

    def _setup_ui(self):
        self.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid #B0BEC5; border-radius: 6px; margin-top: 10px; }")
        self.layout = QVBoxLayout(self)
        
        # 行 1: 标题与操作
        header = QHBoxLayout()
        header.addWidget(QLabel("标题:"))
        self.edit_title = QLineEdit(f"Oscilloscope {self.plot_id}")
        self.edit_title.textChanged.connect(self.sig_structure_changed.emit)
        header.addWidget(self.edit_title)
        
        self.btn_add_wave = QPushButton("➕ 添加波形")
        self.btn_add_wave.clicked.connect(self.add_waveform)
        header.addWidget(self.btn_add_wave)
        
        self.btn_del_plot = QPushButton("🗑️")
        self.btn_del_plot.clicked.connect(lambda: self.sig_plot_removed.emit(self))
        header.addWidget(self.btn_del_plot)
        self.layout.addLayout(header)

        # 行 2: 坐标轴控制面板 (X滑窗 + Y定标)
        axis_ctrl = QHBoxLayout()
        axis_ctrl.setContentsMargins(0, 0, 0, 0)
        
        self.cb_slide_x = QCheckBox("X滑窗(s):")
        self.cb_slide_x.setChecked(True)
        self.edit_slide_x = QLineEdit("5.0")
        self.edit_slide_x.setMaximumWidth(40)
        axis_ctrl.addWidget(self.cb_slide_x)
        axis_ctrl.addWidget(self.edit_slide_x)
        
        axis_ctrl.addStretch()
        self.cb_auto_y = QCheckBox("Y自适应")
        self.cb_auto_y.setChecked(True)
        self.edit_y_min = QLineEdit("-10")
        self.edit_y_min.setMaximumWidth(40)
        self.edit_y_max = QLineEdit("10")
        self.edit_y_max.setMaximumWidth(40)
        axis_ctrl.addWidget(self.cb_auto_y)
        axis_ctrl.addWidget(self.edit_y_min)
        axis_ctrl.addWidget(QLabel("~"))
        axis_ctrl.addWidget(self.edit_y_max)
        self.layout.addLayout(axis_ctrl)

        self.wave_container = QVBoxLayout()
        self.layout.addLayout(self.wave_container)

    def add_waveform(self):
        self.wave_counter += 1
        wave_widget = WaveformConfigWidget(self.plot_id, self.wave_counter)
        wave_widget.sig_removed.connect(self.remove_waveform)
        wave_widget.sig_name_changed.connect(self.sig_structure_changed.emit)
        wave_widget.sig_target_changed_attempt.connect(self.sig_wave_target_changed.emit)
        
        self.waveforms.append(wave_widget)
        self.wave_container.addWidget(wave_widget)
        self.sig_structure_changed.emit()

    def remove_waveform(self, wave_widget):
        self.waveforms.remove(wave_widget)
        self.wave_container.removeWidget(wave_widget)
        wave_widget.deleteLater()
        self.sig_structure_changed.emit()

    def serialize(self):
        return {
            'plot_id': self.plot_id, 'title': self.edit_title.text(),
            'slide_x': self.cb_slide_x.isChecked(), 'slide_x_val': self.edit_slide_x.text(),
            'auto_y': self.cb_auto_y.isChecked(), 'y_min': self.edit_y_min.text(), 'y_max': self.edit_y_max.text(),
            'waves': [w.serialize() for w in self.waveforms]
        }

    def deserialize(self, data):
        self.edit_title.setText(data.get('title', f"Oscilloscope {self.plot_id}"))
        self.cb_slide_x.setChecked(data.get('slide_x', True))
        self.edit_slide_x.setText(data.get('slide_x_val', "5.0"))
        self.cb_auto_y.setChecked(data.get('auto_y', True))
        self.edit_y_min.setText(data.get('y_min', "-10"))
        self.edit_y_max.setText(data.get('y_max', "10"))
        
        for w_data in data.get('waves', []):
            self.wave_counter += 1
            wave_widget = WaveformConfigWidget(self.plot_id, self.wave_counter)
            wave_widget.deserialize(w_data)
            
            wave_widget.sig_removed.connect(self.remove_waveform)
            wave_widget.sig_name_changed.connect(self.sig_structure_changed.emit)
            wave_widget.sig_target_changed_attempt.connect(self.sig_wave_target_changed.emit)
            
            self.waveforms.append(wave_widget)
            self.wave_container.addWidget(wave_widget)
        self.sig_structure_changed.emit()


# =========================================================
# 第一层：Chronos 独立记录页 (Page) 核心引擎
# =========================================================
class ChronosPage(QWidget):
    def __init__(self, hermes: HermesDatalinkQt, page_name: str):
        super().__init__()
        self.hermes = hermes
        self.page_name = page_name
        self.plot_configs = []
        self.plot_counter = 0
        
        self.graphics_items = {} 
        self.data_buffers = {}
        self.MAX_POINTS = 50000 # 放开缓存池应对高频
        
        self.is_running = False
        self.start_time = 0.0
        self.mem_req_queue = deque() 
        self._is_loading_json = False
        
        self._setup_ui()
        self.hermes.sig_bus_event.connect(self.process_bus_event)
        
        self.daq_timer = QTimer()
        self.daq_timer.timeout.connect(self._daq_scheduler_tick)
        
        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self._render_tick)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        toolbar = QHBoxLayout()
        self.btn_add_plot = QPushButton("🪟 增加波形窗口")
        self.btn_add_plot.setStyleSheet("background-color: #BBDEFB; font-weight: bold; padding: 6px;")
        self.btn_add_plot.clicked.connect(self.add_plot_window)
        toolbar.addWidget(self.btn_add_plot)
        
        self.btn_clear = QPushButton("🧹 清空画布")
        self.btn_clear.clicked.connect(self.clear_buffers)
        toolbar.addWidget(self.btn_clear)
        
        self.btn_export_csv = QPushButton("📊 导出为 CSV")
        self.btn_export_csv.clicked.connect(self.export_to_csv)
        toolbar.addWidget(self.btn_export_csv)
        
        self.btn_import_cfg = QPushButton("📂 导入配置")
        self.btn_import_cfg.clicked.connect(self.action_load_json)
        toolbar.addWidget(self.btn_import_cfg)
        
        self.btn_export_cfg = QPushButton("💾 导出配置")
        self.btn_export_cfg.clicked.connect(self.action_save_json)
        toolbar.addWidget(self.btn_export_cfg)
        
        toolbar.addStretch()
        
        self.btn_run = QPushButton("▶️ 启动实时 DAQ 引擎")
        self.btn_run.setStyleSheet("background-color: #C8E6C9; font-weight: bold; font-size: 14px; padding-left: 15px; padding-right: 15px;")
        self.btn_run.clicked.connect(self.toggle_running)
        toolbar.addWidget(self.btn_run)
        
        layout.addLayout(toolbar)

        splitter = QSplitter(Qt.Horizontal)
        
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setMinimumWidth(380)
        self.scroll_area.setMaximumWidth(450)
        self.config_container = QWidget()
        self.config_layout = QVBoxLayout(self.config_container)
        self.config_layout.setAlignment(Qt.AlignTop)
        self.scroll_area.setWidget(self.config_container)
        splitter.addWidget(self.scroll_area)
        
        self.graphics_layout = pg.GraphicsLayoutWidget()
        splitter.addWidget(self.graphics_layout)
        
        splitter.setSizes([400, 1100])
        layout.addWidget(splitter)

    def add_plot_window(self):
        self.plot_counter += 1
        p_conf = PlotConfigWidget(self.plot_counter)
        p_conf.sig_plot_removed.connect(self.remove_plot_window)
        p_conf.sig_structure_changed.connect(self.rebuild_graphics_layout)
        p_conf.sig_wave_target_changed.connect(self.handle_wave_target_changed)
        
        self.plot_configs.append(p_conf)
        self.config_layout.addWidget(p_conf)
        self.rebuild_graphics_layout()
        return p_conf

    def remove_plot_window(self, p_conf):
        self.plot_configs.remove(p_conf)
        self.config_layout.removeWidget(p_conf)
        p_conf.deleteLater()
        self.rebuild_graphics_layout()

    def handle_wave_target_changed(self, w_conf):
        """核心防丢锁：验证是否需要抛弃数据"""
        if self._is_loading_json: return
        
        buf_key = (w_conf.plot_id, w_conf.wave_id)
        has_data = buf_key in self.data_buffers and len(self.data_buffers[buf_key][0]) > 0
        
        if has_data:
            reply = QMessageBox.question(self, "警告", "修改采样通道将彻底放弃当前的波形数据，是否继续？", QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.No:
                w_conf.revert_target()
                return
            
            # 清空该通道数据
            self.data_buffers[buf_key][0].clear()
            self.data_buffers[buf_key][1].clear()
            
        w_conf.commit_target()
        self.rebuild_graphics_layout()

    def rebuild_graphics_layout(self):
        if self._is_loading_json: return
        
        self.graphics_layout.clear()
        self.graphics_items.clear()
        
        for p_conf in self.plot_configs:
            self.graphics_layout.nextRow()
            plot_item = self.graphics_layout.addPlot(title=p_conf.edit_title.text())
            plot_item.showGrid(x=True, y=True, alpha=0.3)
            plot_item.addLegend(offset=(10, 10))
            plot_item.setLabel('bottom', "时间 (s)")
            
            curves = {}
            for w_conf in p_conf.waveforms:
                # 仅更新视觉和名字，不触碰数据缓冲池
                curve = plot_item.plot(pen=pg.mkPen(color=w_conf.color, width=1.5), name=w_conf.edit_name.text())
                curves[w_conf.wave_id] = curve
                
                buf_key = (p_conf.plot_id, w_conf.wave_id)
                if buf_key not in self.data_buffers:
                    self.data_buffers[buf_key] = (deque(maxlen=self.MAX_POINTS), deque(maxlen=self.MAX_POINTS))
                
            self.graphics_items[p_conf.plot_id] = (plot_item, curves)

    def clear_buffers(self):
        for buf_x, buf_y in self.data_buffers.values():
            buf_x.clear()
            buf_y.clear()
        self.start_time = time.time()
        self._render_tick() 

    def toggle_running(self):
        if self.is_running:
            self.is_running = False
            self.daq_timer.stop()
            self.gui_timer.stop()
            self.btn_run.setText("▶️ 启动实时 DAQ 引擎")
            self.btn_run.setStyleSheet("background-color: #C8E6C9; font-weight: bold; font-size: 14px; padding-left: 15px; padding-right: 15px;")
        else:
            if not self.hermes.running:
                QMessageBox.warning(self, "错误", "底层串口未连接！")
                return
            self.is_running = True
            
            # 如果是刚开始跑，同步下起跑线
            all_empty = all(len(bx) == 0 for bx, by in self.data_buffers.values())
            if all_empty: self.start_time = time.time()
                
            self.mem_req_queue.clear()
            for p in self.plot_configs:
                for w in p.waveforms: 
                    w.last_req_time = 0
                    w.waiting_for_ack = False
                    
            self.daq_timer.start(5)   # DAQ 扫描频率：5ms 极速轮询
            self.gui_timer.start(33)  # GUI 渲染频率：约 30 FPS
            self.btn_run.setText("⏸️ 停止数据采集")
            self.btn_run.setStyleSheet("background-color: #FFF59D; font-weight: bold; font-size: 14px; padding-left: 15px; padding-right: 15px;")

    # =========================================================
    # DAQ 调度引擎与路由
    # =========================================================
    def _daq_scheduler_tick(self):
        if not self.is_running: return
        current_time = time.time()

        for p_conf in self.plot_configs:
            for w_conf in p_conf.waveforms:
                target_str = w_conf.edit_target.text().strip()
                cmd_str = w_conf.edit_cmd.text().strip()
                if not target_str or not cmd_str: continue
                
                try:
                    cmd = int(cmd_str, 16)
                    rate_ms = w_conf.get_rate_ms()
                    is_mem = w_conf.cb_mode.currentIndex() == 1
                    
                    should_send = False
                    if rate_ms == 0:
                        if not w_conf.waiting_for_ack: should_send = True
                    else:
                        if (current_time - w_conf.last_req_time) >= (rate_ms / 1000.0): should_send = True

                    if should_send:
                        w_conf.last_req_time = current_time
                        w_conf.waiting_for_ack = True
                        
                        if is_mem:
                            addr = int(target_str, 16) if target_str.lower().startswith('0x') else int(target_str)
                            fmt, size = TYPE_FORMATS[w_conf.cb_type.currentText()]
                            payload = struct.pack('<IBH', addr, size, 1)
                            self.mem_req_queue.append((p_conf.plot_id, w_conf.wave_id, w_conf.cb_type.currentText()))
                        else:
                            var_id = int(target_str)
                            payload = struct.pack('<BB', 1, var_id)
                            
                        self.hermes.send_frame(0x01, cmd, bytes(payload), priority=2)
                except ValueError:
                    continue

    def process_bus_event(self, ev: dict):
        if not self.is_running or ev.get('type') != 'DL' or ev.get('dir') != 'RX' or not ev.get('dl_crc_ok'):
            return

        cmd = ev['dl_cmd']
        payload = ev['dl_payload']
        recv_time = time.time() - self.start_time

        if cmd == 0x30:
            if len(payload) < 2: return
            valid_cnt, param_id = payload[0], payload[1]
            for p_conf in self.plot_configs:
                for w_conf in p_conf.waveforms:
                    if w_conf.cb_mode.currentIndex() == 0 and w_conf.edit_target.text() == str(param_id):
                        if len(payload) >= 6:
                            val = struct.unpack_from('<f', payload, 2)[0]
                            self._append_data(p_conf.plot_id, w_conf.wave_id, recv_time, val)
                        w_conf.waiting_for_ack = False

        elif cmd == 0x50:
            if len(payload) < 2 or not self.mem_req_queue: return
            status = payload[0]
            req_plot_id, req_wave_id, req_type = self.mem_req_queue.popleft()
            
            if status == 0:
                fmt, size = TYPE_FORMATS[req_type]
                if len(payload) >= 1 + size:
                    val = struct.unpack_from(fmt, payload, 1)[0]
                    self._append_data(req_plot_id, req_wave_id, recv_time, val)
                    
            for p_conf in self.plot_configs:
                if p_conf.plot_id == req_plot_id:
                    for w_conf in p_conf.waveforms:
                        if w_conf.wave_id == req_wave_id:
                            w_conf.waiting_for_ack = False

    def _append_data(self, plot_id, wave_id, x_val, y_val):
        buf_key = (plot_id, wave_id)
        if buf_key in self.data_buffers:
            x_buf, y_buf = self.data_buffers[buf_key]
            x_buf.append(x_val)
            y_buf.append(y_val)

    # =========================================================
    # 多态坐标轴与异步GUI渲染器
    # =========================================================
    def _render_tick(self):
        for p_conf in self.plot_configs:
            if p_conf.plot_id not in self.graphics_items: continue
            plot_item = self.graphics_items[p_conf.plot_id][0]
            curves = self.graphics_items[p_conf.plot_id][1]
            
            max_x_in_plot = 0.0
            
            for w_conf in p_conf.waveforms:
                buf_key = (p_conf.plot_id, w_conf.wave_id)
                if buf_key in self.data_buffers and w_conf.wave_id in curves:
                    x_buf, y_buf = self.data_buffers[buf_key]
                    if len(x_buf) > 0:
                        max_x_in_plot = max(max_x_in_plot, x_buf[-1])
                        curves[w_conf.wave_id].setData(np.array(x_buf), np.array(y_buf))
            
            # X轴滑窗定标逻辑
            if p_conf.cb_slide_x.isChecked():
                try:
                    win_s = float(p_conf.edit_slide_x.text())
                    plot_item.setXRange(max(0, max_x_in_plot - win_s), max_x_in_plot, padding=0)
                except: pass
            
            # Y轴自动定标逻辑
            if not p_conf.cb_auto_y.isChecked():
                try:
                    y_min = float(p_conf.edit_y_min.text())
                    y_max = float(p_conf.edit_y_max.text())
                    plot_item.setYRange(y_min, y_max, padding=0)
                except: pass
            else:
                plot_item.enableAutoRange(axis=pg.ViewBox.YAxis)

    # =========================================================
    # 导入导出生态 (JSON / CSV)
    # =========================================================
    def action_save_json(self):
        path, _ = QFileDialog.getSaveFileName(self, "导出 Chronos 记录仪配置", f"chronos_{self.page_name}.json", "JSON Files (*.json)")
        if path:
            data = {'plots': [p.serialize() for p in self.plot_configs]}
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4)
            QMessageBox.information(self, "成功", "配置导出成功！")

    def action_load_json(self):
        path, _ = QFileDialog.getOpenFileName(self, "载入 Chronos 配置", "", "JSON Files (*.json)")
        if path:
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    
                self._is_loading_json = True
                
                # 清理旧视图
                for p_conf in self.plot_configs:
                    self.config_layout.removeWidget(p_conf)
                    p_conf.deleteLater()
                self.plot_configs.clear()
                self.data_buffers.clear()
                
                # 重建视图
                for p_data in data.get('plots', []):
                    p_conf = PlotConfigWidget(p_data['plot_id'])
                    p_conf.deserialize(p_data)
                    p_conf.sig_plot_removed.connect(self.remove_plot_window)
                    p_conf.sig_structure_changed.connect(self.rebuild_graphics_layout)
                    p_conf.sig_wave_target_changed.connect(self.handle_wave_target_changed)
                    self.plot_configs.append(p_conf)
                    self.config_layout.addWidget(p_conf)
                    self.plot_counter = max(self.plot_counter, p_conf.plot_id)
                
                self._is_loading_json = False
                self.rebuild_graphics_layout()
                QMessageBox.information(self, "成功", "配置载入成功！")
            except Exception as e:
                self._is_loading_json = False
                QMessageBox.critical(self, "错误", f"载入失败: {e}")

    def export_to_csv(self):
        """异步总线下的健壮型多列 CSV 导出引擎"""
        if not self.data_buffers:
            QMessageBox.warning(self, "为空", "没有可导出的数据！")
            return
            
        path, _ = QFileDialog.getSaveFileName(self, "导出为 CSV", f"chronos_data.csv", "CSV Files (*.csv)")
        if path:
            try:
                headers = []
                data_columns = []
                max_rows = 0
                
                for p_conf in self.plot_configs:
                    for w_conf in p_conf.waveforms:
                        buf_key = (p_conf.plot_id, w_conf.wave_id)
                        if buf_key in self.data_buffers:
                            x_buf, y_buf = self.data_buffers[buf_key]
                            if len(x_buf) > 0:
                                name = w_conf.edit_name.text()
                                headers.extend([f"{name}_Time(s)", f"{name}_Value"])
                                data_columns.append(list(x_buf))
                                data_columns.append(list(y_buf))
                                max_rows = max(max_rows, len(x_buf))
                
                if max_rows == 0:
                    QMessageBox.warning(self, "为空", "所有通道都没有接收到数据！")
                    return

                with open(path, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow(headers)
                    for i in range(max_rows):
                        row = []
                        for col in data_columns:
                            row.append(col[i] if i < len(col) else "")
                        writer.writerow(row)
                QMessageBox.information(self, "成功", "CSV 导出成功！")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"CSV 导出失败: {e}")

# =========================================================
# 容器管理面板 (TabChronosManager) 
# =========================================================
class TabChronosManager(QWidget):
    def __init__(self, hermes: HermesDatalinkQt):
        super().__init__()
        self.hermes = hermes
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        toolbar = QHBoxLayout()
        self.btn_add_tab = QPushButton("➕ 新建记录页 (New Chronos Page)")
        self.btn_add_tab.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 5px;")
        self.btn_add_tab.clicked.connect(self.add_new_page)
        
        toolbar.addWidget(self.btn_add_tab)
        toolbar.addStretch()
        layout.addLayout(toolbar)
        
        self.tab_widget = QTabWidget()
        self.tab_widget.setTabsClosable(True)
        self.tab_widget.tabCloseRequested.connect(self.remove_page)
        layout.addWidget(self.tab_widget)
        
        self.add_new_page(default_name="Scope 1")

    def add_new_page(self, checked=False, default_name=None):
        name = default_name
        if not name:
            name, ok = QInputDialog.getText(self, "新建页面", "请输入记录页名称:")
            if not ok or not name.strip(): return
                
        new_page = ChronosPage(self.hermes, name)
        self.tab_widget.addTab(new_page, f"📈 {name}")
        self.tab_widget.setCurrentWidget(new_page)
        
        if default_name:
            p_conf = new_page.add_plot_window()
            p_conf.add_waveform()

    def remove_page(self, index):
        widget = self.tab_widget.widget(index)
        if hasattr(widget, 'is_running') and widget.is_running:
            widget.toggle_running() 
        self.tab_widget.removeTab(index)