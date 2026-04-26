import struct
from PyQt5.QtCore import Qt, QTimer, QObject, QEvent
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                             QLineEdit, QPushButton, QLabel, QGroupBox, QFormLayout,
                             QGridLayout, QCheckBox, QScrollArea, QApplication)
from core_datalink import HermesDatalinkQt

# 内部相对偏移量
REL_OFFSET_SET_MASK   = 10
REL_OFFSET_SET_MASK_ACK = 11
REL_OFFSET_STEP       = 12
REL_OFFSET_STEP_ACK   = 13
REL_OFFSET_SET_INPUT  = 14
REL_OFFSET_SET_INPUT_ACK = 15
REL_OFFSET_GET_OUTPUT = 16
REL_OFFSET_GET_OUTPUT_ACK = 17

class CheckboxDragFilter(QObject):
    """滑动拖拽批量勾选/取消"""
    def __init__(self, parent_sync_func):
        super().__init__()
        self.dragging = False
        self.target_state = False
        self.sync_func = parent_sync_func

    def eventFilter(self, obj, event):
        if isinstance(obj, QCheckBox) and obj.property("is_mask_cb"):
            if event.type() == QEvent.MouseButtonPress and event.button() == Qt.LeftButton:
                self.dragging = True
                self.target_state = not obj.isChecked()
            elif event.type() == QEvent.MouseMove and self.dragging:
                widget = QApplication.widgetAt(event.globalPos())
                if isinstance(widget, QCheckBox) and widget.property("is_mask_cb"):
                    if widget.isChecked() != self.target_state:
                        widget.setChecked(self.target_state)
                        self.sync_func()
            elif event.type() == QEvent.MouseButtonRelease and event.button() == Qt.LeftButton:
                self.dragging = False
                self.sync_func()
        return False

class TabSim(QWidget):
    def __init__(self, hermes: HermesDatalinkQt):
        super().__init__()
        self.hermes = hermes
        self.hermes.sig_bus_event.connect(self.on_bus_event)
        
        self.current_mask_tx = 0xFFFFFFFF
        self.current_mask_rx = 0xFFFFFFFF
        
        self.rx_widgets = {} 
        self.tx_widgets = {} 
        
        self.drag_filter = CheckboxDragFilter(self.sync_local_masks)
        
        self._setup_ui()
        self.sync_local_masks()

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)

        # ==========================================
        # 1. 顶部配置：Base CMD
        # ==========================================
        config_group = QGroupBox("仿真引擎协议配置")
        config_layout = QHBoxLayout()
        
        config_layout.addWidget(QLabel("Datalink Base CMD:"))
        self.input_base_cmd = QLineEdit("0x10")
        self.input_base_cmd.setMaximumWidth(80)
        config_layout.addWidget(self.input_base_cmd)
        
        self.btn_set_mask = QPushButton("同步 Mask 状态至下位机")
        self.btn_set_mask.clicked.connect(self.cmd_set_mask)
        self.set_mask_button_state(False) # 初始化为未同步状态
        
        config_layout.addStretch()
        config_layout.addWidget(self.btn_set_mask)
        
        config_group.setLayout(config_layout)
        main_layout.addWidget(config_group)

        # ==========================================
        # 2. 核心数据区 (左右半屏分栏)
        # ==========================================
        data_scroll = QScrollArea()
        data_scroll.setWidgetResizable(True)
        data_content = QWidget()
        data_grid_main = QHBoxLayout(data_content)
        
        self.tx_mask_boxes = []
        self.rx_mask_boxes = []

        # 左半屏：RX (输入，PC -> MCU)
        left_vbox = QVBoxLayout()
        left_vbox.addWidget(self._build_mask_group("RX Mask (仿真输入)", 
                            [("ADC Results", 24), ("Panel Inputs", 8)], self.rx_mask_boxes))
        
        rx_data_group = QGroupBox("RX 寄存器数值配置")
        rx_data_vbox = QVBoxLayout(rx_data_group)
        self._build_rx_data_table(rx_data_vbox)
        left_vbox.addWidget(rx_data_group, stretch=1)
        data_grid_main.addLayout(left_vbox, stretch=1)

        # 右半屏：TX (输出，MCU -> PC)
        right_vbox = QVBoxLayout()
        right_vbox.addWidget(self._build_mask_group("TX Mask (算法输出)", 
                            [("PWM Compares", 8), ("DAC Outs", 8), ("Monitors", 16)], self.tx_mask_boxes))
        
        tx_data_group = QGroupBox("TX 寄存器数值监视")
        tx_data_vbox = QVBoxLayout(tx_data_group)
        self._build_tx_data_table(tx_data_vbox)
        right_vbox.addWidget(tx_data_group, stretch=1)
        data_grid_main.addLayout(right_vbox, stretch=1)
        
        data_scroll.setWidget(data_content)
        main_layout.addWidget(data_scroll, stretch=1)

        # ==========================================
        # 3. 动作按钮区
        # ==========================================
        btn_layout = QHBoxLayout()
        
        self.btn_set_input = QPushButton("1. 仅设输入 (SET INPUT)")
        self.btn_set_input.setFixedHeight(50)
        self.btn_set_input.clicked.connect(self.cmd_set_input)
        
        self.btn_get_out = QPushButton("2. 仅拉状态 (GET OUTPUT)")
        self.btn_get_out.setFixedHeight(50)
        self.btn_get_out.clicked.connect(self.cmd_get_output)
        
        self.btn_step = QPushButton("⚡ 执行单步闭环仿真 (STEP)")
        self.btn_step.setFixedHeight(50)
        self.btn_step.setStyleSheet("background-color: #C8E6C9; font-weight: bold; font-size: 16px; border: 2px solid #4CAF50; border-radius: 4px;")
        self.btn_step.clicked.connect(self.cmd_step)
        
        btn_layout.addWidget(self.btn_set_input, stretch=1)
        btn_layout.addWidget(self.btn_get_out, stretch=1)
        btn_layout.addWidget(self.btn_step, stretch=2)
        main_layout.addLayout(btn_layout)

    # ------------------------------------------------------
    # UI 构建辅助
    # ------------------------------------------------------
    def _build_mask_group(self, title, config, box_list):
        group = QGroupBox(title)
        grid = QGridLayout()
        grid.setSpacing(5)
        
        current_bit = 0
        grid_row = 0
        for label, count in config:
            grid.addWidget(QLabel(f"<b>{label}:</b>"), grid_row, 0, 1, 4)
            grid_row += 1
            for i in range(count):
                cb = QCheckBox(f"Ch {i}")
                cb.setChecked(True) 
                cb.setProperty("is_mask_cb", True)
                cb.installEventFilter(self.drag_filter)
                cb.setToolTip(f"Global Bit {current_bit}")
                cb.stateChanged.connect(self.sync_local_masks)
                box_list.append(cb)
                grid.addWidget(cb, grid_row + (i // 4), i % 4)
                current_bit += 1
            grid_row += (count - 1) // 4 + 1
            
        group.setLayout(grid)
        return group

    def _build_rx_data_table(self, layout):
        form = QFormLayout()
        self.rx_widgets['isr_ticks'] = QLineEdit("0")
        self.rx_widgets['dig_in'] = QLineEdit("0x00000000")
        form.addRow("ISR Ticks:", self.rx_widgets['isr_ticks'])
        form.addRow("Digital In (Hex):", self.rx_widgets['dig_in'])
        layout.addLayout(form)
        
        layout.addWidget(QLabel("<b>ADC Results (16-bit):</b>"))
        grid_adc = QGridLayout()
        for i in range(24):
            w = QLineEdit("0")
            w.setFixedWidth(70)
            self.rx_widgets[f'adc_{i}'] = w
            grid_adc.addWidget(w, i // 4, i % 4)
        layout.addLayout(grid_adc)
        
        layout.addWidget(QLabel("<b>Panel Inputs (float):</b>"))
        grid_p = QGridLayout()
        for i in range(8):
            w = QLineEdit("0.0")
            w.setFixedWidth(70)
            self.rx_widgets[f'panel_{i}'] = w
            grid_p.addWidget(w, i // 4, i % 4)
        layout.addLayout(grid_p)

    def _build_tx_data_table(self, layout):
        form = QFormLayout()
        self.tx_widgets['dig_out'] = QLineEdit("0x00000000")
        form.addRow("Digital Out (Hex):", self.tx_widgets['dig_out'])
        layout.addLayout(form)

        layout.addWidget(QLabel("<b>PWM Compares (16-bit):</b>"))
        grid_pwm = QGridLayout()
        for i in range(8):
            w = QLineEdit("0") 
            w.setFixedWidth(70)
            self.tx_widgets[f'pwm_{i}'] = w
            grid_pwm.addWidget(w, i // 4, i % 4)
        layout.addLayout(grid_pwm)
        
        layout.addWidget(QLabel("<b>DAC Outputs (16-bit):</b>"))
        grid_dac = QGridLayout()
        for i in range(8):
            w = QLineEdit("0") 
            w.setFixedWidth(70)
            self.tx_widgets[f'dac_{i}'] = w
            grid_dac.addWidget(w, i // 4, i % 4)
        layout.addLayout(grid_dac)

        layout.addWidget(QLabel("<b>Monitor Variables (float):</b>"))
        grid_m = QGridLayout()
        for i in range(16):
            w = QLineEdit("0.0") 
            w.setFixedWidth(70)
            self.tx_widgets[f'mon_{i}'] = w
            grid_m.addWidget(w, i // 4, i % 4)
        layout.addLayout(grid_m)

    # =========================================================
    # 状态控制与交互反馈
    # =========================================================
    def set_mask_button_state(self, is_synced: bool):
        """控制 Mask 同步按钮的闭环状态指示"""
        if is_synced:
            self.btn_set_mask.setText("✅ Mask 已同步")
            self.btn_set_mask.setStyleSheet("background-color: #C8E6C9; color: #2E7D32; font-weight: bold; height: 30px; padding: 0 15px; border: 1px solid #4CAF50; border-radius: 4px;")
        else:
            self.btn_set_mask.setText("同步 Mask 状态至下位机")
            self.btn_set_mask.setStyleSheet("background-color: #FFF9C4; color: black; font-weight: bold; height: 30px; padding: 0 15px; border: 1px solid #FBC02D; border-radius: 4px;")

    def apply_base_style(self, w: QLineEdit, is_tx=False):
        if w.property("mask_enabled"):
            w.setStyleSheet("border: 2px solid #FBC02D; background-color: #FFFDE7; color: black; border-radius: 3px;")
            if not is_tx: w.setReadOnly(False)
        else:
            w.setStyleSheet("border: 1px solid #E0E0E0; background-color: #F5F5F5; color: #BDBDBD; border-radius: 3px;")
            if not is_tx: w.setReadOnly(True)

    def highlight_widget(self, w: QLineEdit, is_tx=False):
        w.setStyleSheet("border: 2px solid #4CAF50; background-color: #E8F5E9; color: black; font-weight: bold; border-radius: 3px;")
        QTimer.singleShot(800, lambda: self.apply_base_style(w, is_tx))

    def sync_local_masks(self):
        """复选框变动时触发，更新样式并打破同步状态"""
        self.current_mask_tx = sum((1 << i) for i, cb in enumerate(self.tx_mask_boxes) if cb.isChecked())
        self.current_mask_rx = sum((1 << i) for i, cb in enumerate(self.rx_mask_boxes) if cb.isChecked())

        # 只要修改了，立刻标记为未同步
        self.set_mask_button_state(False)

        for w in [self.rx_widgets['isr_ticks'], self.rx_widgets['dig_in']]:
            w.setProperty("mask_enabled", True); self.apply_base_style(w, False)
        self.tx_widgets['dig_out'].setProperty("mask_enabled", True)
        self.apply_base_style(self.tx_widgets['dig_out'], True)

        for i in range(24):
            self.rx_widgets[f'adc_{i}'].setProperty("mask_enabled", bool((self.current_mask_rx >> i) & 1))
            self.apply_base_style(self.rx_widgets[f'adc_{i}'], False)
        for i in range(8):
            self.rx_widgets[f'panel_{i}'].setProperty("mask_enabled", bool((self.current_mask_rx >> (24 + i)) & 1))
            self.apply_base_style(self.rx_widgets[f'panel_{i}'], False)
            
        for i in range(8):
            self.tx_widgets[f'pwm_{i}'].setProperty("mask_enabled", bool((self.current_mask_tx >> i) & 1))
            self.apply_base_style(self.tx_widgets[f'pwm_{i}'], True)
            self.tx_widgets[f'pwm_{i}'].setReadOnly(True)
            
            self.tx_widgets[f'dac_{i}'].setProperty("mask_enabled", bool((self.current_mask_tx >> (8 + i)) & 1))
            self.apply_base_style(self.tx_widgets[f'dac_{i}'], True)
            self.tx_widgets[f'dac_{i}'].setReadOnly(True)
            
        for i in range(16):
            self.tx_widgets[f'mon_{i}'].setProperty("mask_enabled", bool((self.current_mask_tx >> (16 + i)) & 1))
            self.apply_base_style(self.tx_widgets[f'mon_{i}'], True)
            self.tx_widgets[f'mon_{i}'].setReadOnly(True)

    # =========================================================
    # 核心封包与指令交互
    # =========================================================
    def _get_target_cmd(self, rel_offset):
        try: return int(self.input_base_cmd.text(), 16) + rel_offset
        except ValueError: return 0

    def cmd_set_mask(self):
        # 点击时同步当前 Mask 并发送
        self.sync_local_masks()
        payload = struct.pack('<II', self.current_mask_tx, self.current_mask_rx)
        self.hermes.send_frame(0x01, self._get_target_cmd(REL_OFFSET_SET_MASK), payload)

    def cmd_set_input(self):
        self.hermes.send_frame(0x01, self._get_target_cmd(REL_OFFSET_SET_INPUT), self._pack_rx_buffer())

    def cmd_get_output(self):
        self.hermes.send_frame(0x01, self._get_target_cmd(REL_OFFSET_GET_OUTPUT), b'')

    def cmd_step(self):
        self.hermes.send_frame(0x01, self._get_target_cmd(REL_OFFSET_STEP), self._pack_rx_buffer())

    def _pack_rx_buffer(self) -> bytes:
        payload = bytearray()
        try:
            ticks = int(self.rx_widgets['isr_ticks'].text())
            dig = int(self.rx_widgets['dig_in'].text(), 16)
        except: ticks, dig = 0, 0
        payload.extend(struct.pack('<II', ticks, dig))
        
        for i in range(24):
            if (self.current_mask_rx >> i) & 1:
                try: val = int(self.rx_widgets[f'adc_{i}'].text())
                except: val = 0
                payload.extend(struct.pack('<H', val))
                
        for i in range(8):
            if (self.current_mask_rx >> (24 + i)) & 1:
                try: val = float(self.rx_widgets[f'panel_{i}'].text())
                except: val = 0.0
                payload.extend(struct.pack('<f', val))
        return bytes(payload)

    def on_bus_event(self, ev: dict):
        if ev['type'] != 'DL' or ev['dir'] != 'RX' or not ev['dl_crc_ok']: return
        cmd, payload = ev['dl_cmd'], ev['dl_payload']

        # 1. 处理 Mask 设置的回传校验 (闭环核心)
        if cmd == self._get_target_cmd(REL_OFFSET_SET_MASK_ACK):
            if len(payload) >= 12:
                status, ack_tx, ack_rx = struct.unpack_from('<III', payload, 0)
                if status == 0 and ack_tx == self.current_mask_tx and ack_rx == self.current_mask_rx:
                    self.set_mask_button_state(True)
                    # 【核心修复】：增加明确的日志反馈，防止“静默成功”
                    self.log(f"✅ MASK 闭环同步成功! (TX: 0x{ack_tx:08X}, RX: 0x{ack_rx:08X})", "green")
                else:
                    self.log(f"⚠️ MASK 同步比对失败! 期望TX: {self.current_mask_tx:08X}, 实际: {ack_tx:08X}", "red")
            else:
                self.log(f"❌ 收到无效的 MASK ACK，长度不足: {len(payload)} 字节", "red")

        # 2. 处理 STEP 和 GET_OUTPUT 的数据回传
        elif cmd in (self._get_target_cmd(REL_OFFSET_STEP_ACK), self._get_target_cmd(REL_OFFSET_GET_OUTPUT_ACK)):
            if len(payload) < 8: return
            mask_tx, dig_out = struct.unpack_from('<II', payload, 0)
            idx = 8
            
            w = self.tx_widgets['dig_out']
            w.setText(f"0x{dig_out:08X}")
            self.highlight_widget(w, True)
            
            for i in range(8):
                if (mask_tx >> i) & 1:
                    w = self.tx_widgets[f'pwm_{i}']
                    w.setText(str(struct.unpack_from('<H', payload, idx)[0]))
                    self.highlight_widget(w, True)
                    idx += 2
            
            for i in range(8):
                if (mask_tx >> (8 + i)) & 1:
                    w = self.tx_widgets[f'dac_{i}']
                    w.setText(str(struct.unpack_from('<H', payload, idx)[0]))
                    self.highlight_widget(w, True)
                    idx += 2

            for i in range(16):
                if (mask_tx >> (16 + i)) & 1:
                    w = self.tx_widgets[f'mon_{i}']
                    w.setText(f"{struct.unpack_from('<f', payload, idx)[0]:.4f}")
                    self.highlight_widget(w, True)
                    idx += 4