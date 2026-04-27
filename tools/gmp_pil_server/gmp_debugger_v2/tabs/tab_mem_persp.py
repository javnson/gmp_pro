import os
import json
import struct
import csv
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                             QLineEdit, QPushButton, QLabel, QFormLayout, 
                             QTableWidget, QTableWidgetItem, QHeaderView, 
                             QFileDialog, QMessageBox, QComboBox, QMenu, QInputDialog)
from PyQt5.QtCore import Qt, pyqtSignal
from core_datalink import HermesDatalinkQt

# 支持的 Cast 数据类型与对应的 struct 格式及字节长度
CAST_TYPES = {
    'F32': ('<f', 4), 'I32': ('<i', 4), 'U32': ('<I', 4),
    'I16': ('<h', 2), 'U16': ('<H', 2), 'I8': ('<b', 1), 'U8': ('<B', 1)
}

class TabMemPersp(QWidget):
    def __init__(self, hermes: HermesDatalinkQt):
        super().__init__()
        self.hermes = hermes
        self.hermes.sig_bus_event.connect(self.on_bus_event)
        
        self.base_cmd = 0x50
        self.target_addr = 0x00000000
        self.total_length = 256  # 默认读取长度 (Bytes)
        self.granularity = 1     # 默认颗粒度 (1=8bit, 2=16bit, 4=32bit)
        
        self.memory_buffer = bytearray()
        
        # 记录特定偏移量下的强转配置: { byte_offset: {"type": "F32", "active": True} }
        self.casts = {}
        
        # 分片读取状态机参数
        self.is_reading = False
        self.read_offset = 0
        self.read_chunk_bytes = 128 # 每次向 MCU 请求的字节数，防止总线超载
        
        self._setup_ui()

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)

        # ==========================================
        # 1. 顶部配置与导出控制 (分两行优化布局)
        # ==========================================
        ctrl_group = QGroupBox("Argos 内存透视配置 (Memory Perspective)")
        ctrl_layout = QVBoxLayout() # 使用垂直布局容纳两行
        ctrl_layout.setSpacing(10)
        
        # --- 第一行：基础指令、颗粒度、导出 ---
        row1_layout = QHBoxLayout()
        row1_layout.addWidget(QLabel("Base CMD:"))
        self.edit_cmd = QLineEdit("0x50")
        self.edit_cmd.setMaximumWidth(60)
        row1_layout.addWidget(self.edit_cmd)
        
        row1_layout.addWidget(QLabel("  读取颗粒度:"))
        self.cb_granularity = QComboBox()
        self.cb_granularity.addItems(["1 Byte (8-bit)", "2 Bytes (16-bit)", "4 Bytes (32-bit)"])
        row1_layout.addWidget(self.cb_granularity)
        
        row1_layout.addStretch()
        
        self.btn_export_csv = QPushButton("📊 导出为 CSV")
        self.btn_export_csv.clicked.connect(self.action_export_csv)
        row1_layout.addWidget(self.btn_export_csv)
        
        # --- 第二行：物理地址、长度、配置存取 ---
        row2_layout = QHBoxLayout()
        row2_layout.addWidget(QLabel("起始物理地址 (Hex):"))
        self.edit_addr = QLineEdit("0x20000000")
        self.edit_addr.setMaximumWidth(100)
        row2_layout.addWidget(self.edit_addr)
        
        row2_layout.addWidget(QLabel("  总字节数:"))
        self.edit_len = QLineEdit("256")
        self.edit_len.setMaximumWidth(60)
        row2_layout.addWidget(self.edit_len)
        
        row2_layout.addStretch()
        
        self.btn_load_cfg = QPushButton("📂 载入 Cast 配置")
        self.btn_load_cfg.clicked.connect(self.action_load_cfg)
        row2_layout.addWidget(self.btn_load_cfg)
        
        self.btn_save_cfg = QPushButton("💾 保存 Cast 配置")
        self.btn_save_cfg.clicked.connect(self.action_save_cfg)
        row2_layout.addWidget(self.btn_save_cfg)

        # 将两行加入主控制组
        ctrl_layout.addLayout(row1_layout)
        ctrl_layout.addLayout(row2_layout)
        ctrl_group.setLayout(ctrl_layout)
        main_layout.addWidget(ctrl_group)

        # ==========================================
        # 2. 核心数据网格区
        # ==========================================
        self.table = QTableWidget(0, 16) # 默认 16 列
        self.table.setStyleSheet("QTableWidget { font-family: Consolas, monospace; font-size: 13px; }")
        self.table.setEditTriggers(QTableWidget.NoEditTriggers) # 禁止直接双击打字，使用右键菜单修改
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        # 绑定事件
        self.table.setContextMenuPolicy(Qt.CustomContextMenu)
        self.table.customContextMenuRequested.connect(self.show_context_menu)
        self.table.cellDoubleClicked.connect(self.on_cell_double_clicked)
        
        main_layout.addWidget(self.table)

        # ==========================================
        # 3. 底部执行按钮
        # ==========================================
        btn_layout = QHBoxLayout()
        self.btn_read = QPushButton("🔍 提取并刷新全局内存区域 (Read All)")
        self.btn_read.setMinimumHeight(45)
        self.btn_read.setStyleSheet("background-color: #E3F2FD; font-weight: bold; font-size: 14px;")
        self.btn_read.clicked.connect(self.cmd_start_read_all)
        
        btn_layout.addWidget(self.btn_read)
        main_layout.addLayout(btn_layout)

    def log(self, msg, color="black"):
        if hasattr(self.hermes, 'sig_log_msg'):
            self.hermes.sig_log_msg.emit(f"<span style='color:{color};'>[Argos Mem] {msg}</span>")
        else:
            print(msg)

    def _sync_config(self):
        try: self.base_cmd = int(self.edit_cmd.text(), 16)
        except: pass
        try: self.target_addr = int(self.edit_addr.text(), 16)
        except: pass
        try: self.total_length = int(self.edit_len.text())
        except: pass
        
        gran_text = self.cb_granularity.currentText()
        if "1 Byte" in gran_text: self.granularity = 1
        elif "2 Bytes" in gran_text: self.granularity = 2
        else: self.granularity = 4

    # =========================================================
    # 分片读取状态机 (Chunked Reading Engine)
    # =========================================================
    def cmd_start_read_all(self):
        if not self.hermes.running or self.is_reading: return
        self._sync_config()
        
        # 根据颗粒度强制对齐总长度
        remainder = self.total_length % self.granularity
        if remainder != 0:
            self.total_length += (self.granularity - remainder)
            self.edit_len.setText(str(self.total_length))
            self.log(f"长度已自动对齐至 {self.total_length} Bytes", "orange")

        self.memory_buffer = bytearray(self.total_length)
        self.is_reading = True
        self.read_offset = 0
        self.btn_read.setText("⏳ 正在提取内存...")
        self.btn_read.setEnabled(False)
        
        self._request_next_chunk()

    def _request_next_chunk(self):
        if self.read_offset >= self.total_length:
            self.is_reading = False
            self.btn_read.setText("🔍 提取并刷新全局内存区域 (Read All)")
            self.btn_read.setEnabled(True)
            self.render_table()
            self.log("✅ 内存区域提取完毕", "green")
            return

        remain = self.total_length - self.read_offset
        chunk_bytes = min(self.read_chunk_bytes, remain)
        # 确保 chunk_bytes 是颗粒度的整数倍
        chunk_bytes -= (chunk_bytes % self.granularity)
        
        current_addr = self.target_addr + self.read_offset
        item_count = chunk_bytes // self.granularity

        # Payload: [Address(4B)] + [Item Size(1B)] + [Item Count(2B)]
        req_pld = struct.pack('<IBH', current_addr, self.granularity, item_count)
        self.hermes.send_frame(0x01, self.base_cmd, req_pld, priority=1)

    # =========================================================
    # 数据网格渲染与 Cast 融合
    # =========================================================
    def render_table(self):
        if not self.memory_buffer: return
        
        # 固定每行显示 16 Bytes 的物理跨度
        bytes_per_row = 16
        cols = bytes_per_row // self.granularity
        rows = (self.total_length + bytes_per_row - 1) // bytes_per_row
        
        self.table.clear()
        self.table.setRowCount(rows)
        self.table.setColumnCount(cols)
        
        # 设置行表头 (物理地址)
        row_labels = [f"0x{self.target_addr + r * bytes_per_row:08X}" for r in range(rows)]
        self.table.setVerticalHeaderLabels(row_labels)
        
        # 设置列表头 (相对偏移)
        col_labels = [f"+{c * self.granularity:02X}" for c in range(cols)]
        self.table.setHorizontalHeaderLabels(col_labels)
        
        row, col = 0, 0
        offset = 0
        
        while offset < self.total_length:
            if col >= cols:
                row += 1; col = 0
                
            # 1. 检查是否存在活跃的 Cast
            cast_handled = False
            if offset in self.casts and self.casts[offset]['active']:
                c_type = self.casts[offset]['type']
                fmt, c_len = CAST_TYPES[c_type]
                
                if offset + c_len <= self.total_length:
                    chunk = self.memory_buffer[offset : offset + c_len]
                    try:
                        val = struct.unpack(fmt, chunk)[0]
                        txt = f"{val:.4f}" if 'F' in c_type else str(val)
                        txt += f" ({c_type})"
                        
                        item = QTableWidgetItem(txt)
                        item.setTextAlignment(Qt.AlignCenter)
                        item.setBackground(Qt.yellow)
                        item.setForeground(Qt.black)
                        item.setData(Qt.UserRole, offset)
                        
                        # 计算在当前颗粒度下需要横跨几列
                        col_span = max(1, c_len // self.granularity)
                        
                        self.table.setItem(row, col, item)
                        if col_span > 1:
                            self.table.setSpan(row, col, 1, col_span)
                            
                        offset += c_len
                        col += col_span
                        cast_handled = True
                    except: pass
            
            # 2. 正常的 Hex 渲染
            if not cast_handled:
                chunk = self.memory_buffer[offset : offset + self.granularity]
                # 根据颗粒度转化为对应的无符号整数显示
                if self.granularity == 1:   txt = f"{chunk[0]:02X}"
                elif self.granularity == 2: txt = f"{struct.unpack('<H', chunk)[0]:04X}"
                elif self.granularity == 4: txt = f"{struct.unpack('<I', chunk)[0]:08X}"
                
                item = QTableWidgetItem(txt)
                item.setTextAlignment(Qt.AlignCenter)
                item.setData(Qt.UserRole, offset) # 隐式绑定字节偏移量
                
                self.table.setItem(row, col, item)
                
                offset += self.granularity
                col += 1

    # =========================================================
    # 交互：上下文菜单与数据修改 (局部原子写)
    # =========================================================
    def show_context_menu(self, pos):
        item = self.table.itemAt(pos)
        if not item: return
        
        # 默认获取鼠标指针下方单元格的偏移量
        offset = item.data(Qt.UserRole)
        
        # 判断右键点击的位置是否在用户的“框选区域”内
        selected_items = self.table.selectedItems()
        if len(selected_items) > 1 and item in selected_items:
            valid_offsets = [it.data(Qt.UserRole) for it in selected_items if it.data(Qt.UserRole) is not None]
            if valid_offsets:
                offset = min(valid_offsets)
        
        menu = QMenu(self)
        
        # 1. 局部强转子菜单
        cast_menu = menu.addMenu("🔄 局部强转 (Cast to...)")
        for t in CAST_TYPES.keys():
            action = cast_menu.addAction(t)
            action.triggered.connect(lambda checked, t_name=t, o=offset: self.apply_cast(o, t_name))
            
        # 2. 全局强转子菜单
        global_cast_menu = menu.addMenu("🌐 全局批量强转 (Global Cast to...)")
        for t in CAST_TYPES.keys():
            action = global_cast_menu.addAction(t)
            action.triggered.connect(lambda checked, t_name=t: self.apply_global_cast(t_name))
            
        menu.addSeparator()
            
        # 3. 清除强转的快捷操作
        if offset in self.casts:
            action_rm = menu.addAction("❌ 移除当前强转 (Remove Cast)")
            action_rm.triggered.connect(lambda checked, o=offset: self.remove_cast(o))
            
        if self.casts:
            action_rm_all = menu.addAction("🗑️ 清除所有强转 (Clear All Casts)")
            action_rm_all.triggered.connect(self.clear_all_casts)
            
        menu.addSeparator()
        
        # 4. 修改值入口
        action_edit = menu.addAction("✏️ 修改该值 (Edit & Write)")
        action_edit.triggered.connect(lambda checked, o=offset: self.edit_and_write_value(o))
        
        menu.exec_(self.table.viewport().mapToGlobal(pos))

    def on_cell_double_clicked(self, row, col):
        item = self.table.item(row, col)
        if not item: return
        offset = item.data(Qt.UserRole)
        
        # 【体验优化】：双击直接触发该地址的局部修改弹窗
        self.edit_and_write_value(offset)

    def apply_cast(self, offset, type_name):
        self.casts[offset] = {'type': type_name, 'active': True}
        self.table.clearSpans()
        self.render_table()
        
    def remove_cast(self, offset):
        if offset in self.casts:
            del self.casts[offset]
            self.table.clearSpans()
            self.render_table()

    def edit_and_write_value(self, offset):
        if not self.hermes.running:
            self.log("⚠️ 串口未打开，无法下发修改指令", "red")
            return
            
        target_addr = self.target_addr + offset
        
        # 1. 如果该单元格是 Cast 类型
        if offset in self.casts and self.casts[offset]['active']:
            c_type = self.casts[offset]['type']
            fmt, c_len = CAST_TYPES[c_type]
            
            prompt_str = f"请输入新的 {c_type} 数值:"
            new_val_str, ok = QInputDialog.getText(self, "修改内存", prompt_str)
            if not ok or not new_val_str.strip(): return
            
            try:
                val = float(new_val_str) if 'F' in c_type else int(new_val_str)
                data_bytes = struct.pack(fmt, val)
                
                # 针对 Cast 类型，Item Size = c_len, Count = 1
                req_pld = bytearray(struct.pack('<IBH', target_addr, c_len, 1))
                req_pld.extend(data_bytes)
                
                self.hermes.send_frame(0x01, self.base_cmd + 1, bytes(req_pld), priority=0)
                self.log(f"局部覆写指令已下发 -> Addr: 0x{target_addr:08X} | Val: {val}", "blue")
            except Exception as e:
                QMessageBox.critical(self, "格式错误", f"输入的数据格式不符合 {c_type} 要求!\n{e}")
                
        # 2. 如果是正常的 Hex 单元格
        else:
            prompt_str = f"请输入新的 {self.granularity * 2} 位 Hex 字符 (无需0x):"
            new_val_str, ok = QInputDialog.getText(self, "修改内存", prompt_str)
            if not ok or not new_val_str.strip(): return
            
            clean_hex = re.sub(r'[^0-9a-fA-F]', '', new_val_str)
            if len(clean_hex) != self.granularity * 2:
                QMessageBox.critical(self, "长度错误", f"您当前的读取颗粒度为 {self.granularity} Bytes，必须输入 {self.granularity * 2} 个 Hex 字符！")
                return
                
            data_bytes = bytes.fromhex(clean_hex)
            # 因为 Hex 框显示时可能考虑了端序，写入时需要按 Little-Endian 翻转
            data_bytes = data_bytes[::-1] 
            
            # Item Size = granularity, Count = 1
            req_pld = bytearray(struct.pack('<IBH', target_addr, self.granularity, 1))
            req_pld.extend(data_bytes)
            
            self.hermes.send_frame(0x01, self.base_cmd + 1, bytes(req_pld), priority=0)
            self.log(f"局部覆写指令已下发 -> Addr: 0x{target_addr:08X} | RAW: {clean_hex}", "blue")

    # =========================================================
    # 总线事件响应
    # =========================================================
    def on_bus_event(self, ev: dict):
        if ev['type'] != 'DL' or ev['dir'] != 'RX' or not ev['dl_crc_ok']: return
        cmd = ev['dl_cmd']
        payload = ev['dl_payload']

        # 响应读取 (READ)
        if cmd == self.base_cmd and self.is_reading:
            if len(payload) < 1: return
            status = payload[0]
            
            if status == 0:
                data = payload[1:]
                end_idx = self.read_offset + len(data)
                self.memory_buffer[self.read_offset : end_idx] = data
                self.read_offset = end_idx
                self._request_next_chunk()
            else:
                self.is_reading = False
                self.btn_read.setText("🔍 提取并刷新全局内存区域 (Read All)")
                self.btn_read.setEnabled(True)
                self.log(f"❌ 内存读取被下位机拦截！地址可能越界或处于沙箱外", "red")

        # 响应局部写入 (WRITE ACK)
        elif cmd == self.base_cmd + 1:
            if len(payload) >= 1:
                if payload[0] == 0:
                    self.log("✅ 局部内存修改成功！", "green")
                    # 写完立刻触发一次全局刷新确权
                    self.cmd_start_read_all()
                else:
                    self.log("❌ 写入失败，可能处于 ReadOnly 沙箱或越界", "red")

    # =========================================================
    # 辅助功能：导入/导出
    # =========================================================
    def action_save_cfg(self):
        path, _ = QFileDialog.getSaveFileName(self, "保存 Cast 配置", "argos_casts.json", "JSON Files (*.json)")
        if path:
            with open(path, 'w') as f:
                json.dump(self.casts, f, indent=4)
            self.log("Cast 配置文件已保存", "green")

    def action_load_cfg(self):
        path, _ = QFileDialog.getOpenFileName(self, "载入 Cast 配置", "", "JSON Files (*.json)")
        if path:
            try:
                with open(path, 'r') as f:
                    # JSON keys 是 string，需转回 int
                    raw_dict = json.load(f)
                    self.casts = {int(k): v for k, v in raw_dict.items()}
                self.table.clearSpans()
                self.render_table()
                self.log("Cast 配置文件已加载", "green")
            except Exception as e:
                self.log(f"加载失败: {e}", "red")

    def action_export_csv(self):
        if not self.memory_buffer:
            QMessageBox.warning(self, "为空", "没有可导出的内存数据！")
            return
            
        path, _ = QFileDialog.getSaveFileName(self, "导出为 CSV", f"mem_{self.target_addr:08X}.csv", "CSV Files (*.csv)")
        if path:
            try:
                with open(path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    # 写入表头
                    headers = ["Address"] + [self.table.horizontalHeaderItem(i).text() for i in range(self.table.columnCount())]
                    writer.writerow(headers)
                    
                    # 写入数据
                    for r in range(self.table.rowCount()):
                        row_data = [self.table.verticalHeaderItem(r).text()]
                        for c in range(self.table.columnCount()):
                            item = self.table.item(r, c)
                            # 如果该列是被 Span 吃掉的空白列，填入留空符
                            row_data.append(item.text() if item else "")
                        writer.writerow(row_data)
                self.log("✅ CSV 导出成功", "green")
            except Exception as e:
                self.log(f"❌ 导出失败: {e}", "red")

    def apply_global_cast(self, type_name):
        """将当前读取的整个内存段，全局格式化为指定的同构数组"""
        fmt, c_len = CAST_TYPES[type_name]
        
        # 全局应用前，先清空零散的配置
        self.casts.clear() 
        
        # 按照类型的字节长度，铺满整个内存段
        for offset in range(0, self.total_length, c_len):
            if offset + c_len <= self.total_length:
                self.casts[offset] = {'type': type_name, 'active': True}
                
        self.table.clearSpans()
        self.render_table()
        self.log(f"✅ 已将整个内存区域全局强转为 {type_name} 数组", "green")

    def clear_all_casts(self, checked=False):
        """一键清除所有强转效果，恢复原始 Hex 视图"""
        self.casts.clear()
        self.table.clearSpans()
        self.render_table()
        self.log("🗑️ 已清除所有强转效果", "gray")
