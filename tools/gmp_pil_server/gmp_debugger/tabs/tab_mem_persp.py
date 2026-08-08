import os
import json
import struct
import csv
import re
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                             QLineEdit, QPushButton, QLabel, QFormLayout, 
                             QTableWidget, QTableWidgetItem, QHeaderView, 
                             QFileDialog, QMessageBox, QComboBox, QMenu, QInputDialog)
from PyQt5.QtCore import Qt, pyqtSignal
from core_datalink import HermesDatalinkQt
from resource_discovery import MemoryRegion, ResourceDiscovery

# Supported casts and their little-endian struct formats.
CAST_TYPES = {
    'F32': ('<f', 4), 'I32': ('<i', 4), 'U32': ('<I', 4),
    'I16': ('<h', 2), 'U16': ('<H', 2), 'I8': ('<b', 1), 'U8': ('<B', 1)
}

class TabMemPersp(QWidget):
    def __init__(self, hermes: HermesDatalinkQt, discovery: ResourceDiscovery):
        super().__init__()
        self.hermes = hermes
        self.discovery = discovery
        self.hermes.sig_bus_event.connect(self.on_bus_event)
        self.discovery.memory_regions_changed.connect(self._on_regions_changed)
        
        self.base_cmd = 0x50
        self.target_addr = 0x00000000
        self.total_length = 256  # Default read length in bytes.
        self.granularity = 1     # One, two, or four bytes per displayed item.
        
        self.memory_buffer = bytearray()
        
        # Per-offset cast configuration: {offset: {"type": "F32", "active": True}}.
        self.casts = {}
        
        # Chunked read state.
        self.is_reading = False
        self.read_offset = 0
        self.read_chunk_bytes = 128  # Bound each request to avoid bus monopolization.
        
        self._setup_ui()

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)

        # ==========================================
        # Configuration and export controls.
        # ==========================================
        ctrl_group = QGroupBox("Argos Memory Perspective")
        ctrl_layout = QVBoxLayout()
        ctrl_layout.setSpacing(10)
        
        # Service command, target discovery, granularity, and export.
        row1_layout = QHBoxLayout()
        row1_layout.addWidget(QLabel("Base CMD:"))
        self.edit_cmd = QLineEdit("0x50")
        self.edit_cmd.setMaximumWidth(60)
        row1_layout.addWidget(self.edit_cmd)

        self.cb_regions = QComboBox()
        self.cb_regions.setMinimumWidth(220)
        self.cb_regions.currentIndexChanged.connect(self._apply_selected_region)
        row1_layout.addWidget(QLabel("  Target region:"))
        row1_layout.addWidget(self.cb_regions)
        self.btn_discover = QPushButton("Discover Regions")
        self.btn_discover.clicked.connect(self.action_discover_regions)
        row1_layout.addWidget(self.btn_discover)

        row1_layout.addWidget(QLabel("  Read granularity:"))
        self.cb_granularity = QComboBox()
        self.cb_granularity.addItems(["1 Byte (8-bit)", "2 Bytes (16-bit)", "4 Bytes (32-bit)"])
        row1_layout.addWidget(self.cb_granularity)

        row1_layout.addStretch()
        
        self.btn_export_csv = QPushButton("Export CSV")
        self.btn_export_csv.clicked.connect(self.action_export_csv)
        row1_layout.addWidget(self.btn_export_csv)
        
        # Selected byte address, length, and cast configuration.
        row2_layout = QHBoxLayout()
        row2_layout.addWidget(QLabel("Start address (hex):"))
        self.edit_addr = QLineEdit("0x20000000")
        self.edit_addr.setMaximumWidth(100)
        row2_layout.addWidget(self.edit_addr)
        
        row2_layout.addWidget(QLabel("  Total bytes:"))
        self.edit_len = QLineEdit("256")
        self.edit_len.setMaximumWidth(60)
        row2_layout.addWidget(self.edit_len)
        
        row2_layout.addStretch()
        
        self.btn_load_cfg = QPushButton("Load Casts")
        self.btn_load_cfg.clicked.connect(self.action_load_cfg)
        row2_layout.addWidget(self.btn_load_cfg)
        
        self.btn_save_cfg = QPushButton("Save Casts")
        self.btn_save_cfg.clicked.connect(self.action_save_cfg)
        row2_layout.addWidget(self.btn_save_cfg)

        # Add both control rows to the group.
        ctrl_layout.addLayout(row1_layout)
        ctrl_layout.addLayout(row2_layout)
        ctrl_group.setLayout(ctrl_layout)
        main_layout.addWidget(ctrl_group)

        # ==========================================
        # Memory data grid.
        # ==========================================
        self.table = QTableWidget(0, 16)
        self.table.setStyleSheet("QTableWidget { font-family: Consolas, monospace; font-size: 13px; }")
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        # Use explicit context-menu writes to prevent accidental edits.
        self.table.setContextMenuPolicy(Qt.CustomContextMenu)
        self.table.customContextMenuRequested.connect(self.show_context_menu)
        self.table.cellDoubleClicked.connect(self.on_cell_double_clicked)
        
        main_layout.addWidget(self.table)

        # ==========================================
        # Read action.
        # ==========================================
        btn_layout = QHBoxLayout()
        self.btn_read = QPushButton("Read Selected Memory Region")
        self.btn_read.setMinimumHeight(45)
        self.btn_read.setStyleSheet("background-color: #E3F2FD; font-weight: bold; font-size: 14px;")
        self.btn_read.clicked.connect(self.cmd_start_read_all)
        
        btn_layout.addWidget(self.btn_read)
        main_layout.addLayout(btn_layout)

    def action_discover_regions(self):
        """Request the target Memory Perspective whitelist descriptors."""
        self._sync_config()
        self.discovery.discover_memory(self.base_cmd)

    def _on_regions_changed(self, regions):
        """Populate the selector with target-reported memory regions."""
        self.cb_regions.blockSignals(True)
        self.cb_regions.clear()
        for region in regions:
            self.cb_regions.addItem(region.name, region)
        self.cb_regions.blockSignals(False)
        if regions:
            self.cb_regions.setCurrentIndex(0)
            self._apply_selected_region()
        self.log(f"Discovered {len(regions)} target memory regions.", "green")

    def _apply_selected_region(self):
        """Apply one discovered descriptor without changing the user's cast."""
        region = self.cb_regions.currentData()
        if not isinstance(region, MemoryRegion):
            return
        self.edit_addr.setText(f"0x{region.address:08X}")
        self.edit_len.setText(str(region.byte_length))
        self.target_addr = region.address
        self.total_length = region.byte_length

    def log(self, msg, color="black"):
        if hasattr(self.hermes, "emit_log"):
            self.hermes.emit_log("Memory", msg)
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
    # Chunked read state machine.
    # =========================================================
    def cmd_start_read_all(self):
        if not self.hermes.running or self.is_reading: return
        self._sync_config()
        
        # Align the requested length to the selected item granularity.
        remainder = self.total_length % self.granularity
        if remainder != 0:
            self.total_length += (self.granularity - remainder)
            self.edit_len.setText(str(self.total_length))
            self.log(f"Length aligned to {self.total_length} bytes.", "orange")

        self.memory_buffer = bytearray(self.total_length)
        self.is_reading = True
        self.read_offset = 0
        self.btn_read.setText("Reading memory...")
        self.btn_read.setEnabled(False)
        
        self._request_next_chunk()

    def _request_next_chunk(self):
        if self.read_offset >= self.total_length:
            self.is_reading = False
            self.btn_read.setText("Read Selected Memory Region")
            self.btn_read.setEnabled(True)
            self.render_table()
            self.log("Memory region read complete.", "green")
            return

        remain = self.total_length - self.read_offset
        chunk_bytes = min(self.read_chunk_bytes, remain)
        # Keep each chunk aligned to the selected item size.
        chunk_bytes -= (chunk_bytes % self.granularity)
        
        current_addr = self.target_addr + self.read_offset
        item_count = chunk_bytes // self.granularity

        # Payload: [Address(4B)] + [Item Size(1B)] + [Item Count(2B)]
        req_pld = struct.pack('<IBH', current_addr, self.granularity, item_count)
        self.hermes.send_frame(0x01, self.base_cmd, req_pld, priority=1)

    # =========================================================
    # Data grid rendering and typed casts.
    # =========================================================
    def render_table(self):
        if not self.memory_buffer: return
        
        # Each row spans sixteen physical bytes.
        bytes_per_row = 16
        cols = bytes_per_row // self.granularity
        rows = (self.total_length + bytes_per_row - 1) // bytes_per_row
        
        self.table.clear()
        self.table.setRowCount(rows)
        self.table.setColumnCount(cols)
        
        # Row headers show physical addresses.
        row_labels = [f"0x{self.target_addr + r * bytes_per_row:08X}" for r in range(rows)]
        self.table.setVerticalHeaderLabels(row_labels)
        
        # Column headers show relative byte offsets.
        col_labels = [f"+{c * self.granularity:02X}" for c in range(cols)]
        self.table.setHorizontalHeaderLabels(col_labels)
        
        row, col = 0, 0
        offset = 0
        
        while offset < self.total_length:
            if col >= cols:
                row += 1; col = 0
                
            # Prefer an active typed cast at this offset.
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
                        
                        # Span every grid cell consumed by the cast.
                        col_span = max(1, c_len // self.granularity)
                        
                        self.table.setItem(row, col, item)
                        if col_span > 1:
                            self.table.setSpan(row, col, 1, col_span)
                            
                        offset += c_len
                        col += col_span
                        cast_handled = True
                    except: pass
            
            # Otherwise render an unsigned hexadecimal item.
            if not cast_handled:
                chunk = self.memory_buffer[offset : offset + self.granularity]
                # Decode the selected unsigned little-endian granularity.
                if self.granularity == 1:   txt = f"{chunk[0]:02X}"
                elif self.granularity == 2: txt = f"{struct.unpack('<H', chunk)[0]:04X}"
                elif self.granularity == 4: txt = f"{struct.unpack('<I', chunk)[0]:08X}"
                
                item = QTableWidgetItem(txt)
                item.setTextAlignment(Qt.AlignCenter)
                item.setData(Qt.UserRole, offset)
                
                self.table.setItem(row, col, item)
                
                offset += self.granularity
                col += 1

    # =========================================================
    # Context menu and atomic local writes.
    # =========================================================
    def show_context_menu(self, pos):
        item = self.table.itemAt(pos)
        if not item: return
        
        # Start with the byte offset under the pointer.
        offset = item.data(Qt.UserRole)
        
        # Use the first offset when the pointer is inside a selection.
        selected_items = self.table.selectedItems()
        if len(selected_items) > 1 and item in selected_items:
            valid_offsets = [it.data(Qt.UserRole) for it in selected_items if it.data(Qt.UserRole) is not None]
            if valid_offsets:
                offset = min(valid_offsets)
        
        menu = QMenu(self)
        
        # Local cast submenu.
        cast_menu = menu.addMenu("Cast Item To...")
        for t in CAST_TYPES.keys():
            action = cast_menu.addAction(t)
            action.triggered.connect(lambda checked, t_name=t, o=offset: self.apply_cast(o, t_name))
            
        # Homogeneous whole-region cast submenu.
        global_cast_menu = menu.addMenu("Cast Entire Region To...")
        for t in CAST_TYPES.keys():
            action = global_cast_menu.addAction(t)
            action.triggered.connect(lambda checked, t_name=t: self.apply_global_cast(t_name))
            
        menu.addSeparator()
            
        # Cast removal actions.
        if offset in self.casts:
            action_rm = menu.addAction("Remove Item Cast")
            action_rm.triggered.connect(lambda checked, o=offset: self.remove_cast(o))
            
        if self.casts:
            action_rm_all = menu.addAction("Clear All Casts")
            action_rm_all.triggered.connect(self.clear_all_casts)
            
        menu.addSeparator()
        
        # Explicit write action.
        action_edit = menu.addAction("Edit and Write Value")
        action_edit.triggered.connect(lambda checked, o=offset: self.edit_and_write_value(o))
        
        menu.exec_(self.table.viewport().mapToGlobal(pos))

    def on_cell_double_clicked(self, row, col):
        item = self.table.item(row, col)
        if not item: return
        offset = item.data(Qt.UserRole)
        
        # Double-click opens the same guarded write dialog.
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
            self.log("Open the serial port before writing memory.", "red")
            return
            
        target_addr = self.target_addr + offset
        
        # Write a typed cast value.
        if offset in self.casts and self.casts[offset]['active']:
            c_type = self.casts[offset]['type']
            fmt, c_len = CAST_TYPES[c_type]
            
            prompt_str = f"Enter the new {c_type} value:"
            new_val_str, ok = QInputDialog.getText(self, "Write Memory", prompt_str)
            if not ok or not new_val_str.strip(): return
            
            try:
                val = float(new_val_str) if 'F' in c_type else int(new_val_str)
                data_bytes = struct.pack(fmt, val)
                
                # A typed write contains one item of the cast width.
                req_pld = bytearray(struct.pack('<IBH', target_addr, c_len, 1))
                req_pld.extend(data_bytes)
                
                self.hermes.send_frame(0x01, self.base_cmd + 1, bytes(req_pld), priority=0)
                self.log(f"Write sent: address 0x{target_addr:08X}, value {val}.", "blue")
            except Exception as e:
                QMessageBox.critical(self, "Invalid Format", f"The value is not valid for {c_type}.\n{e}")
                
        # Write a normal hexadecimal grid item.
        else:
            prompt_str = f"Enter exactly {self.granularity * 2} hexadecimal digits without 0x:"
            new_val_str, ok = QInputDialog.getText(self, "Write Memory", prompt_str)
            if not ok or not new_val_str.strip(): return
            
            clean_hex = re.sub(r'[^0-9a-fA-F]', '', new_val_str)
            if len(clean_hex) != self.granularity * 2:
                QMessageBox.critical(self, "Invalid Length", f"A {self.granularity}-byte item requires {self.granularity * 2} hexadecimal digits.")
                return
                
            data_bytes = bytes.fromhex(clean_hex)
            # Convert the displayed big-endian hexadecimal text to wire little endian.
            data_bytes = data_bytes[::-1] 
            
            # Item Size = granularity, Count = 1
            req_pld = bytearray(struct.pack('<IBH', target_addr, self.granularity, 1))
            req_pld.extend(data_bytes)
            
            self.hermes.send_frame(0x01, self.base_cmd + 1, bytes(req_pld), priority=0)
            self.log(f"Write sent: address 0x{target_addr:08X}, raw {clean_hex}.", "blue")

    # =========================================================
    # Bus response handling.
    # =========================================================
    def on_bus_event(self, ev: dict):
        if ev['type'] != 'DL' or ev['dir'] != 'RX' or not ev['dl_crc_ok']: return
        cmd = ev['dl_cmd']
        payload = ev['dl_payload']

        # Read response.
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
                self.btn_read.setText("Read Selected Memory Region")
                self.btn_read.setEnabled(True)
                self.log("The target rejected an out-of-range memory read.", "red")

        # Write acknowledgement. Discovery responses are longer and ignored here.
        elif cmd == self.base_cmd + 1:
            if len(payload) == 1:
                if payload[0] == 0:
                    self.log("Memory write completed.", "green")
                    # Read back the complete selection for confirmation.
                    self.cmd_start_read_all()
                else:
                    self.log("Memory write was rejected as read-only or out of range.", "red")

    # =========================================================
    # Import and export helpers.
    # =========================================================
    def action_save_cfg(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Cast Configuration", "argos_casts.json", "JSON Files (*.json)")
        if path:
            with open(path, 'w') as f:
                json.dump(self.casts, f, indent=4)
            self.log("Cast configuration saved.", "green")

    def action_load_cfg(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Cast Configuration", "", "JSON Files (*.json)")
        if path:
            try:
                with open(path, 'r') as f:
                    # JSON object keys must be converted back to integer offsets.
                    raw_dict = json.load(f)
                    self.casts = {int(k): v for k, v in raw_dict.items()}
                self.table.clearSpans()
                self.render_table()
                self.log("Cast configuration loaded.", "green")
            except Exception as e:
                self.log(f"Failed to load cast configuration: {e}", "red")

    def action_export_csv(self):
        if not self.memory_buffer:
            QMessageBox.warning(self, "No Data", "Read a memory region before exporting.")
            return
            
        path, _ = QFileDialog.getSaveFileName(self, "Export CSV", f"mem_{self.target_addr:08X}.csv", "CSV Files (*.csv)")
        if path:
            try:
                with open(path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    # Write table headers.
                    headers = ["Address"] + [self.table.horizontalHeaderItem(i).text() for i in range(self.table.columnCount())]
                    writer.writerow(headers)
                    
                    # Write visible grid data.
                    for r in range(self.table.rowCount()):
                        row_data = [self.table.verticalHeaderItem(r).text()]
                        for c in range(self.table.columnCount()):
                            item = self.table.item(r, c)
                            # Spanned cells are represented by empty CSV fields.
                            row_data.append(item.text() if item else "")
                        writer.writerow(row_data)
                self.log("CSV export completed.", "green")
            except Exception as e:
                self.log(f"CSV export failed: {e}", "red")

    def apply_global_cast(self, type_name):
        """Format the complete selection as a homogeneous typed array."""
        fmt, c_len = CAST_TYPES[type_name]
        
        # Replace all previous local casts.
        self.casts.clear() 
        
        # Tile the region using the selected type width.
        for offset in range(0, self.total_length, c_len):
            if offset + c_len <= self.total_length:
                self.casts[offset] = {'type': type_name, 'active': True}
                
        self.table.clearSpans()
        self.render_table()
        self.log(f"Entire memory region cast to a {type_name} array.", "green")

    def clear_all_casts(self, checked=False):
        """Clear every typed cast and restore the hexadecimal view."""
        self.casts.clear()
        self.table.clearSpans()
        self.render_table()
        self.log("All casts cleared.", "gray")
