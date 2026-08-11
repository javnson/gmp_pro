import os
import json
import re
import struct
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                             QLineEdit, QPushButton, QLabel, QFormLayout, 
                             QTableWidget, QTableWidgetItem, QHeaderView, 
                             QFileDialog, QDialog, QTextEdit, QMessageBox, 
                             QCheckBox, QDoubleSpinBox, QTabWidget, QInputDialog,
                             QComboBox, QMenu)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from core_datalink import HermesDatalinkQt
from resource_discovery import ResourceDiscovery

# Parameter types and their little-endian wire formats.
TYPE_MAP = {
    'GMP_PARAM_TYPE_U16': ('<H', 2),
    'GMP_PARAM_TYPE_I16': ('<h', 2),
    'GMP_PARAM_TYPE_U32': ('<I', 4),
    'GMP_PARAM_TYPE_I32': ('<i', 4),
    'GMP_PARAM_TYPE_F32': ('<f', 4)
}

# =========================================================
# C source parsing helpers.
# =========================================================
def strip_c_comments(text: str) -> str:
    """Remove C block and line comments before dictionary parsing."""
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    text = re.sub(r'//.*', '', text)
    return text

# =========================================================
# C tunable dictionary parser dialog.
# =========================================================
class CCodeParserDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Import Tunable Dictionary from C")
        self.resize(600, 400)
        self.parsed_data = []
        
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Paste a target const gmp_param_item_t array definition:"))
        
        self.txt_code = QTextEdit()
        self.txt_code.setPlaceholderText("""Example:
const gmp_param_item_t dict_m1[] = {
    { &m1.kp, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW },
    { &m1.speed, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO },
};""")
        layout.addWidget(self.txt_code)
        
        btn_layout = QHBoxLayout()
        self.btn_parse = QPushButton("Parse Dictionary")
        self.btn_parse.clicked.connect(self.parse_code)
        btn_layout.addStretch()
        btn_layout.addWidget(self.btn_parse)
        layout.addLayout(btn_layout)

    def parse_code(self):
        code = self.txt_code.toPlainText()
        code = strip_c_comments(code)
        
        pattern = (r'\{\s*&\s*([^,]+?)\s*,\s*(GMP_PARAM_TYPE_[A-Z0-9_]+)\s*,'
                   r'\s*(GMP_PARAM_PERM_[A-Z]+)'
                   r'(?:\s*,\s*"([^"]*)"\s*,\s*"([^"]*)")?\s*\}')
        matches = re.findall(pattern, code)
        
        if not matches:
            QMessageBox.warning(self, "Parse Failed", "No supported dictionary entries were found.")
            return
            
        self.parsed_data = []
        for i, match in enumerate(matches):
            var_name, var_type, var_perm, display_name, unit = match
            if var_type not in TYPE_MAP:
                QMessageBox.warning(self, "Unsupported Type", f"Unsupported parameter type: {var_type}")
                return
                
            self.parsed_data.append({
                "id": i,
                "name": display_name or var_name.strip(),
                "unit": unit,
                "type": var_type,
                "perm": var_perm,
                "display_hex": False, 
                "enum_map": None      
            })
            
        QMessageBox.information(self, "Import Complete", f"Imported {len(self.parsed_data)} parameters.")
        self.accept()

# =========================================================
# C enum parser dialog.
# =========================================================
class CEnumParserDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Bind a C Enum")
        self.resize(500, 400)
        self.parsed_map = {}
        
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Paste a C enum definition, including implicit incremented values:"))
        
        self.txt_code = QTextEdit()
        self.txt_code.setPlaceholderText("""typedef enum {
    CIA402_CMD_NULL = 0,
    CIA402_CMD_DISABLE_VOLTAGE = 1,
    CIA402_CMD_SHUTDOWN,
    CIA402_CMD_SWITCHON
} cia402_cmd_t;""")
        layout.addWidget(self.txt_code)
        
        btn_layout = QHBoxLayout()
        self.btn_parse = QPushButton("Parse and Apply")
        self.btn_parse.clicked.connect(self.parse_code)
        btn_layout.addStretch()
        btn_layout.addWidget(self.btn_parse)
        layout.addLayout(btn_layout)

    def parse_code(self):
        code = self.txt_code.toPlainText()
        code = strip_c_comments(code)
        
        match = re.search(r'\{([^}]+)\}', code)
        if not match:
            QMessageBox.warning(self, "Parse Failed", "No enum body enclosed by braces was found.")
            return
        
        body = match.group(1)
        current_val = 0
        self.parsed_map = {}
        
        for item in body.split(','):
            item = item.strip()
            if not item: continue
            if '=' in item:
                name, val_str = item.split('=', 1)
                name = name.strip()
                val_str = val_str.strip()
                if val_str.lower().startswith('0x'):
                    current_val = int(val_str, 16)
                else:
                    current_val = int(val_str)
                self.parsed_map[current_val] = name
            else:
                name = item.strip()
                self.parsed_map[current_val] = name
            current_val += 1
            
        if not self.parsed_map:
            QMessageBox.warning(self, "Empty Enum", "No valid enum entries were parsed.")
            return
            
        self.accept()

# =========================================================
# Guarded parameter editing controls.
# =========================================================

class ParamLineEdit(QLineEdit):
    sig_user_editing = pyqtSignal()
    sig_write_requested = pyqtSignal()
    sig_edit_aborted = pyqtSignal()

    def __init__(self, param_id, is_ro, parent=None):
        super().__init__(parent)
        self.param_id = param_id
        self.is_permanently_ro = is_ro
        self.is_dirty = False 
        
        # A double-click is required before a writable value can be edited.
        self.setReadOnly(True) 
        
        # Each editor owns its status-color timer.
        self.fade_timer = QTimer(self)
        self.fade_timer.setSingleShot(True)
        self.fade_timer.timeout.connect(self._restore_style)

        if self.is_permanently_ro:
            self.setStyleSheet("background-color: #F5F5F5; color: #757575;")
        else:
            self.textEdited.connect(self._on_text_edited)
            self.returnPressed.connect(self._on_return_pressed)

    def mouseDoubleClickEvent(self, event):
        """Unlock writable values only on a deliberate double-click."""
        if not self.is_permanently_ro:
            self.setReadOnly(False)
            self.setStyleSheet("background-color: #E3F2FD; color: #0D47A1; font-weight: bold;")
            self.setFocus()
            self.selectAll()
            self.sig_user_editing.emit()
        super().mouseDoubleClickEvent(event)

    def focusOutEvent(self, event):
        super().focusOutEvent(event)
        if self.is_dirty:
            self.is_dirty = False
            self.sig_edit_aborted.emit() 
            
        if not self.is_permanently_ro:
            self.setReadOnly(True)  # Restore the guard after focus is lost.
        self._restore_style()

    def _on_text_edited(self):
        self.is_dirty = True
        self.setStyleSheet("background-color: #FFE082; color: #E65100; font-weight: bold;")

    def _on_return_pressed(self):
        if self.is_dirty:
            self.is_dirty = False
            self.clearFocus()
            self.sig_write_requested.emit()

    def confirm_write(self):
        self.is_dirty = False
        self.clearFocus()
        self.sig_write_requested.emit()

    def _restore_style(self):
        if self.hasFocus() and not self.isReadOnly(): return
        if self.is_permanently_ro:
            self.setStyleSheet("background-color: #F5F5F5; color: #757575;")
        else:
            self.setStyleSheet("")

    def flash_status(self, changed):
        """Extend the status timer when refreshes overlap."""
        if self.hasFocus() and not self.isReadOnly(): return
        
        if changed:
            self.setStyleSheet("background-color: #FFF59D; color: #D32F2F; font-weight: bold;")
        else:
            self.setStyleSheet("background-color: #C8E6C9; color: black;")
            
        self.fade_timer.start(1000)

    def update_display_value(self, val, is_float=False, is_hex=False):
        if self.hasFocus() and not self.isReadOnly(): return False
        if self.is_dirty: return False
        
        if is_float: txt = f"{val:.4f}"
        else: txt = f"0x{int(val):X}" if is_hex else str(int(val))
        self.setText(txt)
        return True

    def get_write_value(self):
        return self.text()


class ParamComboBox(QComboBox):
    sig_user_editing = pyqtSignal()
    sig_write_requested = pyqtSignal()
    sig_edit_aborted = pyqtSignal()

    def __init__(self, param_id, is_ro, enum_map, parent=None):
        super().__init__(parent)
        self.param_id = param_id
        self.is_ro = is_ro
        self.enum_map = enum_map
        self.is_popup_open = False 
        
        self.fade_timer = QTimer(self)
        self.fade_timer.setSingleShot(True)
        self.fade_timer.timeout.connect(self._restore_style)
        
        self.addItem("-", "")
        
        for val, name in self.enum_map.items():
            self.addItem(f"{name} ({val})", val)
            
        if self.is_ro:
            self.setEnabled(False)
            self.setStyleSheet("background-color: #F5F5F5; color: #757575;")
        else:
            self.activated.connect(self._on_activated)

    def mousePressEvent(self, event):
        """Let a single click select the table row without opening the list."""
        event.ignore()

    def mouseDoubleClickEvent(self, event):
        """Open the enum list only on a deliberate double-click."""
        if not self.is_ro:
            self.showPopup()

    def showPopup(self):
        if not self.is_ro:
            self.sig_user_editing.emit() 
        super().showPopup()
        self.is_popup_open = True

    def hidePopup(self):
        super().hidePopup()
        self.is_popup_open = False
        if not self.is_ro:
            self.sig_edit_aborted.emit()
        self._restore_style()

    def _on_activated(self, index):
        data = self.itemData(index)
        if data != "":
            self.confirm_write()
        else:
            self.clearFocus()

    def confirm_write(self):
        self.clearFocus()
        self.sig_write_requested.emit()

    def _restore_style(self):
        if self.is_popup_open: return
        if self.is_ro:
            self.setStyleSheet("background-color: #F5F5F5; color: #757575;")
        else:
            self.setStyleSheet("")

    def flash_status(self, changed):
        if self.is_popup_open: return
        if changed:
            self.setStyleSheet("background-color: #FFF59D; color: #D32F2F; font-weight: bold;")
        else:
            self.setStyleSheet("background-color: #C8E6C9; color: black;")
        self.fade_timer.start(1000)

    def update_display_value(self, val, is_float=False, is_hex=False):
        if self.is_popup_open: return False
        
        val_int = int(val)
        idx = self.findData(val_int)
        
        if idx >= 0:
            self.setCurrentIndex(idx)
        else:
            self.addItem(f"UNKNOWN_STATE ({val_int})", val_int)
            self.setCurrentIndex(self.count() - 1)
        return True

    def get_write_value(self):
        data = self.currentData()
        return str(data) if data != "" else None


# =========================================================
# One tunable service instance.
# =========================================================
class TunableInstanceWidget(QWidget):
    sig_bus_busy = pyqtSignal(bool)

    def __init__(self, hermes: HermesDatalinkQt, discovery: ResourceDiscovery,
                 instance_name: str, base_cmd: int = 0x30):
        super().__init__()
        self.hermes = hermes
        self.discovery = discovery
        self.instance_name = instance_name
        self.base_cmd = base_cmd
        
        self.current_params = [] 
        self.last_values = {}
        self.pending_read_batches = []
        self.awaiting_discovery = False
        self.discovery.tunable_items_changed.connect(self._on_tunable_items_changed)
        
        self.auto_timer = QTimer()
        self.auto_timer.timeout.connect(self.cmd_read_all)

        self.busy_timer = QTimer()
        self.busy_timer.setSingleShot(True)
        self.busy_timer.timeout.connect(self._force_release_bus)
        
        self._setup_ui()

    def _set_bus_occupied(self, state: bool):
        self.sig_bus_busy.emit(state)
        if state:
            self.busy_timer.start(400)
        else:
            self.busy_timer.stop()

    def _force_release_bus(self):
        self.sig_bus_busy.emit(False)
        self.log("Bus ownership timed out; a response may have been lost.", "orange")

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)

        ctrl_group = QGroupBox(f"{self.instance_name} Configuration")
        ctrl_layout = QHBoxLayout()
        
        ctrl_layout.addWidget(QLabel("Base CMD (Hex):"))
        self.edit_cmd = QLineEdit(hex(self.base_cmd))
        self.edit_cmd.setMaximumWidth(60)
        self.edit_cmd.textChanged.connect(self._update_base_cmd)
        ctrl_layout.addWidget(self.edit_cmd)
        
        self.btn_import_c = QPushButton("Import C")
        self.btn_import_c.clicked.connect(self.action_import_c)
        ctrl_layout.addWidget(self.btn_import_c)

        self.btn_discover = QPushButton("Discover from Target")
        self.btn_discover.clicked.connect(self.action_discover_target)
        ctrl_layout.addWidget(self.btn_discover)
        
        self.btn_load_json = QPushButton("Load JSON")
        self.btn_load_json.clicked.connect(self.action_load_json)
        ctrl_layout.addWidget(self.btn_load_json)
        
        self.btn_save_json = QPushButton("Save JSON")
        self.btn_save_json.clicked.connect(self.action_save_json)
        ctrl_layout.addWidget(self.btn_save_json)
        
        ctrl_layout.addStretch()
        
        self.cb_auto_refresh = QCheckBox("Automatic refresh")
        self.cb_auto_refresh.stateChanged.connect(self._toggle_auto_refresh)
        ctrl_layout.addWidget(self.cb_auto_refresh)
        
        self.spin_interval = QDoubleSpinBox()
        self.spin_interval.setRange(0.1, 10.0)
        self.spin_interval.setValue(2.0)
        self.spin_interval.setSuffix(" s")
        self.spin_interval.valueChanged.connect(self._update_timer_interval)
        ctrl_layout.addWidget(self.spin_interval)
        
        ctrl_group.setLayout(ctrl_layout)
        main_layout.addWidget(ctrl_group)

        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(["ID", "Parameter", "Type", "Permission", "Current Value", "Actions"])
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)
        
        self.table.setContextMenuPolicy(Qt.CustomContextMenu)
        self.table.customContextMenuRequested.connect(self.show_context_menu)
        
        main_layout.addWidget(self.table)

        btn_layout = QHBoxLayout()
        self.btn_read_all = QPushButton("Read All Parameters")
        self.btn_read_all.setMinimumHeight(40)
        self.btn_read_all.setStyleSheet("background-color: #E3F2FD; font-weight: bold;")
        self.btn_read_all.clicked.connect(lambda: self.cmd_read_all(False))
        
        btn_layout.addWidget(self.btn_read_all)
        main_layout.addLayout(btn_layout)

    def action_discover_target(self):
        """Import the tunable dictionary reported by the connected target."""
        self._update_base_cmd()
        self.awaiting_discovery = True
        self.discovery.discover_tunables(self.base_cmd)

    def _on_tunable_items_changed(self, items):
        """Replace this instance dictionary with discovered target metadata."""
        if not self.awaiting_discovery:
            return
        self.awaiting_discovery = False
        type_names = {
            0: "GMP_PARAM_TYPE_U16",
            1: "GMP_PARAM_TYPE_I16",
            2: "GMP_PARAM_TYPE_U32",
            3: "GMP_PARAM_TYPE_I32",
            4: "GMP_PARAM_TYPE_F32",
        }
        self.current_params = [
            {
                "id": item.item_id,
                "name": item.name,
                "unit": item.unit,
                "type": type_names.get(item.data_type, "GMP_PARAM_TYPE_U16"),
                "perm": "GMP_PARAM_PERM_RW" if item.permission else "GMP_PARAM_PERM_RO",
                "display_hex": False,
                "enum_map": None,
            }
            for item in items
        ]
        self.last_values.clear()
        self._render_table()
        self.log(f"Discovered {len(items)} target tunable parameters.", "green")

    def log(self, msg, color="black"):
        if hasattr(self.hermes, "emit_log"):
            self.hermes.emit_log("Tunable", f"[{self.instance_name}] {msg}")
        else:
            print(msg)

    def show_context_menu(self, pos):
        item = self.table.itemAt(pos)
        if not item: return
        
        row = item.row()
        param_id_str = self.table.item(row, 0).text()
        param_id = int(param_id_str)
        param_info = next((p for p in self.current_params if p['id'] == param_id), None)
        if not param_info: return

        menu = QMenu(self)
        is_float = 'F32' in param_info['type']
        
        if not is_float:
            is_hex = param_info.get('display_hex', False)
            hex_action = menu.addAction("Disable Hexadecimal Display" if is_hex else "Enable Hexadecimal Display")
            hex_action.triggered.connect(lambda: self.toggle_hex_display(param_id))
            menu.addSeparator()

        enum_action = menu.addAction("Bind C Enum")
        enum_action.triggered.connect(lambda: self.bind_enum(param_id))
        
        if param_info.get('enum_map'):
            clear_enum_action = menu.addAction("Remove Enum Binding")
            clear_enum_action.triggered.connect(lambda: self.clear_enum(param_id))

        menu.exec_(self.table.viewport().mapToGlobal(pos))

    # =========================================================
    # Pause polling while rebuilding editor widgets.
    # =========================================================
    def toggle_hex_display(self, param_id):
        param_info = next((p for p in self.current_params if p['id'] == param_id), None)
        if param_info:
            was_running = self.auto_timer.isActive()
            if was_running: self.auto_timer.stop()
            
            param_info['display_hex'] = not param_info.get('display_hex', False)
            self._render_table()
            self.log(f"Changed the display mode for [{param_info['name']}].", "blue")
            
            if was_running and self.cb_auto_refresh.isChecked():
                self.auto_timer.start(int(self.spin_interval.value() * 1000))

    def bind_enum(self, param_id):
        param_info = next((p for p in self.current_params if p['id'] == param_id), None)
        if not param_info: return
        
        was_running = self.auto_timer.isActive()
        if was_running: self.auto_timer.stop()
        
        dialog = CEnumParserDialog(self)
        if dialog.exec_() == QDialog.Accepted and dialog.parsed_map:
            param_info['enum_map'] = dialog.parsed_map
            param_info['display_hex'] = False 
            self._render_table()
            self.log(f"Bound {len(dialog.parsed_map)} enum values to [{param_info['name']}].", "green")
            
        if was_running and self.cb_auto_refresh.isChecked():
            self.auto_timer.start(int(self.spin_interval.value() * 1000))

    def clear_enum(self, param_id):
        param_info = next((p for p in self.current_params if p['id'] == param_id), None)
        if param_info:
            was_running = self.auto_timer.isActive()
            if was_running: self.auto_timer.stop()
            
            param_info['enum_map'] = None
            self._render_table()
            self.log(f"Removed the enum binding from [{param_info['name']}].", "gray")
            
            if was_running and self.cb_auto_refresh.isChecked():
                self.auto_timer.start(int(self.spin_interval.value() * 1000))

    def action_import_c(self):
        was_running = self.auto_timer.isActive()
        if was_running: self.auto_timer.stop()
        
        dialog = CCodeParserDialog(self)
        if dialog.exec_() == QDialog.Accepted and dialog.parsed_data:
            self.current_params = dialog.parsed_data
            self.last_values.clear()
            self._render_table()
            
        if was_running and self.cb_auto_refresh.isChecked():
            self.auto_timer.start(int(self.spin_interval.value() * 1000))

    def action_load_json(self):
        was_running = self.auto_timer.isActive()
        if was_running: self.auto_timer.stop()
        
        path, _ = QFileDialog.getOpenFileName(self, "Load Tunable Configuration", "", "JSON Files (*.json)")
        if path:
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self.base_cmd = data.get("base_cmd", self.base_cmd)
                    self.edit_cmd.setText(hex(self.base_cmd))
                    self.spin_interval.setValue(data.get("refresh_interval", 2.0))
                    self.cb_auto_refresh.setChecked(data.get("auto_refresh", False))
                    self.current_params = data.get("params", [])
                    
                    for p in self.current_params:
                        if p.get('enum_map'):
                            p['enum_map'] = {int(k): v for k, v in p['enum_map'].items()}
                            
                    self.last_values.clear()
                    self._render_table()
                self.log(f"Loaded configuration: {os.path.basename(path)}", "green")
            except Exception as e:
                self.log(f"Failed to load JSON: {e}", "red")
                
        if was_running and self.cb_auto_refresh.isChecked():
            self.auto_timer.start(int(self.spin_interval.value() * 1000))

    def _update_base_cmd(self):
        try:
            self.base_cmd = int(self.edit_cmd.text(), 16)
        except ValueError: pass

    def _update_timer_interval(self):
        if self.auto_timer.isActive():
            self.auto_timer.setInterval(int(self.spin_interval.value() * 1000))

    def _toggle_auto_refresh(self, state):
        if state == Qt.Checked:
            self.auto_timer.start(int(self.spin_interval.value() * 1000))
            self.log(f"Automatic refresh started ({self.spin_interval.value()} s).", "blue")
        else:
            self.auto_timer.stop()
            self.log("Automatic refresh stopped.", "gray")

    def stop_timers(self):
        if self.auto_timer.isActive():
            self.auto_timer.stop()

    def action_save_json(self):
        if not self.current_params:
            QMessageBox.warning(self, "No Parameters", "There is no parameter dictionary to export.")
            return
        default_name = f"tunable_{self.instance_name.replace(' ', '_').replace('/', '_')}.json"
        path, _ = QFileDialog.getSaveFileName(self, "Save Tunable Configuration", default_name, "JSON Files (*.json)")
        if path:
            data = {
                "base_cmd": self.base_cmd,
                "auto_refresh": self.cb_auto_refresh.isChecked(),
                "refresh_interval": self.spin_interval.value(),
                "params": self.current_params
            }
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4)
            self.log(f"Configuration saved to: {os.path.basename(path)}", "green")

    def _render_table(self):
        self.table.clearContents()
        self.table.setRowCount(len(self.current_params))
        
        for row, p in enumerate(self.current_params):
            item_id = QTableWidgetItem(str(p['id']))
            item_id.setTextAlignment(Qt.AlignCenter)
            item_id.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.table.setItem(row, 0, item_id)
            
            display_name = p['name']
            if p.get('unit'):
                display_name += f" [{p['unit']}]"
            item_name = QTableWidgetItem(display_name)
            item_name.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.table.setItem(row, 1, item_name)
            
            self.table.setItem(row, 2, QTableWidgetItem(p['type'].replace("GMP_PARAM_TYPE_", "")))
            self.table.setItem(row, 3, QTableWidgetItem("ReadOnly" if "RO" in p['perm'] else "Read/Write"))
            
            if p.get('enum_map'):
                val_edit = ParamComboBox(p['id'], "RO" in p['perm'], p['enum_map'])
            else:
                val_edit = ParamLineEdit(p['id'], "RO" in p['perm'])
                
            val_edit.sig_user_editing.connect(self.on_user_editing)
            val_edit.sig_edit_aborted.connect(self.on_edit_aborted)
            val_edit.sig_write_requested.connect(lambda w=val_edit: self.request_write(w))
            
            self.table.setCellWidget(row, 4, val_edit)
            
            action_widget = QWidget()
            action_layout = QHBoxLayout(action_widget)
            action_layout.setContentsMargins(2, 2, 2, 2)
            
            btn_read = QPushButton("R")
            btn_read.setMaximumWidth(30)
            btn_read.clicked.connect(lambda checked, idx=p['id']: self.cmd_read_single(idx))
            
            btn_write = QPushButton("W")
            btn_write.setMaximumWidth(30)
            if "RO" in p['perm']: btn_write.setEnabled(False)
            btn_write.clicked.connect(lambda checked, w=val_edit: w.confirm_write())
            
            action_layout.addWidget(btn_read)
            action_layout.addWidget(btn_write)
            self.table.setCellWidget(row, 5, action_widget)

    def on_user_editing(self):
        if self.cb_auto_refresh.isChecked() and self.auto_timer.isActive():
            self.auto_timer.stop()

    def on_edit_aborted(self):
        if self.cb_auto_refresh.isChecked() and not self.auto_timer.isActive():
            self.auto_timer.start(int(self.spin_interval.value() * 1000))

    def request_write(self, widget):
        val_str = widget.get_write_value()
        if val_str is None or val_str == "":
            return
            
        self.cmd_write_single(widget.param_id, val_str)
        if self.cb_auto_refresh.isChecked() and not self.auto_timer.isActive():
            self.auto_timer.start(int(self.spin_interval.value() * 1000))

    def cmd_read_all(self, keep_bus=False):
        if (not self.hermes.running or not self.current_params or
                self.pending_read_batches):
            return
        if not keep_bus:
            self._set_bus_occupied(True)

        parameter_ids = [parameter['id'] for parameter in self.current_params]
        self.pending_read_batches = [
            parameter_ids[index:index + 40]
            for index in range(0, len(parameter_ids), 40)
        ]
        self._send_next_read_batch()

    def _send_next_read_batch(self):
        """Send one bounded batch so its worst-case response remains below the MTU."""
        if not self.pending_read_batches:
            self._set_bus_occupied(False)
            return
        parameter_ids = self.pending_read_batches.pop(0)
        payload = bytes((len(parameter_ids), *parameter_ids))
        self.hermes.send_frame(0x01, self.base_cmd, payload, priority=0)

    def cmd_read_single(self, param_id):
        if not self.hermes.running: return
        self.pending_read_batches = []
        self._set_bus_occupied(True)
        
        payload = bytes([1, param_id])
        self.hermes.send_frame(0x01, self.base_cmd, payload, priority=0)

    def cmd_write_single(self, param_id, val_str):
        if not self.hermes.running: return
        param_info = next((p for p in self.current_params if p['id'] == param_id), None)
        if not param_info: return
        
        fmt, _ = TYPE_MAP[param_info['type']]
        try:
            if 'F32' in param_info['type']:
                val = float(val_str)
            else:
                val = int(val_str, 16) if val_str.lower().startswith('0x') else int(val_str)
                
            self._set_bus_occupied(True) 
            
            payload = bytearray([1, param_id])
            payload.extend(struct.pack(fmt, val))
            self.hermes.send_frame(0x01, self.base_cmd + 1, bytes(payload), priority=0)
        except ValueError:
            self.log(f"Write failed because the value is invalid: {val_str}", "red")
        except struct.error:
            self.log(f"Write failed because the value is out of range: {val_str}", "red")

    def process_bus_event(self, ev: dict):
        if ev.get('type') != 'DL' or ev.get('dir') != 'RX' or not ev.get('dl_crc_ok'):
            return

        cmd = ev['dl_cmd']
        payload = ev['dl_payload']
        
        if cmd == self.base_cmd:
            if len(payload) < 1: return
            valid_cnt, idx = payload[0], 1
            
            for _ in range(valid_cnt):
                if idx >= len(payload): break
                param_id = payload[idx]
                idx += 1
                
                param_info = next((p for p in self.current_params if p['id'] == param_id), None)
                if not param_info: continue
                
                fmt, size = TYPE_MAP[param_info['type']]
                if idx + size > len(payload): break
                val = struct.unpack_from(fmt, payload, idx)[0]
                idx += size
                
                is_float = 'F32' in param_info['type']
                last_val = self.last_values.get(param_id, None)
                changed = False
                
                if last_val is not None:
                    if is_float: changed = abs(val - last_val) > 1e-5
                    else: changed = (val != last_val)
                        
                self.last_values[param_id] = val
                
                for row in range(self.table.rowCount()):
                    w = self.table.cellWidget(row, 4)
                    
                    if hasattr(w, 'param_id') and w.param_id == param_id:
                        is_hex = param_info.get('display_hex', False)
                        updated = w.update_display_value(val, is_float, is_hex)
                        
                        # Each editor owns its transient value-change presentation.
                        if updated:
                            w.flash_status(changed)
                        break
            if self.pending_read_batches:
                self._send_next_read_batch()
            else:
                self._set_bus_occupied(False)
        
        elif cmd == self.base_cmd + 1:
            if len(payload) == 1:
                status = payload[0]
                if status == 0:
                    self.log("Parameter write completed.", "green")
                    self.cmd_read_all(keep_bus=True)
                else:
                    self._set_bus_occupied(False) 
                    self.log("The target rejected the parameter write.", "red")
            else:
                self._set_bus_occupied(False)

class TabTunableManager(QWidget):
    sig_global_bus_busy = pyqtSignal(bool)

    def __init__(self, hermes: HermesDatalinkQt, discovery: ResourceDiscovery):
        super().__init__()
        self.hermes = hermes
        self.discovery = discovery
        self._setup_ui()
        self.hermes.sig_bus_event.connect(self.dispatch_bus_event)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        toolbar = QHBoxLayout()
        self.btn_add_tab = QPushButton("Add Tunable Target")
        self.btn_add_tab.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 5px;")
        self.btn_add_tab.clicked.connect(self.add_new_instance)
        
        toolbar.addWidget(self.btn_add_tab)
        toolbar.addStretch()
        layout.addLayout(toolbar)
        
        self.tab_widget = QTabWidget()
        self.tab_widget.setTabsClosable(True)
        self.tab_widget.tabCloseRequested.connect(self.remove_instance)
        layout.addWidget(self.tab_widget)
        
        self.add_new_instance(default_name="Axis A / Node 1", default_cmd=0x30)

    def add_new_instance(self, checked=False, default_name=None, default_cmd=0x30):
        name = default_name
        if not name:
            name, ok = QInputDialog.getText(self, "Add Tunable Target", "Target name, for example Axis B:")
            if not ok or not name.strip(): return
                
        suggested_cmd = default_cmd + (self.tab_widget.count() * 0x10)
        new_instance = TunableInstanceWidget(self.hermes, self.discovery, name, suggested_cmd)

        new_instance.sig_bus_busy.connect(self.sig_global_bus_busy.emit)

        self.tab_widget.addTab(new_instance, f"⚙️ {name}")
        self.tab_widget.setCurrentWidget(new_instance)

    def remove_instance(self, index):
        widget = self.tab_widget.widget(index)
        if isinstance(widget, TunableInstanceWidget):
            widget.stop_timers()
        self.tab_widget.removeTab(index)

    def dispatch_bus_event(self, ev: dict):
        for i in range(self.tab_widget.count()):
            widget = self.tab_widget.widget(i)
            if isinstance(widget, TunableInstanceWidget):
                widget.process_bus_event(ev)
