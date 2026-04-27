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

# 变量类型对应的 struct 解析格式与字节大小
TYPE_MAP = {
    'GMP_PARAM_TYPE_U16': ('<H', 2),
    'GMP_PARAM_TYPE_I16': ('<h', 2),
    'GMP_PARAM_TYPE_U32': ('<I', 4),
    'GMP_PARAM_TYPE_I32': ('<i', 4),
    'GMP_PARAM_TYPE_F32': ('<f', 4)
}

# =========================================================
# 内部辅助：C 语言注释清除器
# =========================================================
def strip_c_comments(text: str) -> str:
    """彻底清除 C 语言代码中的块注释 /* */ 和行注释 //"""
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    text = re.sub(r'//.*', '', text)
    return text

# =========================================================
# 对话框：C 语言 Tunable 字典解析器
# =========================================================
class CCodeParserDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("从 C 代码解析 Tunable 字典")
        self.resize(600, 400)
        self.parsed_data = []
        
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("请粘贴下位机的 const gmp_param_item_t 数组定义："))
        
        self.txt_code = QTextEdit()
        self.txt_code.setPlaceholderText("""例如：
const gmp_param_item_t dict_m1[] = {
    { &m1.kp, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW },
    { &m1.speed, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO },
};""")
        layout.addWidget(self.txt_code)
        
        btn_layout = QHBoxLayout()
        self.btn_parse = QPushButton("🔍 解析并生成字典")
        self.btn_parse.clicked.connect(self.parse_code)
        btn_layout.addStretch()
        btn_layout.addWidget(self.btn_parse)
        layout.addLayout(btn_layout)

    def parse_code(self):
        code = self.txt_code.toPlainText()
        code = strip_c_comments(code)
        
        pattern = r'\{\s*&\s*([^,]+?)\s*,\s*(GMP_PARAM_TYPE_[A-Z0-9_]+)\s*,\s*(GMP_PARAM_PERM_[A-Z]+)\s*\}'
        matches = re.findall(pattern, code)
        
        if not matches:
            QMessageBox.warning(self, "解析失败", "未检测到符合规则的字典项，请检查语法格式。")
            return
            
        self.parsed_data = []
        for i, match in enumerate(matches):
            var_name, var_type, var_perm = match
            if var_type not in TYPE_MAP:
                QMessageBox.warning(self, "类型错误", f"不支持的变量类型: {var_type}")
                return
                
            self.parsed_data.append({
                "id": i,
                "name": var_name.strip(),
                "type": var_type,
                "perm": var_perm,
                "display_hex": False, 
                "enum_map": None      
            })
            
        QMessageBox.information(self, "解析成功", f"成功解析出 {len(self.parsed_data)} 个变量！")
        self.accept()

# =========================================================
# 对话框：C 语言 Enum 枚举解析器
# =========================================================
class CEnumParserDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("绑定 C 语言枚举定义")
        self.resize(500, 400)
        self.parsed_map = {}
        
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("请粘贴 C 语言的 enum 定义块 (支持 CiA402 等自增格式)："))
        
        self.txt_code = QTextEdit()
        self.txt_code.setPlaceholderText("""typedef enum {
    CIA402_CMD_NULL = 0,
    CIA402_CMD_DISABLE_VOLTAGE = 1,
    CIA402_CMD_SHUTDOWN,
    CIA402_CMD_SWITCHON
} cia402_cmd_t;""")
        layout.addWidget(self.txt_code)
        
        btn_layout = QHBoxLayout()
        self.btn_parse = QPushButton("🔍 解析并应用映射")
        self.btn_parse.clicked.connect(self.parse_code)
        btn_layout.addStretch()
        btn_layout.addWidget(self.btn_parse)
        layout.addLayout(btn_layout)

    def parse_code(self):
        code = self.txt_code.toPlainText()
        code = strip_c_comments(code)
        
        match = re.search(r'\{([^}]+)\}', code)
        if not match:
            QMessageBox.warning(self, "解析失败", "未找到大括号 {} 包含的枚举内容！")
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
            QMessageBox.warning(self, "解析为空", "未能解析出任何有效的枚举项！")
            return
            
        self.accept()

# =========================================================
# 交互增强型控件库 (组件化状态机与防频闪引擎)
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
        
        # 初始始终锁定为只读，阻挡单击误编辑
        self.setReadOnly(True) 
        
        # 【性能引擎】独立托管本组件的样式恢复定时器
        self.fade_timer = QTimer(self)
        self.fade_timer.setSingleShot(True)
        self.fade_timer.timeout.connect(self._restore_style)

        if self.is_permanently_ro:
            self.setStyleSheet("background-color: #F5F5F5; color: #757575;")
        else:
            self.textEdited.connect(self._on_text_edited)
            self.returnPressed.connect(self._on_return_pressed)

    def mouseDoubleClickEvent(self, event):
        """【体验优化】双击才允许编辑，单机只做焦点选取"""
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
            self.setReadOnly(True) # 失去焦点立即恢复护盾
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
        """【防频闪机制】刷新时间重叠时，直接延期定时器，杜绝闪烁"""
        if self.hasFocus() and not self.isReadOnly(): return
        
        if changed:
            self.setStyleSheet("background-color: #FFF59D; color: #D32F2F; font-weight: bold;") # 突变：黄底红字
        else:
            self.setStyleSheet("background-color: #C8E6C9; color: black;") # 持续刷新：静谧绿底
            
        self.fade_timer.start(1000) # 保持 1s

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
        """【体验优化】拦截鼠标单击，让事件穿透给表格，实现选中行而防误下拉"""
        event.ignore()

    def mouseDoubleClickEvent(self, event):
        """双击才强制展开下拉列表"""
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
# 单个可调对象实例面板
# =========================================================
class TunableInstanceWidget(QWidget):
    sig_bus_busy = pyqtSignal(bool)

    def __init__(self, hermes: HermesDatalinkQt, instance_name: str, base_cmd: int = 0x30):
        super().__init__()
        self.hermes = hermes
        self.instance_name = instance_name
        self.base_cmd = base_cmd
        
        self.current_params = [] 
        self.last_values = {} 
        
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
        self.log("⚠️ 总线锁定超时，已强制释放 (可能发生了丢包)", "orange")

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)

        ctrl_group = QGroupBox(f"【{self.instance_name}】对象属性与配置")
        ctrl_layout = QHBoxLayout()
        
        ctrl_layout.addWidget(QLabel("Base CMD (Hex):"))
        self.edit_cmd = QLineEdit(hex(self.base_cmd))
        self.edit_cmd.setMaximumWidth(60)
        self.edit_cmd.textChanged.connect(self._update_base_cmd)
        ctrl_layout.addWidget(self.edit_cmd)
        
        self.btn_import_c = QPushButton("📝 从 C 代码提取")
        self.btn_import_c.clicked.connect(self.action_import_c)
        ctrl_layout.addWidget(self.btn_import_c)
        
        self.btn_load_json = QPushButton("📂 载入 JSON")
        self.btn_load_json.clicked.connect(self.action_load_json)
        ctrl_layout.addWidget(self.btn_load_json)
        
        self.btn_save_json = QPushButton("💾 导出 JSON")
        self.btn_save_json.clicked.connect(self.action_save_json)
        ctrl_layout.addWidget(self.btn_save_json)
        
        ctrl_layout.addStretch()
        
        self.cb_auto_refresh = QCheckBox("启用全局定时刷新")
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
        self.table.setHorizontalHeaderLabels(["ID", "变量名称", "数据类型", "读写权限", "当前数值", "操作"])
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)
        
        self.table.setContextMenuPolicy(Qt.CustomContextMenu)
        self.table.customContextMenuRequested.connect(self.show_context_menu)
        
        main_layout.addWidget(self.table)

        btn_layout = QHBoxLayout()
        self.btn_read_all = QPushButton("🔄 全局读取当前对象 (Read All)")
        self.btn_read_all.setMinimumHeight(40)
        self.btn_read_all.setStyleSheet("background-color: #E3F2FD; font-weight: bold;")
        self.btn_read_all.clicked.connect(lambda: self.cmd_read_all(False))
        
        btn_layout.addWidget(self.btn_read_all)
        main_layout.addLayout(btn_layout)

    def log(self, msg, color="black"):
        if hasattr(self.hermes, 'sig_log_msg'):
            self.hermes.sig_log_msg.emit(f"<span style='color:{color};'>[{self.instance_name}] {msg}</span>")
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
            hex_action = menu.addAction("✅ 16 进制显示 (Hex Mode)" if is_hex else "🔲 16 进制显示 (Hex Mode)")
            hex_action.triggered.connect(lambda: self.toggle_hex_display(param_id))
            menu.addSeparator()

        enum_action = menu.addAction("📜 绑定 C 语言枚举表 (Bind Enum)")
        enum_action.triggered.connect(lambda: self.bind_enum(param_id))
        
        if param_info.get('enum_map'):
            clear_enum_action = menu.addAction("❌ 解除枚举绑定 (Remove Enum)")
            clear_enum_action.triggered.connect(lambda: self.clear_enum(param_id))

        menu.exec_(self.table.viewport().mapToGlobal(pos))

    # =========================================================
    # 安全锁机制：在重建 UI 的操作前暂停轮询
    # =========================================================
    def toggle_hex_display(self, param_id):
        param_info = next((p for p in self.current_params if p['id'] == param_id), None)
        if param_info:
            was_running = self.auto_timer.isActive()
            if was_running: self.auto_timer.stop()
            
            param_info['display_hex'] = not param_info.get('display_hex', False)
            self._render_table()
            self.log(f"变量 [{param_info['name']}] 显示模式已切换", "blue")
            
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
            self.log(f"变量 [{param_info['name']}] 已成功绑定 {len(dialog.parsed_map)} 项枚举映射", "green")
            
        if was_running and self.cb_auto_refresh.isChecked():
            self.auto_timer.start(int(self.spin_interval.value() * 1000))

    def clear_enum(self, param_id):
        param_info = next((p for p in self.current_params if p['id'] == param_id), None)
        if param_info:
            was_running = self.auto_timer.isActive()
            if was_running: self.auto_timer.stop()
            
            param_info['enum_map'] = None
            self._render_table()
            self.log(f"变量 [{param_info['name']}] 已移除枚举映射", "gray")
            
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
        
        path, _ = QFileDialog.getOpenFileName(self, "载入 Tunable 配置", "", "JSON Files (*.json)")
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
                self.log(f"已加载配置: {os.path.basename(path)}", "green")
            except Exception as e:
                self.log(f"加载 JSON 失败: {e}", "red")
                
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
            self.log(f"已启动定时刷新 ({self.spin_interval.value()}s)", "blue")
        else:
            self.auto_timer.stop()
            self.log("已停止定时刷新", "gray")

    def stop_timers(self):
        if self.auto_timer.isActive():
            self.auto_timer.stop()

    def action_save_json(self):
        if not self.current_params:
            QMessageBox.warning(self, "为空", "没有可导出的参数字典！")
            return
        default_name = f"tunable_{self.instance_name.replace(' ', '_').replace('/', '_')}.json"
        path, _ = QFileDialog.getSaveFileName(self, "导出 Tunable 配置", default_name, "JSON Files (*.json)")
        if path:
            data = {
                "base_cmd": self.base_cmd,
                "auto_refresh": self.cb_auto_refresh.isChecked(),
                "refresh_interval": self.spin_interval.value(),
                "params": self.current_params
            }
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4)
            self.log(f"配置已导出至: {os.path.basename(path)}", "green")

    def _render_table(self):
        self.table.clearContents()
        self.table.setRowCount(len(self.current_params))
        
        for row, p in enumerate(self.current_params):
            item_id = QTableWidgetItem(str(p['id']))
            item_id.setTextAlignment(Qt.AlignCenter)
            item_id.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.table.setItem(row, 0, item_id)
            
            item_name = QTableWidgetItem(p['name'])
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
        if not self.hermes.running or not self.current_params: return
        if not keep_bus:
            self._set_bus_occupied(True) 
            
        payload = bytearray([len(self.current_params)])
        for p in self.current_params: payload.append(p['id'])
        self.hermes.send_frame(0x01, self.base_cmd, bytes(payload), priority=0)

    def cmd_read_single(self, param_id):
        if not self.hermes.running: return
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
            self.log(f"写入失败：格式错误 ({val_str})", "red")
        except struct.error:
            self.log(f"写入失败：数值越界 ({val_str})", "red")

    def process_bus_event(self, ev: dict):
        if ev.get('type') != 'DL' or ev.get('dir') != 'RX' or not ev.get('dl_crc_ok'):
            return

        cmd = ev['dl_cmd']
        payload = ev['dl_payload']
        
        if cmd == self.base_cmd:
            self._set_bus_occupied(False) 

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
                        
                        # 【核心解耦】：视觉渲染彻底下放给组件自己管理，拒绝 Lambda 闭包灾难
                        if updated:
                            w.flash_status(changed)
                        break
        
        elif cmd == self.base_cmd + 1:
            if len(payload) >= 1:
                status = payload[0]
                if status == 0:
                    self.log(f"✅ 参数写入成功", "green")
                    self.cmd_read_all(keep_bus=True)
                else:
                    self._set_bus_occupied(False) 
                    self.log(f"❌ 参数被下位机拦截拒绝", "red")
            else:
                self._set_bus_occupied(False)

class TabTunableManager(QWidget):
    sig_global_bus_busy = pyqtSignal(bool)

    def __init__(self, hermes: HermesDatalinkQt):
        super().__init__()
        self.hermes = hermes
        self._setup_ui()
        self.hermes.sig_bus_event.connect(self.dispatch_bus_event)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        toolbar = QHBoxLayout()
        self.btn_add_tab = QPushButton("➕ 添加可调对象 (New Tunable Target)")
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
            name, ok = QInputDialog.getText(self, "添加对象", "请输入调参对象名称 (如: Axis B):")
            if not ok or not name.strip(): return
                
        suggested_cmd = default_cmd + (self.tab_widget.count() * 0x10)
        new_instance = TunableInstanceWidget(self.hermes, name, suggested_cmd)

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