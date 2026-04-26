import os
import json
import re
import struct
import time
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                             QLineEdit, QPushButton, QLabel, QFormLayout, 
                             QTableWidget, QTableWidgetItem, QHeaderView, 
                             QFileDialog, QDialog, QTextEdit, QMessageBox, 
                             QCheckBox, QDoubleSpinBox, QTabWidget, QInputDialog)
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
        pattern = r'\{\s*&([a-zA-Z0-9_>.-]+)\s*,\s*(GMP_PARAM_TYPE_[A-Z0-9_]+)\s*,\s*(GMP_PARAM_PERM_[A-Z]+)\s*\}'
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
                "name": var_name,
                "type": var_type,
                "perm": var_perm
            })
            
        QMessageBox.information(self, "解析成功", f"成功解析出 {len(self.parsed_data)} 个变量！")
        self.accept()

# =========================================================
# 交互增强型文本框 (严格状态机管理)
# =========================================================
class ParamLineEdit(QLineEdit):
    sig_user_editing = pyqtSignal()      # 触发暂停刷新
    sig_write_requested = pyqtSignal()   # 触发下发指令
    sig_edit_aborted = pyqtSignal()      # 触发恢复刷新

    def __init__(self, param_id, is_ro, parent=None):
        super().__init__(parent)
        self.param_id = param_id
        self.is_ro = is_ro
        self.is_dirty = False 

        if self.is_ro:
            self.setReadOnly(True)
            self.setStyleSheet("background-color: #F5F5F5; color: #757575;")
        else:
            self.textEdited.connect(self._on_text_edited)
            self.returnPressed.connect(self._on_return_pressed)

    def focusInEvent(self, event):
        super().focusInEvent(event)
        if not self.is_ro:
            self.setStyleSheet("background-color: #E3F2FD; color: #0D47A1; font-weight: bold;")
            self.sig_user_editing.emit()

    def focusOutEvent(self, event):
        super().focusOutEvent(event)
        if self.is_dirty:
            self.is_dirty = False
            self.sig_edit_aborted.emit() 
        if not self.is_ro:
            self.setStyleSheet("")

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
            self.busy_timer.start(400) # 适当延长超时保护阈值
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

    def action_import_c(self):
        dialog = CCodeParserDialog(self)
        if dialog.exec_() == QDialog.Accepted and dialog.parsed_data:
            self.current_params = dialog.parsed_data
            self.last_values.clear()
            self._render_table()

    def action_load_json(self):
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
                    self.last_values.clear()
                    self._render_table()
                self.log(f"已加载配置: {os.path.basename(path)}", "green")
            except Exception as e:
                self.log(f"加载 JSON 失败: {e}", "red")

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

    # =========================================================
    # 状态互锁管理
    # =========================================================
    def on_user_editing(self):
        if self.cb_auto_refresh.isChecked() and self.auto_timer.isActive():
            self.auto_timer.stop()
            self.log("✏️ 变量编辑中，暂停全局刷新...", "orange")

    def on_edit_aborted(self):
        if self.cb_auto_refresh.isChecked() and not self.auto_timer.isActive():
            self.auto_timer.start(int(self.spin_interval.value() * 1000))
            self.log("▶️ 编辑放弃，恢复定时刷新", "gray")

    def request_write(self, widget: ParamLineEdit):
        self.cmd_write_single(widget.param_id, widget.text())
        if self.cb_auto_refresh.isChecked() and not self.auto_timer.isActive():
            self.auto_timer.start(int(self.spin_interval.value() * 1000))
            self.log("▶️ 写入请求已下发，恢复定时刷新", "blue")

    # =========================================================
    # 核心指令封装 (带线程微秒级避让)
    # =========================================================
    def cmd_read_all(self, keep_bus=False):
        if not self.hermes.running or not self.current_params: return
        if not keep_bus:
            self._set_bus_occupied(True)
            time.sleep(0.005) # 避让后台 PIL 线程 5ms，杜绝串口冲突
            
        payload = bytearray([len(self.current_params)])
        for p in self.current_params: payload.append(p['id'])
        self.hermes.send_frame(0x01, self.base_cmd, bytes(payload))

    def cmd_read_single(self, param_id):
        if not self.hermes.running: return
        self._set_bus_occupied(True)
        time.sleep(0.005) # 避让后台 PIL 线程
        
        payload = bytes([1, param_id])
        self.hermes.send_frame(0x01, self.base_cmd, payload)

    def cmd_write_single(self, param_id, val_str):
        if not self.hermes.running: return
        param_info = next((p for p in self.current_params if p['id'] == param_id), None)
        if not param_info: return
        
        fmt, _ = TYPE_MAP[param_info['type']]
        try:
            val = float(val_str) if 'F32' in param_info['type'] else int(val_str)
            self._set_bus_occupied(True) 
            time.sleep(0.005) # 避让后台 PIL 线程
            
            payload = bytearray([1, param_id])
            payload.extend(struct.pack(fmt, val))
            self.hermes.send_frame(0x01, self.base_cmd + 1, bytes(payload))
        except ValueError:
            self.log(f"写入失败：格式错误 ({val_str})", "red")
        except struct.error:
            self.log(f"写入失败：数值越界 ({val_str})", "red")

    # =========================================================
    # 接收更新与视觉反馈比对引擎
    # =========================================================
    def process_bus_event(self, ev: dict):
        cmd = ev['dl_cmd']
        payload = ev['dl_payload']
        
        # 读回响 (READ ACK)
        if cmd == self.base_cmd:
            self._set_bus_occupied(False) # 所有数据读取完毕，安全释放总线

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
                    if isinstance(w, ParamLineEdit) and w.param_id == param_id:
                        if w.hasFocus() or w.is_dirty: break 
                        
                        w.setText(f"{val:.4f}" if is_float else str(val))
                        
                        if changed:
                            w.setStyleSheet("background-color: #FFF59D; color: black; font-weight: bold;")
                        else:
                            w.setStyleSheet("background-color: #C8E6C9; color: black;")
                            
                        QTimer.singleShot(800, lambda widget=w, ro="RO" in param_info['perm']: 
                                          widget.setStyleSheet("background-color: #F5F5F5; color: #757575;" if ro else ""))
                        break
        
        # 写回响 (WRITE ACK)
        elif cmd == self.base_cmd + 1:
            # 【核心修改】：不释放总线，携带原子锁直接进行状态拉取
            if len(payload) >= 1:
                status = payload[0]
                if status == 0:
                    self.log(f"✅ 参数写入成功", "green")
                    self.cmd_read_all(keep_bus=True) # 保持锁定，直接回读
                else:
                    self._set_bus_occupied(False) # 仅在失败时释放总线
                    self.log(f"❌ 参数被下位机拦截拒绝", "red")
            else:
                self._set_bus_occupied(False)

# =========================================================
# 容器管理面板
# =========================================================
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