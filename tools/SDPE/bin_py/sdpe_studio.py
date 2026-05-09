# -*- coding: utf-8 -*-
"""
SDPE Resource Architect - Robust Version
"""

import os
import sys
import json
from PySide6.QtWidgets import (QApplication, QMainWindow, QTreeView, QTabWidget, 
                             QDockWidget, QFileSystemModel, QVBoxLayout, QHBoxLayout,
                             QWidget, QTableWidget, QTableWidgetItem, QHeaderView, 
                             QSplitter, QTextEdit, QPushButton, QMessageBox, QLabel,
                             QFileDialog)
from PySide6.QtCore import Qt, QDir
from PySide6.QtGui import QFont

class ObjectEditor(QWidget):
    """View for editing JSON as an Object (Property Grid)."""
    def __init__(self, data, save_callback):
        super().__init__()
        self.data = data
        self.save_callback = save_callback
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Parameter", "Value", "Unit"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.refresh_table()
        
        layout.addWidget(QLabel("Object Property Grid:"))
        layout.addWidget(self.table)
        
        save_btn = QPushButton("Apply Property Changes")
        save_btn.clicked.connect(self.collect_and_save)
        layout.addWidget(save_btn)

    def refresh_table(self):
        params = self.data.get("parameters", {})
        self.table.setRowCount(len(params))
        for i, (k, v) in enumerate(params.items()):
            self.table.setItem(i, 0, QTableWidgetItem(k))
            val = v.get("default", v) if isinstance(v, dict) else v
            unit = v.get("unit", "-") if isinstance(v, dict) else "-"
            self.table.setItem(i, 1, QTableWidgetItem(str(val)))
            self.table.setItem(i, 2, QTableWidgetItem(unit))

    def collect_and_save(self):
        for i in range(self.table.rowCount()):
            key = self.table.item(i, 0).text()
            val = self.table.item(i, 1).text()
            try:
                val = float(val) if '.' in val else int(val)
            except: pass
            
            if isinstance(self.data["parameters"].get(key), dict):
                self.data["parameters"][key]["default"] = val
            else:
                self.data["parameters"][key] = val
        self.save_callback(self.data)

class SDPEResourceArchitect(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SDPE Resource Architect")
        self.resize(1300, 900)
        
        # --- Path Initialization with Validation ---
        self.lib_root = os.getenv("GMP_PRO_LOCATION")
        
        if not self.lib_root or not os.path.exists(self.lib_root):
            QMessageBox.warning(self, "Path Error", "GMP_PRO_LOCATION not found. Please select GMP library root.")
            self.lib_root = QFileDialog.getExistingDirectory(self, "Select GMP Library Root")
            if not self.lib_root:
                sys.exit(0)

        self.sdpe_root = os.path.join(self.lib_root, "tools", "SDPE")
        self.paradigms_path = os.path.join(self.sdpe_root, "paradigms")
        self.templates_path = os.path.join(self.sdpe_root, "templates")

        # Create paths if they don't exist (optional, but prevents crashes)
        for p in [self.paradigms_path, self.templates_path]:
            if not os.path.exists(p):
                os.makedirs(p, exist_ok=True)

        print(f"[*] Paradigms Path: {self.paradigms_path}")
        print(f"[*] Templates Path: {self.templates_path}")

        self.setup_ui()

    def setup_ui(self):
        # 1. Left Explorer (Paradigms)
        dock = QDockWidget("Paradigms Explorer", self)
        self.fs_model = QFileSystemModel()
        self.fs_model.setRootPath(self.paradigms_path)
        
        self.tree = QTreeView()
        self.tree.setModel(self.fs_model)
        self.tree.setRootIndex(self.fs_model.index(self.paradigms_path))
        
        for i in range(1, 4): self.tree.setColumnHidden(i, True)
        self.tree.header().hide()
        self.tree.doubleClicked.connect(self.on_item_opened)
        
        dock.setWidget(self.tree)
        self.addDockWidget(Qt.LeftDockWidgetArea, dock)

        # 2. Central Editor
        self.tabs = QTabWidget()
        self.tabs.setTabsClosable(True)
        self.tabs.tabCloseRequested.connect(lambda i: self.tabs.removeTab(i))
        self.setCentralWidget(self.tabs)

    def on_item_opened(self, index):
        path = self.fs_model.filePath(index)
        if os.path.isdir(path): return # Don't try to open folders
        if not path.lower().endswith(".json"): return
        
        filename = os.path.basename(path)
        self.open_paradigm_editor(path, filename)

    def open_paradigm_editor(self, path, filename):
        # Check if already open
        for i in range(self.tabs.count()):
            if self.tabs.tabToolTip(i) == path:
                self.tabs.setCurrentIndex(i)
                return

        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            QMessageBox.warning(self, "Load Error", f"Failed to parse JSON:\n{str(e)}")
            return

        container = QWidget()
        layout = QVBoxLayout(container)
        sub_tabs = QTabWidget()
        
        # Tab 1: Object Mode
        if "parameters" in data:
            obj_editor = ObjectEditor(data, lambda d: self.save_json(path, d))
            sub_tabs.addTab(obj_editor, "Object View")
        
        # Tab 2: Template Mode (Auto-link)
        template_ref = data.get("template_ref")
        if template_ref:
            t_path = os.path.join(self.templates_path, template_ref)
            sub_tabs.addTab(self.create_text_editor(t_path), f"Template ({template_ref})")
        
        # Tab 3: Raw JSON
        sub_tabs.addTab(self.create_text_editor(path, is_json=True), "Raw JSON")

        layout.addWidget(sub_tabs)
        idx = self.tabs.addTab(container, filename)
        self.tabs.setTabToolTip(idx, path)
        self.tabs.setCurrentIndex(idx)

    def create_text_editor(self, path, is_json=False):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        edit = QTextEdit()
        edit.setFont(QFont("Consolas", 10))
        
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                edit.setPlainText(f.read())
        else:
            edit.setPlainText(f"// File not found: {path}")

        btn = QPushButton(f"Save {os.path.basename(path)}")
        btn.clicked.connect(lambda: self.save_text(path, edit.toPlainText()))
        
        layout.addWidget(edit)
        layout.addWidget(btn)
        return widget

    def save_json(self, path, data):
        try:
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4)
            self.statusBar().showMessage(f"Saved: {path}", 3000)
        except Exception as e:
            QMessageBox.warning(self, "Save Error", str(e))

    def save_text(self, path, content):
        try:
            # Ensure directory exists if saving a new template
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w', encoding='utf-8') as f:
                f.write(content)
            self.statusBar().showMessage(f"Saved: {path}", 3000)
        except Exception as e:
            QMessageBox.warning(self, "Save Error", str(e))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = SDPEResourceArchitect()
    window.show()
    sys.exit(app.exec())