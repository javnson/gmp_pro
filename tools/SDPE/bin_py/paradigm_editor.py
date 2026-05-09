# -*- coding: utf-8 -*-
"""
SDPE Paradigm Editor Component
Allows editing of parameter definitions and code generation rules.
"""

from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, 
                             QLineEdit, QTableWidget, QTableWidgetItem, 
                             QLabel, QTextEdit, QPushButton, QGroupBox)
from PySide6.QtCore import Qt

class ParadigmEditor(QWidget):
    def __init__(self, file_path, data):
        super().__init__()
        self.file_path = file_path
        self.data = data
        self.init_ui()

    def init_ui(self):
        layout = QHBoxLayout(self)

        # Left Side: Metadata and Parameter Definitions
        left_panel = QVBoxLayout()
        
        # 1. Metadata Group
        meta_group = QGroupBox("General Metadata")
        meta_layout = QFormLayout()
        self.name_edit = QLineEdit(self.data.get("paradigm_name", ""))
        self.path_edit = QLineEdit(self.data.get("output_path", ""))
        meta_layout.addRow("Paradigm Name:", self.name_edit)
        meta_layout.addRow("Output Category:", self.path_edit)
        meta_group.setLayout(meta_layout)
        left_panel.addWidget(meta_group)

        # 2. Parameters Schema Group
        param_group = QGroupBox("Parameter Definitions (Schema)")
        param_layout = QVBoxLayout()
        self.param_table = QTableWidget(0, 3)
        self.param_table.setHorizontalHeaderLabels(["Key", "Unit", "Default"])
        
        # Load existing params
        params = self.data.get("parameters", {})
        self.param_table.setRowCount(len(params))
        for i, (k, v) in enumerate(params.items()):
            self.param_table.setItem(i, 0, QTableWidgetItem(k))
            self.param_table.setItem(i, 1, QTableWidgetItem(v.get("unit", "")))
            self.param_table.setItem(i, 2, QTableWidgetItem(str(v.get("default", ""))))
            
        param_layout.addWidget(self.param_table)
        add_btn = QPushButton("+ Add Parameter")
        param_layout.addWidget(add_btn)
        param_group.setLayout(param_layout)
        left_panel.addWidget(param_group)
        
        layout.addLayout(left_panel, 1)

        # Right Side: Code Generation Logic (Brief View)
        right_panel = QVBoxLayout()
        code_group = QGroupBox("Code Generation Blocks (Jinja2 Hooks)")
        code_layout = QVBoxLayout()
        
        self.init_code = QTextEdit()
        self.init_code.setPlaceholderText("Initialization Block...")
        self.init_code.setPlainText(self.data.get("template_blocks", {}).get("init", ""))
        
        code_layout.addWidget(QLabel("Init Block:"))
        code_layout.addWidget(self.init_code)
        
        # Save Button
        save_btn = QPushButton("Save Paradigm")
        save_btn.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold; height: 30px;")
        
        right_panel.addWidget(code_group)
        right_panel.addLayout(code_layout)
        right_panel.addWidget(save_btn)
        
        layout.addLayout(right_panel, 1)