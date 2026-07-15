"""Reusable dialogs for the SDPE PyQt GUI."""

from __future__ import annotations

try:
    from PyQt6.QtCore import Qt
    from PyQt6.QtWidgets import (
        QDialog,
        QDialogButtonBox,
        QInputDialog,
        QHeaderView,
        QLineEdit,
        QListWidget,
        QMessageBox,
        QTextEdit,
        QTreeWidget,
        QTreeWidgetItem,
        QVBoxLayout,
        QWidget,
    )
except ImportError:  # pragma: no cover - depends on local desktop environment.
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import (
        QDialog,
        QDialogButtonBox,
        QInputDialog,
        QHeaderView,
        QLineEdit,
        QListWidget,
        QMessageBox,
        QTextEdit,
        QTreeWidget,
        QTreeWidgetItem,
        QVBoxLayout,
        QWidget,
    )


class ConfirmTextEdit(QTextEdit):
    """Text editor where Enter accepts and Shift+Enter inserts a newline."""

    def __init__(self, dialog: QDialog):
        super().__init__()
        self.dialog = dialog

    def keyPressEvent(self, event) -> None:  # noqa: N802 - Qt override
        if event.key() in {Qt.Key.Key_Return, Qt.Key.Key_Enter}:
            if event.modifiers() & Qt.KeyboardModifier.ShiftModifier:
                super().keyPressEvent(event)
            else:
                self.dialog.accept()
            return
        super().keyPressEvent(event)


def edit_multiline(parent: QWidget, title: str, text: str) -> str | None:
    dialog = QDialog(parent)
    dialog.setWindowTitle(title)
    dialog.resize(620, 360)
    edit = ConfirmTextEdit(dialog)
    edit.setPlainText(text)
    buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)
    layout = QVBoxLayout(dialog)
    layout.addWidget(edit)
    layout.addWidget(buttons)
    if dialog.exec() == QDialog.DialogCode.Accepted:
        return edit.toPlainText()
    return None


def choose_tree_item(parent: QWidget, title: str, paths: list[str]) -> str | None:
    dialog = QDialog(parent)
    dialog.setWindowTitle(title)
    dialog.resize(680, 520)
    search = QLineEdit()
    search.setPlaceholderText("Search keywords")
    tree = QTreeWidget()
    tree.setHeaderLabels(["Symbol"])
    tree.header().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)

    def ensure_child(parent_item: QTreeWidgetItem | QTreeWidget, label: str) -> QTreeWidgetItem:
        for index in range(parent_item.childCount() if isinstance(parent_item, QTreeWidgetItem) else parent_item.topLevelItemCount()):
            child = parent_item.child(index) if isinstance(parent_item, QTreeWidgetItem) else parent_item.topLevelItem(index)
            if child.text(0) == label:
                return child
        child = QTreeWidgetItem([label])
        if isinstance(parent_item, QTreeWidgetItem):
            parent_item.addChild(child)
        else:
            parent_item.addTopLevelItem(child)
        return child

    def populate() -> None:
        query = search.text().strip().lower()
        parts = query.split()
        tree.clear()
        for path in paths:
            if parts and not all(part in path.lower() for part in parts):
                continue
            parent_item: QTreeWidgetItem | QTreeWidget = tree
            tokens = path.split(".")
            for index, token in enumerate(tokens):
                parent_item = ensure_child(parent_item, token)
                if index == len(tokens) - 1:
                    parent_item.setData(0, Qt.ItemDataRole.UserRole, path)
                    parent_item.setToolTip(0, path)
        tree.expandAll()
        tree.setColumnWidth(0, max(tree.columnWidth(0), 420))

    search.textChanged.connect(populate)
    populate()
    buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)
    tree.itemDoubleClicked.connect(lambda item, _col: dialog.accept() if item.data(0, Qt.ItemDataRole.UserRole) else None)
    layout = QVBoxLayout(dialog)
    layout.addWidget(search)
    layout.addWidget(tree)
    layout.addWidget(buttons)
    if dialog.exec() == QDialog.DialogCode.Accepted and tree.currentItem():
        item = tree.currentItem()
        value = item.data(0, Qt.ItemDataRole.UserRole)
        return value if value else None
    return None


def choose_item(parent: QWidget, title: str, items: list[str]) -> str | None:
    dialog = QDialog(parent)
    dialog.setWindowTitle(title)
    dialog.resize(620, 460)
    search = QLineEdit()
    search.setPlaceholderText("Search keywords")
    list_widget = QListWidget()

    def populate() -> None:
        query = search.text().strip().lower()
        parts = query.split()
        list_widget.clear()
        for item in items:
            text = item.lower()
            if not parts or all(part in text for part in parts):
                list_widget.addItem(item)
        if list_widget.count() > 0:
            list_widget.setCurrentRow(0)

    search.textChanged.connect(populate)
    populate()
    buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)
    list_widget.itemDoubleClicked.connect(lambda _item: dialog.accept())
    layout = QVBoxLayout(dialog)
    layout.addWidget(search)
    layout.addWidget(list_widget)
    layout.addWidget(buttons)
    if dialog.exec() == QDialog.DialogCode.Accepted and list_widget.currentItem():
        return list_widget.currentItem().text()
    return None


def prompt_identifier(parent: QWidget, title: str, label: str) -> str:
    text, ok = QInputDialog.getText(parent, title, label)
    if not ok:
        return ""
    return text.strip()


def confirm_delete(parent: QWidget, title: str, text: str) -> bool:
    result = QMessageBox.question(
        parent,
        title,
        text,
        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        QMessageBox.StandardButton.No,
    )
    return result == QMessageBox.StandardButton.Yes
