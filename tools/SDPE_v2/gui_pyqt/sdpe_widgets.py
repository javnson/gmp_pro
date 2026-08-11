"""Shared editable table and tree widgets for the SDPE v2 GUI."""

from __future__ import annotations

import json
import re
from typing import Any, Callable
from uuid import uuid4

try:
    from PyQt6.QtCore import QByteArray, QEvent, QMimeData, Qt, pyqtSignal as Signal
    from PyQt6.QtGui import QAction, QColor, QKeySequence, QPen
    from PyQt6.QtWidgets import (
        QAbstractItemView,
        QApplication,
        QCheckBox,
        QComboBox,
        QHBoxLayout,
        QHeaderView,
        QLineEdit,
        QMenu,
        QStyle,
        QStyledItemDelegate,
        QTableWidget,
        QTableWidgetItem,
        QTreeWidget,
        QTreeWidgetItem,
        QWidget,
    )
except ImportError:  # pragma: no cover - depends on local desktop environment.
    from PySide6.QtCore import QByteArray, QEvent, QMimeData, Qt, Signal
    from PySide6.QtGui import QAction, QColor, QKeySequence, QPen
    from PySide6.QtWidgets import (
        QAbstractItemView,
        QApplication,
        QCheckBox,
        QComboBox,
        QHBoxLayout,
        QHeaderView,
        QLineEdit,
        QMenu,
        QStyle,
        QStyledItemDelegate,
        QTableWidget,
        QTableWidgetItem,
        QTreeWidget,
        QTreeWidgetItem,
        QWidget,
    )


VALIDATION_BORDER_ROLE = Qt.ItemDataRole.UserRole.value + 101
READ_ONLY_CELL_ROLE = Qt.ItemDataRole.UserRole.value + 102
_WIDGET_KEY_ROLE = Qt.ItemDataRole.UserRole.value + 102
_SOURCE_ORDER_ROLE = Qt.ItemDataRole.UserRole.value + 23
_SELECTION_KEY_ROLE = Qt.ItemDataRole.UserRole.value + 22
_TABLE_MIME = "application/x-sdpe-table-rows-v1"
_TREE_MIME = "application/x-sdpe-tree-items-v1"
_DATA_ROLE_COUNT = 24


def _json_value(value: Any) -> Any:
    """Return a clipboard-safe representation of a Qt item data value."""

    if value is None or isinstance(value, (bool, int, float, str, list, dict)):
        return value
    return str(value)


def _text_matcher(pattern: str, case_sensitive: bool, regex: bool) -> Callable[[str], bool] | None:
    if not pattern:
        return lambda _text: True
    flags = 0 if case_sensitive else re.IGNORECASE
    if regex:
        try:
            expression = re.compile(pattern, flags)
        except re.error:
            return None
        return lambda text: expression.search(text) is not None
    needle = pattern if case_sensitive else pattern.lower()
    return lambda text: needle in (text if case_sensitive else text.lower())


def _flag_enabled(flags, flag) -> bool:  # noqa: ANN001 - Qt flags vary by binding.
    return bool(flags & flag)


def _set_item_flags(item, state: dict[str, bool]) -> None:  # noqa: ANN001 - table/tree item.
    flags = item.flags()
    for name, flag in (
        ("editable", Qt.ItemFlag.ItemIsEditable),
        ("drag", Qt.ItemFlag.ItemIsDragEnabled),
        ("drop", Qt.ItemFlag.ItemIsDropEnabled),
        ("selectable", Qt.ItemFlag.ItemIsSelectable),
        ("enabled", Qt.ItemFlag.ItemIsEnabled),
    ):
        if state.get(name, True):
            flags |= flag
        else:
            flags &= ~flag
    item.setFlags(flags)


def _item_flag_state(item) -> dict[str, bool]:  # noqa: ANN001 - table/tree item.
    flags = item.flags()
    return {
        "editable": _flag_enabled(flags, Qt.ItemFlag.ItemIsEditable),
        "drag": _flag_enabled(flags, Qt.ItemFlag.ItemIsDragEnabled),
        "drop": _flag_enabled(flags, Qt.ItemFlag.ItemIsDropEnabled),
        "selectable": _flag_enabled(flags, Qt.ItemFlag.ItemIsSelectable),
        "enabled": _flag_enabled(flags, Qt.ItemFlag.ItemIsEnabled),
    }


def _checkbox_from_widget(widget: QWidget | None) -> QCheckBox | None:
    if isinstance(widget, QCheckBox):
        return widget
    checkbox = getattr(widget, "_sdpe_checkbox", None) if widget is not None else None
    return checkbox if isinstance(checkbox, QCheckBox) else None


def _centered_checkbox(checked: bool, callback: Callable[[], None]) -> QWidget:
    box = QCheckBox()
    box.setChecked(checked)
    box.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
    panel = QWidget()
    layout = QHBoxLayout(panel)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
    layout.addWidget(box)
    panel._sdpe_checkbox = box
    box.stateChanged.connect(lambda _state: callback())
    return panel


def _widget_state(widget: QWidget | None) -> dict[str, Any] | None:
    if isinstance(widget, QComboBox):
        return {
            "type": "combo",
            "editable": widget.isEditable(),
            "items": [widget.itemText(index) for index in range(widget.count())],
            "value": widget.currentText(),
        }
    checkbox = _checkbox_from_widget(widget)
    if checkbox is not None:
        return {"type": "check", "checked": checkbox.isChecked()}
    return None


class SDPEComboBox(QComboBox):
    """Combo box that does not steal mouse-wheel scrolling unless focused."""

    def __init__(self):
        super().__init__()
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def wheelEvent(self, event) -> None:  # noqa: N802 - Qt override name.
        if QApplication.focusWidget() is self:
            super().wheelEvent(event)
        else:
            event.ignore()


class ValidationBorderDelegate(QStyledItemDelegate):
    """Draw validation borders and keep in-place editors visually opaque."""

    def paint(self, painter, option, index) -> None:  # noqa: ANN001, N802 - Qt override signature.
        if option.state & QStyle.StateFlag.State_Editing:
            painter.fillRect(option.rect, option.palette.base())
        else:
            super().paint(painter, option, index)
        if not index.data(VALIDATION_BORDER_ROLE):
            return
        painter.save()
        pen = QPen(QColor(220, 40, 40))
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawRect(option.rect.adjusted(1, 1, -2, -2))
        painter.restore()

    def createEditor(self, parent, option, index):  # noqa: ANN001, N802 - Qt override signature.
        editor = super().createEditor(parent, option, index)
        if editor is not None:
            editor.setAutoFillBackground(True)
            editor.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, True)
            editor.setStyleSheet(
                "background-color: palette(base); color: palette(text); "
                "selection-background-color: palette(highlight); "
                "selection-color: palette(highlighted-text);"
            )
        return editor


class SDPEDataViewMixin:
    """Single shared feature layer for every editable SDPE data view."""

    _ACTION_LABELS = {
        "add_group": "Add group",
        "add_item": "Add item",
        "copy": "Copy selected rows",
        "cut": "Cut selected rows",
        "paste": "Paste rows",
        "delete": "Delete selected rows",
    }

    def _init_standard_actions(self) -> None:
        self._sdpe_action_handlers: dict[str, Callable[[], None]] = {}
        self._sdpe_actions: dict[str, QAction] = {}
        shortcuts = {
            "add_group": None,
            "add_item": QKeySequence("Insert"),
            "copy": QKeySequence.StandardKey.Copy,
            "cut": QKeySequence.StandardKey.Cut,
            "paste": QKeySequence.StandardKey.Paste,
            "delete": None,
        }
        for name, label in self._ACTION_LABELS.items():
            action = QAction(label, self)
            if shortcuts[name] is not None:
                action.setShortcut(shortcuts[name])
            action.setShortcutContext(Qt.ShortcutContext.WidgetShortcut)
            action.triggered.connect(lambda _checked=False, n=name: self._run_standard_action(n))
            self.addAction(action)
            self._sdpe_actions[name] = action
        self._sdpe_actions["add_group"].setVisible(False)
        self._sdpe_actions["add_item"].setVisible(False)

    def _run_standard_action(self, name: str) -> None:
        mutating = name in {"add_group", "add_item", "cut", "paste", "delete"}
        if mutating:
            self.clear_display_sort()
            self.mutationStarted.emit()
        try:
            handler = self._sdpe_action_handlers.get(name)
            if handler is not None:
                handler()
                return
            default = getattr(self, f"_default_{name}", None)
            if default is not None:
                default()
        finally:
            if mutating:
                self.mutationFinished.emit()

    def set_action_handler(self, name: str, handler: Callable[[], None] | None) -> None:
        """Override one row action while retaining the shared shortcut/menu action."""

        if name not in self._sdpe_actions:
            raise ValueError(f"Unknown SDPE edit action: {name}")
        if handler is None:
            self._sdpe_action_handlers.pop(name, None)
        else:
            self._sdpe_action_handlers[name] = handler

    def set_action_enabled(self, name: str, enabled: bool) -> None:
        if name not in self._sdpe_actions:
            raise ValueError(f"Unknown SDPE edit action: {name}")
        self._sdpe_actions[name].setEnabled(enabled)

    def set_insert_handlers(
        self,
        *,
        add_item: Callable[[], None] | None = None,
        item_label: str = "Add item",
        add_group: Callable[[], None] | None = None,
        group_label: str = "Add group",
    ) -> None:
        """Configure semantic insert commands exposed through the Edit menu."""

        for name, handler, label in (
            ("add_item", add_item, item_label),
            ("add_group", add_group, group_label),
        ):
            self.set_action_handler(name, handler)
            action = self._sdpe_actions[name]
            action.setText(label)
            action.setEnabled(handler is not None)
            action.setVisible(handler is not None)

    def action(self, name: str) -> QAction:
        return self._sdpe_actions[name]

    def run_action(self, name: str) -> None:
        self._run_standard_action(name)

    def _watch_item_widget(self, widget: QWidget) -> None:
        """Forward row shortcuts from permanent cell widgets to the data view."""

        widget.installEventFilter(self)
        for child in widget.findChildren(QWidget):
            child.installEventFilter(self)

    def eventFilter(self, watched, event) -> bool:  # noqa: ANN001, N802 - Qt override signature.
        if event.type() == QEvent.Type.KeyPress:
            if isinstance(watched, QLineEdit) or (isinstance(watched, QComboBox) and watched.isEditable()):
                return super().eventFilter(watched, event)
            if self.handle_standard_key(event):
                return True
        return super().eventFilter(watched, event)

    def handle_standard_key(self, event) -> bool:  # noqa: ANN001 - accepts QKeyEvent from either Qt binding.
        """Dispatch row shortcuts even when Qt's QAction resolver is ambiguous."""

        if self.state() == QAbstractItemView.State.EditingState:
            return False
        if event.key() == Qt.Key.Key_Delete:
            action = "delete"
        elif event.matches(QKeySequence.StandardKey.Copy):
            action = "copy"
        elif event.matches(QKeySequence.StandardKey.Cut):
            action = "cut"
        elif event.matches(QKeySequence.StandardKey.Paste):
            action = "paste"
        else:
            return False
        self._run_standard_action(action)
        event.accept()
        return True

    def header_labels(self) -> list[str]:
        raise NotImplementedError

    def apply_text_filter(self, pattern: str, case_sensitive: bool = False, regex: bool = False) -> bool:
        raise NotImplementedError

    def find_text(self, pattern: str, forward: bool = True, case_sensitive: bool = False, regex: bool = False) -> bool:
        raise NotImplementedError

    def apply_display_sort(self, column: int, ascending: bool = True) -> None:
        raise NotImplementedError

    def clear_display_sort(self) -> None:
        raise NotImplementedError

    def cycle_display_sort(self, column: int) -> None:
        """Cycle a column through ascending, descending, then source order."""

        current_column = getattr(self, "_sdpe_sort_column", None)
        current_ascending = getattr(self, "_sdpe_sort_ascending", None)
        if current_column != column or current_ascending is None:
            self.apply_display_sort(column, ascending=True)
        elif current_ascending:
            self.apply_display_sort(column, ascending=False)
        else:
            self.clear_display_sort()
        signal = getattr(self, "displaySortChanged", None)
        if signal is not None:
            state = 0 if getattr(self, "_sdpe_sort_ascending", None) is None else (1 if self._sdpe_sort_ascending else 2)
            signal.emit(column, state)

    def add_standard_actions_to_menu(self, menu: QMenu) -> None:
        """Append the shared row actions to a page-specific context menu."""

        insert_actions = [self._sdpe_actions[name] for name in ("add_group", "add_item")]
        for action in insert_actions:
            if action.isVisible():
                menu.addAction(action)
        if any(action.isVisible() for action in insert_actions):
            menu.addSeparator()
        for name in ("copy", "cut", "paste", "delete"):
            menu.addAction(self._sdpe_actions[name])

    def enable_default_context_menu(self) -> None:
        self.setContextMenuPolicy(Qt.ContextMenuPolicy.ActionsContextMenu)

    def copy(self) -> None:
        self._run_standard_action("copy")

    def add_group(self) -> None:
        self._run_standard_action("add_group")

    def add_item(self) -> None:
        self._run_standard_action("add_item")

    def cut(self) -> None:
        self._run_standard_action("cut")

    def paste(self) -> None:
        self._run_standard_action("paste")

    def delete_selected(self) -> None:
        self._run_standard_action("delete")


class SDPETableWidget(SDPEDataViewMixin, QTableWidget):
    """Standard SDPE flat table with row clipboard and stable internal moves."""

    contentChanged = Signal()
    rowsReordered = Signal()
    activated = Signal(object)
    mutationStarted = Signal()
    mutationFinished = Signal()
    statusTextChanged = Signal(str)
    displaySortChanged = Signal(int, int)

    def __init__(self):
        super().__init__()
        self._init_standard_actions()
        self._sdpe_description_col: int | None = None
        self.enable_default_context_menu()
        self.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.setEditTriggers(
            QAbstractItemView.EditTrigger.DoubleClicked
            | QAbstractItemView.EditTrigger.EditKeyPressed
        )
        self.setAlternatingRowColors(True)
        self.setItemDelegate(ValidationBorderDelegate(self))
        self.setDragDropOverwriteMode(False)
        self.set_row_drag_enabled(True)
        self.horizontalHeader().sectionClicked.connect(self.cycle_display_sort)
        self.model().rowsInserted.connect(lambda *_args: self.contentChanged.emit())
        self.model().rowsRemoved.connect(lambda *_args: self.contentChanged.emit())
        self.model().rowsMoved.connect(lambda *_args: self.contentChanged.emit())
        self.model().layoutChanged.connect(lambda *_args: self.contentChanged.emit())

    def configure(
        self,
        headers: list[str],
        resize_mode: QHeaderView.ResizeMode = QHeaderView.ResizeMode.Stretch,
    ) -> None:
        """Apply the shared visual/editing contract for an SDPE data table."""

        self.setColumnCount(len(headers))
        self.setHorizontalHeaderLabels(headers)
        for col, header in enumerate(headers):
            item = self.horizontalHeaderItem(col)
            if item is None:
                continue
            if header == "En":
                item.setToolTip("Enable: checked items are emitted; unchecked items are commented out.")
            elif header == "Wk":
                item.setToolTip("Weak: checked items are wrapped by #ifndef / #define / #endif.")
        self.horizontalHeader().setSectionResizeMode(resize_mode)
        self.horizontalHeader().setStretchLastSection(resize_mode != QHeaderView.ResizeMode.Interactive)
        self.verticalHeader().setVisible(False)
        self.resizeColumnsToContents()
        self._install_description_status()

    def setCellWidget(self, row: int, column: int, widget: QWidget) -> None:  # noqa: N802 - Qt override name.
        super().setCellWidget(row, column, widget)
        self._watch_item_widget(widget)

    def set_row_drag_enabled(self, enabled: bool) -> None:
        self.setDragEnabled(enabled)
        self.setAcceptDrops(enabled)
        self.setDropIndicatorShown(enabled)
        self.setDragDropMode(
            QAbstractItemView.DragDropMode.InternalMove
            if enabled
            else QAbstractItemView.DragDropMode.NoDragDrop
        )
        self.setDefaultDropAction(Qt.DropAction.MoveAction)

    def _install_description_status(self) -> None:
        description_col = next(
            (
                col
                for col in range(self.columnCount())
                if self.horizontalHeaderItem(col)
                and "description" in self.horizontalHeaderItem(col).text().lower()
            ),
            None,
        )
        if description_col is None or getattr(self, "_sdpe_status_installed", False):
            return
        self._sdpe_description_col = description_col
        self._sdpe_status_installed = True
        self.setMouseTracking(True)

        def show_row(row: int) -> None:
            if row < 0 or row >= self.rowCount():
                return
            item = self.item(row, description_col)
            text = item.text().strip() if item is not None else ""
            if text:
                self.statusTextChanged.emit(text)

        self.cellEntered.connect(lambda row, _col: show_row(row))
        self.currentCellChanged.connect(lambda row, _col, _old_row, _old_col: show_row(row))

    def current_status_text(self) -> str:
        if self._sdpe_description_col is None or self.currentRow() < 0:
            return ""
        return self._cell_text(self.currentRow(), self._sdpe_description_col)

    def publish_current_status(self) -> None:
        text = self.current_status_text()
        if text:
            self.statusTextChanged.emit(text)

    def header_labels(self) -> list[str]:
        return [
            self.horizontalHeaderItem(col).text() if self.horizontalHeaderItem(col) else str(col)
            for col in range(self.columnCount())
        ]

    def capture_source_order(self, force: bool = False) -> None:
        for row in range(self.rowCount()):
            item = self.item(row, 0)
            if item is None:
                item = QTableWidgetItem("")
                self.setItem(row, 0, item)
            if force or item.data(_SOURCE_ORDER_ROLE) is None:
                item.setData(_SOURCE_ORDER_ROLE, row)

    def source_row_numbers(self) -> list[int]:
        blocked = self.blockSignals(True)
        try:
            self.capture_source_order()
        finally:
            self.blockSignals(blocked)
        return sorted(
            range(self.rowCount()),
            key=lambda row: int(self.item(row, 0).data(_SOURCE_ORDER_ROLE)),
        )

    def apply_text_filter(self, pattern: str, case_sensitive: bool = False, regex: bool = False) -> bool:
        matcher = _text_matcher(pattern, case_sensitive, regex)
        if matcher is None:
            return False
        blocked = self.blockSignals(True)
        try:
            for row in range(self.rowCount()):
                text = "\t".join(self._cell_text(row, col) for col in range(self.columnCount()))
                self.setRowHidden(row, not matcher(text))
        finally:
            self.blockSignals(blocked)
        return True

    def find_text(self, pattern: str, forward: bool = True, case_sensitive: bool = False, regex: bool = False) -> bool:
        matcher = _text_matcher(pattern, case_sensitive, regex)
        if matcher is None or not pattern or not self.rowCount():
            return False
        start = self.currentRow()
        offsets = range(1, self.rowCount() + 1) if forward else range(-1, -self.rowCount() - 1, -1)
        for offset in offsets:
            row = (start + offset) % self.rowCount()
            text = "\t".join(self._cell_text(row, col) for col in range(self.columnCount()))
            if matcher(text):
                self.setRowHidden(row, False)
                self.setCurrentCell(row, 0)
                self.selectRow(row)
                self.scrollToItem(self.item(row, 0))
                return True
        return False

    def apply_display_sort(self, column: int, ascending: bool = True) -> None:
        if not 0 <= column < self.columnCount():
            return
        blocked = self.blockSignals(True)
        try:
            self.capture_source_order()
            self._sdpe_display_sort_active = True
            self._sdpe_sort_column = column
            self._sdpe_sort_ascending = ascending
            # sortItems is an explicit display operation.  Keeping Qt's live
            # sorting disabled prevents a later edit from moving a row away.
            self.setSortingEnabled(False)
            order = Qt.SortOrder.AscendingOrder if ascending else Qt.SortOrder.DescendingOrder
            self.sortItems(column, order)
            self.horizontalHeader().setSortIndicator(column, order)
            self.horizontalHeader().setSortIndicatorShown(True)
        finally:
            self.blockSignals(blocked)

    def clear_display_sort(self) -> None:
        if not getattr(self, "_sdpe_display_sort_active", False):
            self._sdpe_sort_column = None
            self._sdpe_sort_ascending = None
            self.horizontalHeader().setSortIndicatorShown(False)
            return
        selected_keys, current_key, current_column = self._capture_sort_selection()
        snapshots = [self._snapshot_row(row) for row in self.source_row_numbers()]
        headers = self.header_labels()
        blocked = self.blockSignals(True)
        try:
            self.setSortingEnabled(False)
            self.setRowCount(0)
            for row, cells in enumerate(snapshots):
                self._insert_snapshot_row(row, cells, headers)
            self.capture_source_order(force=True)
            self._sdpe_display_sort_active = False
            self._sdpe_sort_column = None
            self._sdpe_sort_ascending = None
            self.horizontalHeader().setSortIndicatorShown(False)
            self._restore_sort_selection(selected_keys, current_key, current_column)
        finally:
            self.blockSignals(blocked)

    def _capture_sort_selection(self) -> tuple[set[str], str | None, int]:
        """Give selected rows a temporary identity before rebuilding source order."""

        selected_keys: set[str] = set()
        current_key = None
        for row in self.selected_row_numbers():
            item = self.item(row, 0)
            if item is None:
                continue
            key = uuid4().hex
            item.setData(_SELECTION_KEY_ROLE, key)
            selected_keys.add(key)
            if row == self.currentRow():
                current_key = key
        return selected_keys, current_key, self.currentColumn()

    def _restore_sort_selection(self, selected_keys: set[str], current_key: str | None, current_column: int) -> None:
        self.clearSelection()
        current_row = -1
        for row in range(self.rowCount()):
            item = self.item(row, 0)
            if item is None:
                continue
            key = item.data(_SELECTION_KEY_ROLE)
            item.setData(_SELECTION_KEY_ROLE, None)
            if key in selected_keys:
                self.selectRow(row)
            if key == current_key:
                current_row = row
        if current_row >= 0:
            self.setCurrentCell(current_row, min(max(current_column, 0), max(self.columnCount() - 1, 0)))

    def focusInEvent(self, event) -> None:  # noqa: N802 - Qt override name.
        super().focusInEvent(event)
        self.activated.emit(self)
        self.publish_current_status()

    def mousePressEvent(self, event) -> None:  # noqa: N802 - Qt override name.
        self.activated.emit(self)
        super().mousePressEvent(event)

    def edit(self, index, trigger, event) -> bool:  # noqa: ANN001, N802 - Qt override signature.
        if index.isValid() and self.cellWidget(index.row(), index.column()) is not None:
            return False
        return super().edit(index, trigger, event)

    def keyPressEvent(self, event) -> None:  # noqa: N802 - Qt override name.
        if self.handle_standard_key(event):
            return
        super().keyPressEvent(event)

    def selected_row_numbers(self) -> list[int]:
        rows = sorted({index.row() for index in self.selectedIndexes()})
        if not rows and self.currentRow() >= 0:
            rows = [self.currentRow()]
        return rows

    def _cell_text(self, row: int, col: int) -> str:
        widget = self.cellWidget(row, col)
        if isinstance(widget, QComboBox):
            return widget.currentText().strip()
        checkbox = _checkbox_from_widget(widget)
        if checkbox is not None:
            return "1" if checkbox.isChecked() else "0"
        item = self.item(row, col)
        return item.text().strip() if item is not None else ""

    def _snapshot_cell(self, row: int, col: int) -> dict[str, Any]:
        item = self.item(row, col)
        data: dict[str, Any] = {
            "text": item.text() if item is not None else "",
            "widget": _widget_state(self.cellWidget(row, col)),
        }
        if item is not None:
            data["flags"] = _item_flag_state(item)
            roles = {}
            for offset in range(_DATA_ROLE_COUNT):
                value = item.data(Qt.ItemDataRole.UserRole.value + offset)
                if value is not None:
                    roles[str(offset)] = _json_value(value)
            if roles:
                data["roles"] = roles
        return data

    def _snapshot_row(self, row: int) -> list[dict[str, Any]]:
        return [self._snapshot_cell(row, col) for col in range(self.columnCount())]

    def _restore_widget(self, row: int, col: int, state: dict[str, Any] | None) -> None:
        if not state:
            return
        if state.get("type") == "combo":
            combo = SDPEComboBox()
            combo.setEditable(bool(state.get("editable", True)))
            combo.addItems([str(value) for value in state.get("items", [])])
            combo.setCurrentText(str(state.get("value", "")))
            combo.currentTextChanged.connect(lambda _text: self.contentChanged.emit())
            self.setCellWidget(row, col, combo)
        elif state.get("type") == "check":
            self.setCellWidget(
                row,
                col,
                _centered_checkbox(bool(state.get("checked", False)), self.contentChanged.emit),
            )

    def _insert_snapshot_row(self, row: int, cells: list[dict[str, Any]], headers: list[str] | None = None) -> None:
        self.insertRow(row)
        source_headers = headers or []
        target_headers = [
            self.horizontalHeaderItem(col).text() if self.horizontalHeaderItem(col) else str(col)
            for col in range(self.columnCount())
        ]
        for target_col, target_header in enumerate(target_headers):
            source_col = target_col
            if source_headers and target_header in source_headers:
                source_col = source_headers.index(target_header)
            if source_col >= len(cells):
                continue
            cell = cells[source_col]
            item = QTableWidgetItem(str(cell.get("text", "")))
            if isinstance(cell.get("flags"), dict):
                _set_item_flags(item, cell["flags"])
            for offset, value in cell.get("roles", {}).items():
                item.setData(Qt.ItemDataRole.UserRole.value + int(offset), value)
            self.setItem(row, target_col, item)
            self._restore_widget(row, target_col, cell.get("widget"))

    def _clipboard_payload(self, rows: list[int]) -> dict[str, Any]:
        return {
            "sdpe_clipboard": "table_rows_v1",
            "headers": [
                self.horizontalHeaderItem(col).text() if self.horizontalHeaderItem(col) else str(col)
                for col in range(self.columnCount())
            ],
            "rows": [self._snapshot_row(row) for row in rows],
        }

    def _default_copy(self) -> None:
        rows = self.selected_row_numbers()
        if not rows:
            return
        payload = self._clipboard_payload(rows)
        mime = QMimeData()
        mime.setData(_TABLE_MIME, QByteArray(json.dumps(payload, ensure_ascii=False).encode("utf-8")))
        mime.setText(
            "\n".join(
                "\t".join(self._cell_text(row, col) for col in range(self.columnCount()))
                for row in rows
            )
        )
        QApplication.clipboard().setMimeData(mime)

    def _default_cut(self) -> None:
        rows = self.selected_row_numbers()
        if not rows:
            return
        self._default_copy()
        for row in reversed(rows):
            self.removeRow(row)
        self.contentChanged.emit()

    def _default_delete(self) -> None:
        rows = self.selected_row_numbers()
        for row in reversed(rows):
            self.removeRow(row)
        if rows:
            self.contentChanged.emit()

    def _default_paste(self) -> None:
        clipboard = QApplication.clipboard().mimeData()
        rows: list[list[dict[str, Any]]] = []
        headers: list[str] = []
        if clipboard.hasFormat(_TABLE_MIME):
            try:
                payload = json.loads(bytes(clipboard.data(_TABLE_MIME)).decode("utf-8"))
                rows = payload.get("rows", [])
                headers = payload.get("headers", [])
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError):
                rows = []
        if not rows:
            for line in clipboard.text().splitlines():
                if not line.strip():
                    continue
                rows.append([{"text": value} for value in line.split("\t")])
        if not rows:
            return
        insert_at = self.rowCount() if self.currentRow() < 0 else self.currentRow() + 1
        for offset, cells in enumerate(rows):
            self._insert_snapshot_row(insert_at + offset, cells, headers)
        self.setCurrentCell(insert_at, 0)
        self.contentChanged.emit()

    def dropEvent(self, event) -> None:  # noqa: N802 - Qt override name.
        self.clear_display_sort()
        if event.source() is not self or not self.selected_row_numbers():
            super().dropEvent(event)
            return
        self.mutationStarted.emit()
        rows = self.selected_row_numbers()
        pos = event.position().toPoint()
        target = self.rowAt(pos.y())
        if target < 0:
            target = self.rowCount()
        elif self.dropIndicatorPosition() == QAbstractItemView.DropIndicatorPosition.BelowItem:
            target += 1
        snapshots = [self._snapshot_row(row) for row in rows]
        headers = [
            self.horizontalHeaderItem(col).text() if self.horizontalHeaderItem(col) else str(col)
            for col in range(self.columnCount())
        ]
        target -= sum(1 for row in rows if row < target)
        for row in reversed(rows):
            self.removeRow(row)
        for offset, cells in enumerate(snapshots):
            self._insert_snapshot_row(target + offset, cells, headers)
        self.clearSelection()
        for row in range(target, target + len(snapshots)):
            self.selectRow(row)
        self.setCurrentCell(target, 0)
        event.acceptProposedAction()
        self.capture_source_order(force=True)
        self.rowsReordered.emit()
        self.contentChanged.emit()
        self.mutationFinished.emit()


class SDPETreeWidget(SDPEDataViewMixin, QTreeWidget):
    """Standard SDPE hierarchical table with row actions and widget-safe moves."""

    contentChanged = Signal()
    structureChanged = Signal()
    activated = Signal(object)
    mutationStarted = Signal()
    mutationFinished = Signal()
    statusTextChanged = Signal(str)
    displaySortChanged = Signal(int, int)

    def __init__(self):
        super().__init__()
        self._init_standard_actions()
        self._sdpe_default_leaf_role: str | None = None
        self._sdpe_description_col: int | None = None
        self.setEditTriggers(
            QAbstractItemView.EditTrigger.DoubleClicked
            | QAbstractItemView.EditTrigger.EditKeyPressed
        )
        self.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.setAlternatingRowColors(True)
        self.setItemDelegate(ValidationBorderDelegate(self))
        self.set_tree_drag_enabled(False)
        self.header().sectionClicked.connect(self.cycle_display_sort)

    def configure(
        self,
        headers: list[str],
        *,
        description_col: int | None = None,
        editable: bool = True,
        draggable: bool = True,
        default_context_menu: bool = True,
        leaf_role: str | None = None,
    ) -> None:
        """Apply the shared hierarchical-table behavior used by SDPE editors."""

        self.setHeaderLabels(headers)
        self._sdpe_default_leaf_role = leaf_role
        if not editable:
            self.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.set_tree_drag_enabled(draggable)
        if default_context_menu:
            self.enable_default_context_menu()
        if description_col is not None:
            self.install_description_status(description_col)

    def setItemWidget(self, item: QTreeWidgetItem, column: int, widget: QWidget) -> None:  # noqa: N802 - Qt override name.
        super().setItemWidget(item, column, widget)
        self._watch_item_widget(widget)

    def set_tree_drag_enabled(self, enabled: bool) -> None:
        self.setDragEnabled(enabled)
        self.setAcceptDrops(enabled)
        self.setDropIndicatorShown(enabled)
        self.setDragDropMode(
            QAbstractItemView.DragDropMode.InternalMove
            if enabled
            else QAbstractItemView.DragDropMode.NoDragDrop
        )
        self.setDefaultDropAction(Qt.DropAction.MoveAction)

    def install_description_status(self, description_col: int) -> None:
        if getattr(self, "_sdpe_status_installed", False):
            return
        self._sdpe_status_installed = True
        self._sdpe_description_col = description_col
        self.setMouseTracking(True)

        def show_item(item: QTreeWidgetItem | None) -> None:
            if item is None:
                return
            text = self.cell_text(item, description_col)
            if text:
                self.statusTextChanged.emit(text)

        self.itemEntered.connect(lambda item, _col: show_item(item))
        self.currentItemChanged.connect(lambda current, _previous: show_item(current))

    def current_status_text(self) -> str:
        item = self.currentItem()
        if item is None or self._sdpe_description_col is None:
            return ""
        return self.cell_text(item, self._sdpe_description_col)

    def publish_current_status(self) -> None:
        text = self.current_status_text()
        if text:
            self.statusTextChanged.emit(text)

    def header_labels(self) -> list[str]:
        return [self.headerItem().text(col) for col in range(self.columnCount())]

    def _children(self, parent: QTreeWidgetItem | None) -> list[QTreeWidgetItem]:
        count = parent.childCount() if parent is not None else self.topLevelItemCount()
        return [parent.child(index) if parent is not None else self.topLevelItem(index) for index in range(count)]

    def source_children(self, parent: QTreeWidgetItem | None) -> list[QTreeWidgetItem]:
        items = self._children(parent)
        if any(item.data(0, _SOURCE_ORDER_ROLE) is not None for item in items):
            items.sort(key=lambda item: int(item.data(0, _SOURCE_ORDER_ROLE) or 0))
        return items

    def capture_source_order(self, force: bool = False, parent: QTreeWidgetItem | None = None) -> None:
        for index, item in enumerate(self._children(parent)):
            if force or item.data(0, _SOURCE_ORDER_ROLE) is None:
                item.setData(0, _SOURCE_ORDER_ROLE, index)
            self.capture_source_order(force, item)

    def apply_text_filter(self, pattern: str, case_sensitive: bool = False, regex: bool = False) -> bool:
        matcher = _text_matcher(pattern, case_sensitive, regex)
        if matcher is None:
            return False

        def visit(item: QTreeWidgetItem) -> bool:
            own = matcher("\t".join(self.cell_text(item, col) for col in range(self.columnCount())))
            child_match = any(visit(item.child(index)) for index in range(item.childCount()))
            visible = own or child_match
            item.setHidden(not visible)
            if child_match and pattern:
                item.setExpanded(True)
            return visible

        blocked = self.blockSignals(True)
        try:
            for item in self._children(None):
                visit(item)
        finally:
            self.blockSignals(blocked)
        return True

    def find_text(self, pattern: str, forward: bool = True, case_sensitive: bool = False, regex: bool = False) -> bool:
        matcher = _text_matcher(pattern, case_sensitive, regex)
        if matcher is None or not pattern:
            return False
        items = list(self.iter_items())
        if not items:
            return False
        current = self.currentItem()
        start = items.index(current) if current in items else -1
        offsets = range(1, len(items) + 1) if forward else range(-1, -len(items) - 1, -1)
        for offset in offsets:
            item = items[(start + offset) % len(items)]
            text = "\t".join(self.cell_text(item, col) for col in range(self.columnCount()))
            if matcher(text):
                item.setHidden(False)
                self.setCurrentItem(item, 0)
                self.scrollToItem(item)
                return True
        return False

    def apply_display_sort(self, column: int, ascending: bool = True) -> None:
        if not 0 <= column < self.columnCount():
            return
        blocked = self.blockSignals(True)
        try:
            self.capture_source_order()
            self._sdpe_display_sort_active = True
            self._sdpe_sort_column = column
            self._sdpe_sort_ascending = ascending
            # Do not let the model re-sort while a user is editing a row.
            self.setSortingEnabled(False)
            order = Qt.SortOrder.AscendingOrder if ascending else Qt.SortOrder.DescendingOrder
            self.sortItems(column, order)
            self.header().setSortIndicator(column, order)
            self.header().setSortIndicatorShown(True)
        finally:
            self.blockSignals(blocked)

    def clear_display_sort(self) -> None:
        if not getattr(self, "_sdpe_display_sort_active", False):
            self._sdpe_sort_column = None
            self._sdpe_sort_ascending = None
            self.header().setSortIndicatorShown(False)
            return
        selected_keys, current_key, current_column = self._capture_sort_selection()
        states = self._capture_drop_widgets()
        blocked = self.blockSignals(True)
        try:
            self.setSortingEnabled(False)

            def restore(parent: QTreeWidgetItem | None) -> None:
                items = self._children(parent)
                items.sort(key=lambda item: int(item.data(0, _SOURCE_ORDER_ROLE) or 0))
                if parent is None:
                    while self.topLevelItemCount():
                        self.takeTopLevelItem(0)
                    for item in items:
                        self.addTopLevelItem(item)
                else:
                    while parent.childCount():
                        parent.takeChild(0)
                    for item in items:
                        parent.addChild(item)
                for item in items:
                    restore(item)

            restore(None)
            self._restore_drop_widgets(states)
            self.capture_source_order(force=True)
            self._sdpe_display_sort_active = False
            self._sdpe_sort_column = None
            self._sdpe_sort_ascending = None
            self.header().setSortIndicatorShown(False)
            self._restore_sort_selection(selected_keys, current_key, current_column)
        finally:
            self.blockSignals(blocked)

    def _capture_sort_selection(self) -> tuple[set[str], str | None, int]:
        selected_keys: set[str] = set()
        current = self.currentItem()
        current_key = None
        for item in unique_tree_items(self.selectedItems()):
            key = uuid4().hex
            item.setData(0, _SELECTION_KEY_ROLE, key)
            selected_keys.add(key)
            if item is current:
                current_key = key
        if current is not None and current_key is None:
            key = uuid4().hex
            current.setData(0, _SELECTION_KEY_ROLE, key)
            selected_keys.add(key)
            current_key = key
        return selected_keys, current_key, self.currentColumn()

    def _restore_sort_selection(self, selected_keys: set[str], current_key: str | None, current_column: int) -> None:
        self.clearSelection()
        current_item = None
        for item in self.iter_items():
            key = item.data(0, _SELECTION_KEY_ROLE)
            item.setData(0, _SELECTION_KEY_ROLE, None)
            if key in selected_keys:
                item.setSelected(True)
            if key == current_key:
                current_item = item
        if current_item is not None:
            self.setCurrentItem(current_item, min(max(current_column, 0), max(self.columnCount() - 1, 0)))

    def focusInEvent(self, event) -> None:  # noqa: N802 - Qt override name.
        super().focusInEvent(event)
        self.activated.emit(self)
        self.publish_current_status()

    def mousePressEvent(self, event) -> None:  # noqa: N802 - Qt override name.
        self.activated.emit(self)
        super().mousePressEvent(event)

    def edit(self, index, trigger, event) -> bool:  # noqa: ANN001, N802 - Qt override signature.
        if index.isValid():
            if bool(index.data(READ_ONLY_CELL_ROLE)):
                return False
            item = self.itemFromIndex(index)
            if item is not None and self.itemWidget(item, index.column()) is not None:
                return False
        return super().edit(index, trigger, event)

    def keyPressEvent(self, event) -> None:  # noqa: N802 - Qt override name.
        if self.handle_standard_key(event):
            return
        super().keyPressEvent(event)

    def iter_items(self, parent: QTreeWidgetItem | None = None):
        """Yield every item recursively, allowing arbitrary group depth."""

        for item in self.source_children(parent):
            yield item
            yield from self.iter_items(item)

    def iter_leaf_items(self, roles: set[str] | None = None):
        """Yield recursive leaves, optionally filtering the UserRole item kind."""

        for item in self.iter_items():
            role = item.data(0, Qt.ItemDataRole.UserRole)
            if item.childCount() == 0 and (roles is None or role in roles):
                yield item

    def group_path(self, item: QTreeWidgetItem, separator: str = " / ") -> str:
        """Return the complete group path above an item."""

        names: list[str] = []
        parent = item if item.data(0, Qt.ItemDataRole.UserRole) == "group" else item.parent()
        while parent is not None:
            if parent.data(0, Qt.ItemDataRole.UserRole) == "group":
                names.append(parent.text(0).strip())
            parent = parent.parent()
        return separator.join(reversed([name for name in names if name]))

    def ensure_group_path(
        self,
        path: str,
        factory: Callable[[str], QTreeWidgetItem],
        separator: str = " / ",
    ) -> QTreeWidgetItem:
        """Find or create a nested group path using a page-owned item factory."""

        parts = [part.strip() for part in path.split(separator) if part.strip()]
        if not parts:
            parts = ["Group"]
        parent: QTreeWidgetItem | None = None
        for part in parts:
            count = parent.childCount() if parent is not None else self.topLevelItemCount()
            match = None
            for index in range(count):
                item = parent.child(index) if parent is not None else self.topLevelItem(index)
                if item.data(0, Qt.ItemDataRole.UserRole) == "group" and item.text(0).strip() == part:
                    match = item
                    break
            if match is None:
                match = factory(part)
                if parent is None:
                    self.addTopLevelItem(match)
                else:
                    parent.addChild(match)
            parent = match
        return parent

    def insert_group_near_current(self, item: QTreeWidgetItem) -> None:
        """Insert a group as a child of the selected group or as its sibling."""

        current = self.currentItem()
        if current is None:
            self.addTopLevelItem(item)
            return
        if current.data(0, Qt.ItemDataRole.UserRole) == "group":
            current.addChild(item)
            current.setExpanded(True)
            return
        parent = current.parent()
        if parent is None:
            self.addTopLevelItem(item)
        else:
            parent.insertChild(parent.indexOfChild(current) + 1, item)

    def cell_text(self, item: QTreeWidgetItem, col: int) -> str:
        widget = self.itemWidget(item, col)
        if isinstance(widget, QComboBox):
            return widget.currentText().strip()
        checkbox = _checkbox_from_widget(widget)
        if checkbox is not None:
            return "1" if checkbox.isChecked() else "0"
        value = item.data(col, Qt.ItemDataRole.UserRole)
        if col != 0 and value is not None:
            return str(value).strip()
        return item.text(col).strip()

    def selected_top_level_items(self) -> list[QTreeWidgetItem]:
        """Return selected items excluding descendants of another selected item."""

        selected = unique_tree_items(self.selectedItems())
        selected_ids = {id(item) for item in selected}
        result = []
        for item in selected:
            parent = item.parent()
            while parent is not None and id(parent) not in selected_ids:
                parent = parent.parent()
            if parent is None:
                result.append(item)
        if not result and self.currentItem() is not None:
            result = [self.currentItem()]
        return result

    def _snapshot_item(self, item: QTreeWidgetItem) -> dict[str, Any]:
        cells = []
        for col in range(self.columnCount()):
            roles = {}
            for offset in range(_DATA_ROLE_COUNT):
                value = item.data(col, Qt.ItemDataRole.UserRole.value + offset)
                if value is not None:
                    roles[str(offset)] = _json_value(value)
            cells.append(
                {
                    "text": item.text(col),
                    "roles": roles,
                    "widget": _widget_state(self.itemWidget(item, col)),
                }
            )
        return {
            "cells": cells,
            "flags": _item_flag_state(item),
            "expanded": item.isExpanded(),
            "children": [self._snapshot_item(item.child(index)) for index in range(item.childCount())],
        }

    def _restore_item(self, data: dict[str, Any]) -> QTreeWidgetItem:
        cells = data.get("cells", [])
        item = QTreeWidgetItem([str(cell.get("text", "")) for cell in cells])
        if isinstance(data.get("flags"), dict):
            _set_item_flags(item, data["flags"])
        for col, cell in enumerate(cells[: self.columnCount()]):
            for offset, value in cell.get("roles", {}).items():
                item.setData(col, Qt.ItemDataRole.UserRole.value + int(offset), value)
        for child_data in data.get("children", []):
            item.addChild(self._restore_item(child_data))
        item.setExpanded(bool(data.get("expanded", True)))
        return item

    def _restore_item_widgets(self, item: QTreeWidgetItem, data: dict[str, Any]) -> None:
        cells = data.get("cells", [])
        for col, cell in enumerate(cells[: self.columnCount()]):
            state = cell.get("widget")
            if not state:
                continue
            if state.get("type") == "combo":
                combo = SDPEComboBox()
                combo.setEditable(bool(state.get("editable", True)))
                combo.addItems([str(value) for value in state.get("items", [])])
                combo.setCurrentText(str(state.get("value", "")))
                combo.currentTextChanged.connect(
                    lambda text, i=item, c=col: (
                        i.setData(c, Qt.ItemDataRole.UserRole, text),
                        self.contentChanged.emit(),
                    )
                )
                self.setItemWidget(item, col, combo)
            elif state.get("type") == "check":
                def changed(i=item, c=col) -> None:
                    checkbox = _checkbox_from_widget(self.itemWidget(i, c))
                    if checkbox is not None:
                        i.setData(c, Qt.ItemDataRole.UserRole, checkbox.isChecked())
                    self.contentChanged.emit()

                self.setItemWidget(item, col, _centered_checkbox(bool(state.get("checked", False)), changed))
        for index, child_data in enumerate(data.get("children", [])):
            if index < item.childCount():
                self._restore_item_widgets(item.child(index), child_data)

    def _insert_snapshot(
        self,
        parent: QTreeWidgetItem | None,
        index: int,
        data: dict[str, Any],
    ) -> QTreeWidgetItem:
        item = self._restore_item(data)
        if parent is None:
            self.insertTopLevelItem(index, item)
        else:
            parent.insertChild(index, item)
        self._restore_item_widgets(item, data)
        return item

    def _default_copy(self) -> None:
        items = self.selected_top_level_items()
        if not items:
            return
        payload = {
            "sdpe_clipboard": "tree_items_v1",
            "columns": self.columnCount(),
            "headers": [self.headerItem().text(col) for col in range(self.columnCount())],
            "items": [self._snapshot_item(item) for item in items],
        }
        mime = QMimeData()
        mime.setData(_TREE_MIME, QByteArray(json.dumps(payload, ensure_ascii=False).encode("utf-8")))
        mime.setText(
            "\n".join(
                "\t".join(self.cell_text(item, col) for col in range(self.columnCount()))
                for item in items
            )
        )
        QApplication.clipboard().setMimeData(mime)

    def _remove_items(self, items: list[QTreeWidgetItem]) -> None:
        for item in reversed(items):
            parent = item.parent()
            if parent is None:
                self.takeTopLevelItem(self.indexOfTopLevelItem(item))
            else:
                parent.takeChild(parent.indexOfChild(item))

    def _default_cut(self) -> None:
        items = self.selected_top_level_items()
        if not items:
            return
        self._default_copy()
        self._remove_items(items)
        self.contentChanged.emit()
        self.structureChanged.emit()

    def _default_delete(self) -> None:
        items = self.selected_top_level_items()
        if not items:
            return
        self._remove_items(items)
        self.contentChanged.emit()
        self.structureChanged.emit()

    def _paste_target(self) -> tuple[QTreeWidgetItem | None, int]:
        current = self.currentItem()
        if current is None:
            return None, self.topLevelItemCount()
        if current.data(0, Qt.ItemDataRole.UserRole) == "group":
            return current, current.childCount()
        parent = current.parent()
        if parent is None:
            return None, self.indexOfTopLevelItem(current) + 1
        return parent, parent.indexOfChild(current) + 1

    def _default_paste(self) -> None:
        mime = QApplication.clipboard().mimeData()
        snapshots = []
        if mime.hasFormat(_TREE_MIME):
            try:
                payload = json.loads(bytes(mime.data(_TREE_MIME)).decode("utf-8"))
                headers = [self.headerItem().text(col) for col in range(self.columnCount())]
                if payload.get("headers") == headers:
                    snapshots = payload.get("items", [])
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError):
                snapshots = []
        if not snapshots:
            for line in mime.text().splitlines():
                if not line.strip():
                    continue
                cells = [{"text": value, "roles": {}, "widget": None} for value in line.split("\t")]
                if self._sdpe_default_leaf_role and cells:
                    cells[0]["roles"] = {"0": self._sdpe_default_leaf_role}
                snapshots.append({"cells": cells, "children": [], "expanded": True})
        if not snapshots:
            return
        parent, index = self._paste_target()
        first = None
        for offset, snapshot in enumerate(snapshots):
            item = self._insert_snapshot(parent, index + offset, snapshot)
            if first is None:
                first = item
        if parent is not None:
            parent.setExpanded(True)
        if first is not None:
            self.setCurrentItem(first)
        self.contentChanged.emit()
        self.structureChanged.emit()

    def _capture_drop_widgets(self) -> dict[str, list[dict[str, Any] | None]]:
        states = {}
        for item in self.iter_items():
            key = uuid4().hex
            item.setData(0, _WIDGET_KEY_ROLE, key)
            states[key] = [_widget_state(self.itemWidget(item, col)) for col in range(self.columnCount())]
        return states

    def _restore_drop_widgets(self, states: dict[str, list[dict[str, Any] | None]]) -> None:
        for item in self.iter_items():
            key = item.data(0, _WIDGET_KEY_ROLE)
            item.setData(0, _WIDGET_KEY_ROLE, None)
            if not key or key not in states:
                continue
            self._restore_item_widgets(item, {"cells": [{"widget": state} for state in states[key]]})

    def dropEvent(self, event) -> None:  # noqa: N802 - Qt override name.
        self.clear_display_sort()
        self.mutationStarted.emit()
        states = self._capture_drop_widgets() if event.source() is self else {}
        super().dropEvent(event)
        if states:
            self._restore_drop_widgets(states)
        self.structureChanged.emit()
        self.contentChanged.emit()
        self.capture_source_order(force=True)
        self.mutationFinished.emit()


def unique_tree_items(items) -> list:  # noqa: ANN001 - accepts Qt item sequences from PyQt/PySide.
    """Return tree items once, preserving object identity order."""

    unique = []
    seen: set[int] = set()
    for item in items:
        key = id(item)
        if key in seen:
            continue
        seen.add(key)
        unique.append(item)
    return unique
