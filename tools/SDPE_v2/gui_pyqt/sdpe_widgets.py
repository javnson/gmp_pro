"""Shared Qt widgets for the SDPE v2 GUI."""

from __future__ import annotations

try:
    from PyQt6.QtCore import Qt
    from PyQt6.QtGui import QColor, QPen
    from PyQt6.QtWidgets import (
        QAbstractItemView,
        QApplication,
        QComboBox,
        QStyledItemDelegate,
        QTableWidget,
        QTreeWidget,
    )
except ImportError:  # pragma: no cover - depends on local desktop environment.
    from PySide6.QtCore import Qt
    from PySide6.QtGui import QColor, QPen
    from PySide6.QtWidgets import (
        QAbstractItemView,
        QApplication,
        QComboBox,
        QStyledItemDelegate,
        QTableWidget,
        QTreeWidget,
    )


VALIDATION_BORDER_ROLE = Qt.ItemDataRole.UserRole.value + 101


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
    """Draw a validation outline without changing themed text/background colors."""

    def paint(self, painter, option, index) -> None:  # noqa: ANN001, N802 - Qt override signature.
        super().paint(painter, option, index)
        if not index.data(VALIDATION_BORDER_ROLE):
            return
        painter.save()
        pen = QPen(QColor(220, 40, 40))
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawRect(option.rect.adjusted(1, 1, -2, -2))
        painter.restore()


class SDPETableWidget(QTableWidget):
    """SDPE table with cell-widget aware editing behavior."""

    def edit(self, index, trigger, event) -> bool:  # noqa: ANN001, N802 - Qt override signature.
        if index.isValid() and self.cellWidget(index.row(), index.column()) is not None:
            return False
        return super().edit(index, trigger, event)


class SDPETreeWidget(QTreeWidget):
    """SDPE tree with item-widget aware editing behavior."""

    def __init__(self):
        super().__init__()
        self.setEditTriggers(
            QAbstractItemView.EditTrigger.SelectedClicked
            | QAbstractItemView.EditTrigger.EditKeyPressed
        )

    def edit(self, index, trigger, event) -> bool:  # noqa: ANN001, N802 - Qt override signature.
        if index.isValid():
            item = self.itemFromIndex(index)
            if item is not None and self.itemWidget(item, index.column()) is not None:
                return False
        return super().edit(index, trigger, event)


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
