"""Target-reported Memory Perspective and Tunable resource discovery."""

from __future__ import annotations

from typing import List

from PyQt5.QtCore import QObject, QTimer, pyqtSignal

from core_datalink import HermesDatalinkQt
from apis.protocol import (
    MemoryRegion,
    TunableParameter,
    parse_memory_descriptor,
    parse_tunable_descriptor,
)

TunableItem = TunableParameter


class ResourceDiscovery(QObject):
    """Discover indexed target resources over compact ``base + 1`` commands."""

    memory_regions_changed = pyqtSignal(object)
    tunable_items_changed = pyqtSignal(object)
    discovery_error = pyqtSignal(str)

    def __init__(self, hermes: HermesDatalinkQt) -> None:
        super().__init__()
        self.hermes = hermes
        self.memory_regions: List[MemoryRegion] = []
        self.tunable_items: List[TunableItem] = []
        self.memory_base_command = 0x50
        self.tunable_base_command = 0x30
        self._memory_active = False
        self._tunable_active = False
        self._memory_next_id = 0
        self._tunable_next_id = 0
        self._memory_total = 0
        self._tunable_total = 0
        self.hermes.sig_bus_event.connect(self._on_bus_event)

    def discover_memory(self, base_command: int = 0x50) -> None:
        """Request all registered memory descriptors from the target."""
        if not self.hermes.running:
            self.discovery_error.emit("Connect to a target before discovering memory regions.")
            return
        self.memory_base_command = base_command & 0xFF
        self.memory_regions = []
        self._memory_next_id = 0
        self._memory_total = 0
        self._memory_active = True
        self._send_memory_query()

    def discover_tunables(self, base_command: int = 0x30) -> None:
        """Request all registered tunable descriptors from the target."""
        if not self.hermes.running:
            self.discovery_error.emit("Connect to a target before discovering tunable parameters.")
            return
        self.tunable_base_command = base_command & 0xFF
        self.tunable_items = []
        self._tunable_next_id = 0
        self._tunable_total = 0
        self._tunable_active = True
        self._send_tunable_query()

    def _send_memory_query(self) -> None:
        """Send one indexed memory descriptor query."""
        self.hermes.send_frame(
            0x01,
            (self.memory_base_command + 1) & 0xFF,
            bytes((self._memory_next_id,)),
            priority=0,
        )

    def _send_tunable_query(self) -> None:
        """Send one indexed tunable descriptor query."""
        self.hermes.send_frame(
            0x01,
            (self.tunable_base_command + 1) & 0xFF,
            bytes((self._tunable_next_id,)),
            priority=0,
        )

    @classmethod
    def parse_memory_descriptor(cls, payload: bytes) -> tuple[int, int, MemoryRegion | None]:
        """Parse a minimal v2 or legacy v1 memory descriptor response."""
        return parse_memory_descriptor(payload)

    @classmethod
    def parse_tunable_descriptor(cls, payload: bytes) -> tuple[int, int, TunableItem | None]:
        """Parse a compact v2 or legacy v1 tunable descriptor response."""
        return parse_tunable_descriptor(payload)

    def _on_bus_event(self, event: dict) -> None:
        """Advance active discovery sequences from valid Data Link responses."""
        if (
            event.get("type") != "DL"
            or event.get("dir") != "RX"
            or not event.get("dl_crc_ok")
        ):
            return
        command = event.get("dl_cmd")
        payload = event.get("dl_payload", b"")
        try:
            if self._memory_active and command == ((self.memory_base_command + 1) & 0xFF):
                total, region_id, descriptor = self.parse_memory_descriptor(payload)
                self._memory_total = total
                if total == 0 and descriptor is None:
                    self._memory_active = False
                    self.memory_regions_changed.emit([])
                    return
                if region_id != self._memory_next_id or descriptor is None:
                    raise ValueError(f"Memory descriptor {self._memory_next_id} was rejected.")
                self.memory_regions.append(descriptor)
                self._memory_next_id += 1
                if self._memory_next_id < self._memory_total:
                    QTimer.singleShot(0, self._send_memory_query)
                else:
                    self._memory_active = False
                    self.memory_regions_changed.emit(list(self.memory_regions))
            elif self._tunable_active and command == ((self.tunable_base_command + 1) & 0xFF):
                total, item_id, descriptor = self.parse_tunable_descriptor(payload)
                self._tunable_total = total
                if total == 0 and descriptor is None:
                    self._tunable_active = False
                    self.tunable_items_changed.emit([])
                    return
                if item_id != self._tunable_next_id or descriptor is None:
                    raise ValueError(f"Tunable descriptor {self._tunable_next_id} was rejected.")
                self.tunable_items.append(descriptor)
                self._tunable_next_id += 1
                if self._tunable_next_id < self._tunable_total:
                    QTimer.singleShot(0, self._send_tunable_query)
                else:
                    self._tunable_active = False
                    self.tunable_items_changed.emit(list(self.tunable_items))
        except ValueError as error:
            self._memory_active = False
            self._tunable_active = False
            self.discovery_error.emit(str(error))
