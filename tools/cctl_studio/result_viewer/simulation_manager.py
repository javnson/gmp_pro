"""QProcess supervisor for GMP CCTL simulation executables."""

from __future__ import annotations

import json
from pathlib import Path

from PyQt5 import QtCore


class SimulationProcessManager(QtCore.QObject):
    """Launch and control one CCTL simulator through its stdio protocol."""

    message_received = QtCore.pyqtSignal(dict)
    log_received = QtCore.pyqtSignal(str)
    state_changed = QtCore.pyqtSignal(str)
    outputs_ready = QtCore.pyqtSignal(list)
    process_finished = QtCore.pyqtSignal(int, int)

    def __init__(self, parent: QtCore.QObject | None = None):
        super().__init__(parent)
        self.process = QtCore.QProcess(self)
        self.process.setProcessChannelMode(QtCore.QProcess.SeparateChannels)
        self.process.readyReadStandardOutput.connect(self._read_stdout)
        self.process.readyReadStandardError.connect(self._read_stderr)
        self.process.started.connect(lambda: self._set_state("initializing"))
        self.process.errorOccurred.connect(self._process_error)
        self.process.finished.connect(self._process_finished)
        self._stdout_buffer = bytearray()
        self._auto_start = False
        self.state = "idle"

    def is_active(self) -> bool:
        """Return whether the managed native process still exists."""
        return self.process.state() != QtCore.QProcess.NotRunning

    def launch(
        self,
        executable: Path,
        output_path: Path,
        duration_s: float,
        extra_arguments: list[str] | None = None,
        auto_start: bool = True,
    ) -> None:
        """Start one simulator and wait for its protocol-ready message."""
        if self.is_active():
            raise RuntimeError("a CCTL simulation is already active")
        executable = executable.resolve()
        if not executable.is_file():
            raise FileNotFoundError(f"simulator executable not found: {executable}")
        output_path = output_path.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        arguments = [
            "--supervised",
            "--wait-for-start",
            "--no-pause",
            "--duration",
            f"{duration_s:.17g}",
            "--output",
            str(output_path),
        ]
        arguments.extend(extra_arguments or [])
        self._stdout_buffer.clear()
        self._auto_start = auto_start
        self._set_state("starting")
        self.process.setWorkingDirectory(str(executable.parent))
        self.process.setProgram(str(executable))
        self.process.setArguments(arguments)
        self.process.start()

    def start_simulation(self) -> None:
        """Release a simulator waiting in the ready state."""
        self.send_command({"command": "start"})

    def pause(self) -> None:
        """Pause at the next simulation-step boundary."""
        self.send_command({"command": "pause"})

    def resume(self) -> None:
        """Resume a paused simulation."""
        self.send_command({"command": "resume"})

    def stop(self) -> None:
        """Request graceful model finalization and CSV queue draining."""
        self.send_command({"command": "stop"})

    def set_duration(self, seconds: float) -> None:
        """Set the finite target; zero selects an unlimited run."""
        self.send_command({"command": "set_duration", "seconds": seconds})

    def send_command(self, command: dict) -> None:
        """Serialize one JSON command to the simulator control pipe."""
        if not self.is_active():
            return
        self.process.write(self.encode_command(command))

    @staticmethod
    def encode_command(command: dict) -> bytes:
        """Encode one protocol command as compact newline-delimited JSON."""
        return (json.dumps(command, separators=(",", ":")) + "\n").encode("utf-8")

    def _set_state(self, state: str) -> None:
        if state != self.state:
            self.state = state
            self.state_changed.emit(state)

    @QtCore.pyqtSlot()
    def _read_stdout(self) -> None:
        self._stdout_buffer.extend(bytes(self.process.readAllStandardOutput()))
        while True:
            newline = self._stdout_buffer.find(b"\n")
            if newline < 0:
                break
            raw = bytes(self._stdout_buffer[:newline]).rstrip(b"\r")
            del self._stdout_buffer[: newline + 1]
            if not raw:
                continue
            try:
                message = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self.log_received.emit(raw.decode("utf-8", errors="replace"))
                continue
            self._dispatch_message(message)

    def _dispatch_message(self, message: dict) -> None:
        message_type = message.get("type")
        state = message.get("state")
        if isinstance(state, str):
            self._set_state(state)
        if message_type == "ready":
            outputs = [
                item["path"] for item in message.get("outputs", [])
                if isinstance(item, dict) and isinstance(item.get("path"), str)
            ]
            if outputs:
                self.outputs_ready.emit(outputs)
            if self._auto_start:
                self._auto_start = False
                self.start_simulation()
        elif message_type == "summary":
            self._set_state("completed" if message.get("success") else "failed")
        self.message_received.emit(message)

    @QtCore.pyqtSlot()
    def _read_stderr(self) -> None:
        text = bytes(self.process.readAllStandardError()).decode(
            "utf-8", errors="replace"
        )
        if text:
            self.log_received.emit(text.rstrip("\r\n"))

    @QtCore.pyqtSlot(QtCore.QProcess.ProcessError)
    def _process_error(self, error: QtCore.QProcess.ProcessError) -> None:
        self.log_received.emit(f"QProcess error: {self.process.errorString()}")
        if error == QtCore.QProcess.FailedToStart:
            self._set_state("failed")

    @QtCore.pyqtSlot(int, QtCore.QProcess.ExitStatus)
    def _process_finished(self, exit_code: int, exit_status) -> None:
        if self.state not in {"completed", "failed"}:
            self._set_state("completed" if exit_code == 0 else "failed")
        self.process_finished.emit(exit_code, int(exit_status))
