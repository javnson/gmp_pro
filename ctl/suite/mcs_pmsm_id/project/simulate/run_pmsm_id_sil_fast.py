"""Fast averaged-plant SIL runner for the PMSM offline identifier.

This runner talks to the same native controller executable and uses the same
packed UDP ABI as MCS_STD_PMSM_MODEL.slx.  It is intended for rapid state-
machine and parameter-regression work; use run_pmsm_id_sil.m with a 1 us
plant step for final switching/dead-time correlation.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import socket
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


TX = struct.Struct("<d8I8I16d")
RX = struct.Struct("<d24I16d8i")
FRAME_HEADER = struct.Struct(">IHHIQI")
SESSION_PAYLOAD = struct.Struct(">HHII16s")
STATE_PAYLOAD = struct.Struct(">HHIQ")

FRAME_MAGIC = 0x474D5054
PROTOCOL_VERSION = 1
DATA_REQUEST = 1
DATA_RESPONSE = 2
SIMULATION_STATE = 8
SESSION_HELLO = 10
SESSION_HELLO_RESPONSE = 11
SIMULATION_RUNNING = 2
SIMULATION_COMPLETED = 3
SIMULATION_ABORTED = 4


class SilUdpClient:
    """Minimal unified GMP SIL UDP client used by the fast plant."""

    def __init__(self, config_path: Path) -> None:
        config = json.loads(config_path.read_text(encoding="utf-8"))
        if config.get("transport") != "udp" or config.get("role") != "server":
            raise ValueError("the fast runner requires a UDP controller/server configuration")
        if config.get("protocol_version") != PROTOCOL_VERSION:
            raise ValueError("unsupported GMP SIL protocol version")
        self.connection_id = bytes.fromhex(config["connection_id"])
        if len(self.connection_id) != 16:
            raise ValueError("connection_id must contain 16 bytes")
        self.request_size = int(config["simulink_to_controller_bytes"])
        self.response_size = int(config["controller_to_simulink_bytes"])
        if self.request_size != RX.size or self.response_size != TX.size:
            raise ValueError(
                f"network ABI is {self.request_size}/{self.response_size} bytes; "
                f"the runner requires {RX.size}/{TX.size}"
            )
        self.controller = (config["target_address"], int(config["receive_port"]))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if sys.platform == "win32" and hasattr(socket, "SIO_UDP_CONNRESET"):
            self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        self.sock.bind(("127.0.0.1", int(config["transmit_port"])))
        self.sock.settimeout(0.25)
        self.next_sequence = 1
        self.state_sequence = 0x8000000000000000
        self.connected = False

    @staticmethod
    def _frame(kind: int, sequence: int, payload: bytes, flags: int = 0) -> bytes:
        return FRAME_HEADER.pack(
            FRAME_MAGIC, PROTOCOL_VERSION, kind, flags, sequence, len(payload)
        ) + payload

    def _receive(self) -> tuple[int, int, int, bytes]:
        datagram, sender = self.sock.recvfrom(24 + 65536)
        if sender != self.controller:
            raise RuntimeError(f"received a SIL frame from unexpected peer {sender}")
        if len(datagram) < FRAME_HEADER.size:
            raise RuntimeError("received a SIL datagram shorter than its header")
        magic, version, kind, flags, sequence, payload_size = FRAME_HEADER.unpack_from(datagram)
        payload = datagram[FRAME_HEADER.size:]
        if magic != FRAME_MAGIC or version != PROTOCOL_VERSION:
            raise RuntimeError("received an invalid GMP SIL frame header")
        if payload_size != len(payload):
            raise RuntimeError("received a GMP SIL frame with an invalid payload length")
        return kind, sequence, flags, payload

    def connect(self, timeout_s: float = 10.0) -> None:
        payload = SESSION_PAYLOAD.pack(
            PROTOCOL_VERSION, 0, self.request_size, self.response_size, self.connection_id
        )
        sequence = self.next_sequence
        self.next_sequence += 1
        hello = self._frame(SESSION_HELLO, sequence, payload)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.sock.sendto(hello, self.controller)
            try:
                kind, response_sequence, _, response = self._receive()
            except (TimeoutError, socket.timeout, ConnectionResetError, OSError):
                continue
            if kind != SESSION_HELLO_RESPONSE or response_sequence != sequence:
                continue
            if response != payload:
                raise RuntimeError("controller rejected the SIL session or ABI descriptor")
            self.connected = True
            self.sock.settimeout(10.0)
            return
        raise TimeoutError("timed out establishing the unified GMP SIL session")

    def notify_state(self, state: int, major_step: int) -> None:
        payload = STATE_PAYLOAD.pack(PROTOCOL_VERSION, state, 0, major_step)
        frame = self._frame(SIMULATION_STATE, self.state_sequence, payload)
        self.state_sequence += 1
        copies = 3 if state in (SIMULATION_COMPLETED, SIMULATION_ABORTED) else 1
        for _ in range(copies):
            self.sock.sendto(frame, self.controller)

    def exchange(self, payload: bytes) -> bytes:
        if not self.connected or len(payload) != self.request_size:
            raise RuntimeError("invalid GMP SIL request state or payload size")
        sequence = self.next_sequence
        self.next_sequence += 1
        self.sock.sendto(self._frame(DATA_REQUEST, sequence, payload), self.controller)
        kind, response_sequence, _, response = self._receive()
        if kind != DATA_RESPONSE or response_sequence != sequence:
            raise RuntimeError("controller returned an unexpected GMP SIL response")
        if len(response) != self.response_size:
            raise RuntimeError("controller returned an invalid GMP SIL response size")
        return response

    def close(self) -> None:
        self.connected = False
        self.sock.close()


@dataclass
class Plant:
    rs: float = 4.7
    ld: float = 8.5e-3
    lq: float = 8.5e-3
    flux: float = 0.0038197
    pole_pairs: int = 4
    dc_bus: float = 24.0
    # Representative motor-plus-load mechanics for a multi-second offline-ID
    # trace.  Electrical parameters remain those of the detailed PMSM model.
    inertia: float = 5.0e-4
    friction: float = 1.0e-4
    load_torque: float = 2.0e-4
    mosfet_resistance: float = 0.1
    diode_drop: float = 0.3
    switching_frequency: float = 20e3
    deadtime: float = 1e-6
    compare_period: float = 2500.0
    encoder_counts: int = 16384
    encoder_offset_pu: float = 0.0999145508
    adc_reference: float = 3.3
    adc_counts: int = 4096
    current_sensitivity: float = 0.1
    current_bias: float = 1.65
    voltage_sensitivity: float = 0.04049127
    encoder_fault: str = "none"

    id: float = 0.0
    iq: float = 0.0
    omega_m: float = 0.0
    theta_m: float = 0.0
    theta_e: float = 0.0
    voltage_angle: float | None = None
    omega_e_estimate: float = 0.0
    last_oid_state: int = 2
    calibration_command_e: float = 0.0
    last_encoder_substate: int = 0

    @staticmethod
    def _sign(value: float, threshold: float = 1e-4) -> float:
        if value > threshold:
            return 1.0
        if value < -threshold:
            return -1.0
        return 0.0

    def step(
        self, enable: float, pwm: tuple[int, int, int], dt: float, oid_state: int,
        encoder_substate: int
    ) -> tuple[list[float], list[float]]:
        sin_t = math.sin(self.theta_e)
        cos_t = math.cos(self.theta_e)
        i_alpha = self.id * cos_t - self.iq * sin_t
        i_beta = self.id * sin_t + self.iq * cos_t
        currents = [
            i_alpha,
            -0.5 * i_alpha + 0.5 * math.sqrt(3.0) * i_beta,
            -0.5 * i_alpha - 0.5 * math.sqrt(3.0) * i_beta,
        ]

        if enable >= 0.5:
            # The simulation target intentionally uses negative PWM compare
            # logic, matching the detailed universal-driver model.
            duties = [
                1.0 - max(0.0, min(1.0, value / self.compare_period)) for value in pwm
            ]
            # The fast plant exposes dead time during Rs/DT regression, then
            # represents the compensated inverter used by the later L/flux
            # stages.  The detailed switching model remains the authority for
            # validating the actual compare-level compensation waveform.
            deadtime_drop = (
                self.dc_bus + 2.0 * self.diode_drop
            ) * self.deadtime * self.switching_frequency
            if oid_state != 4:
                deadtime_drop = 0.0
            legs = []
            for duty, current in zip(duties, currents):
                direction = self._sign(current)
                conduction_drop = self.mosfet_resistance * current
                legs.append(duty * self.dc_bus - direction * deadtime_drop - conduction_drop)
        else:
            legs = [0.0, 0.0, 0.0]

        neutral = sum(legs) / 3.0
        phase_voltage = [value - neutral for value in legs]
        v_alpha = (2.0 / 3.0) * (phase_voltage[0] - 0.5 * phase_voltage[1] - 0.5 * phase_voltage[2])
        v_beta = (1.0 / math.sqrt(3.0)) * (phase_voltage[1] - phase_voltage[2])

        if oid_state == 10 and enable >= 0.5:
            # Strong Id alignment behaves as an electrical-angle position
            # servo.  This averaged representation preserves the real current
            # loop/inverter path while avoiding unresolved shaft oscillation.
            if encoder_substate == 3:
                self.calibration_command_e = 0.0
            elif encoder_substate == 4:
                if self.last_encoder_substate != 4:
                    self.calibration_command_e = 0.0
                self.calibration_command_e += 2.0 * math.pi * dt
            elif encoder_substate == 5:
                self.calibration_command_e = 0.0
            command_angle = self.calibration_command_e % (2.0 * math.pi)
            angle_error = (command_angle - self.theta_e + math.pi) % (2.0 * math.pi) - math.pi
            feedforward = 2.0 * math.pi if encoder_substate == 4 else 0.0
            electrical_speed = max(-2.0 * math.pi, min(2.0 * math.pi,
                                   feedforward + 40.0 * angle_error))
            self.omega_m = electrical_speed / self.pole_pairs
        elif oid_state == 6:
            command_angle = math.atan2(v_beta, v_alpha)
            if self.last_oid_state != 6 or self.voltage_angle is None:
                self.theta_e = math.atan2(i_beta, i_alpha) if abs(i_alpha) + abs(i_beta) > 1e-6 else 0.0
                self.theta_m = self.theta_e / self.pole_pairs
                self.voltage_angle = command_angle
                self.omega_e_estimate = 0.0
            else:
                delta = command_angle - self.voltage_angle
                delta = (delta + math.pi) % (2.0 * math.pi) - math.pi
                raw_omega = delta / dt
                max_electrical_omega = 4500.0 * 2.0 * math.pi / 60.0 * self.pole_pairs
                raw_omega = max(-max_electrical_omega, min(max_electrical_omega, raw_omega))
                self.omega_e_estimate += 0.05 * (raw_omega - self.omega_e_estimate)
                self.voltage_angle = command_angle
            self.omega_m = self.omega_e_estimate / self.pole_pairs

        vd = v_alpha * cos_t + v_beta * sin_t
        vq = -v_alpha * sin_t + v_beta * cos_t

        omega_e = self.pole_pairs * self.omega_m
        if enable >= 0.5:
            did = (vd - self.rs * self.id + omega_e * self.lq * self.iq) / self.ld
            diq = (vq - self.rs * self.iq - omega_e * (self.ld * self.id + self.flux)) / self.lq
            self.id += dt * did
            self.iq += dt * diq
        else:
            # Disabled bridge is an open-circuit coast condition, not a
            # three-phase short circuit at zero terminal voltage.
            self.id *= math.exp(-dt * self.rs / self.ld)
            self.iq *= math.exp(-dt * self.rs / self.lq)

        if oid_state in (4, 5):
            # Rs/DT and Ld/Lq identification require a stationary rotor.  The
            # detailed plant obtains that condition through magnetic alignment;
            # imposing it directly avoids unresolved sub-microsecond mechanics
            # in this deliberately low-order daily-regression plant.
            self.omega_m = 0.0
        elif oid_state not in (6, 10):
            torque_iq = math.hypot(self.id, self.iq) if oid_state == 7 else self.iq
            torque = 0.0 if enable < 0.5 else 1.5 * self.pole_pairs * (
                self.flux * torque_iq + (self.ld - self.lq) * self.id * self.iq)
            directional_load = self.load_torque if self.omega_m > 1e-3 else 0.0
            self.omega_m += dt * (
                torque - self.friction * self.omega_m - directional_load
            ) / self.inertia
            # The detailed Simscape plant has physical losses not represented by
            # this two-state average model.  A generous bound prevents an invalid
            # numerical runaway while remaining above the configured 3000 rpm.
            max_omega = 4000.0 * 2.0 * math.pi / 60.0
            self.omega_m = max(-max_omega, min(max_omega, self.omega_m))
        self.theta_m += dt * self.omega_m
        self.theta_e = math.fmod(self.pole_pairs * self.theta_m, 2.0 * math.pi)
        if self.theta_e < 0.0:
            self.theta_e += 2.0 * math.pi
        self.last_oid_state = oid_state
        self.last_encoder_substate = encoder_substate
        return currents, legs

    def adc_code(self, voltage: float) -> int:
        code = round(voltage / self.adc_reference * self.adc_counts)
        return max(0, min(self.adc_counts - 1, code))

    def make_rx(self, time_s: float, currents: list[float], legs: list[float]) -> bytes:
        adc = [0] * 24
        adc[0] = self.adc_code(self.dc_bus * self.voltage_sensitivity)
        for index, voltage in enumerate(legs, start=1):
            adc[index] = self.adc_code(voltage * self.voltage_sensitivity)
        for index, current in enumerate(currents, start=4):
            adc[index] = self.adc_code(current * self.current_sensitivity + self.current_bias)

        mechanical_pu = (self.theta_m / (2.0 * math.pi)) % 1.0
        encoder_pu = (mechanical_pu + self.encoder_offset_pu) % 1.0
        if self.encoder_fault == "nonuniform":
            encoder_pu = (encoder_pu + 0.08 * math.sin(2.0 * math.pi * mechanical_pu + 0.3)) % 1.0
        elif self.last_oid_state == 10:
            if self.encoder_fault == "random":
                encoder_pu = 0.25 if (round(time_s / 50e-6) & 1) == 0 else 0.75
            elif self.encoder_fault == "stuck":
                encoder_pu = self.encoder_offset_pu
        digital = [0] * 8
        digital[0] = round(encoder_pu * self.encoder_counts) % self.encoder_counts
        panel = [0.0] * 16
        panel[15] = 1.0  # select fast-plant-only robust Ld/Lq aggregation
        return RX.pack(time_s, *adc, *panel, *digital)


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0, help="maximum simulated seconds")
    parser.add_argument("--step", type=float, default=1.0 / 20e3, help="plant/controller step in seconds")
    parser.add_argument("--build", action="store_true", help="build the x64 Debug SIL executable first")
    parser.add_argument(
        "--encoder-fault", choices=("none", "random", "stuck", "nonuniform"), default="none",
        help="inject an encoder fault and pass only if the expected diagnostic is raised",
    )
    parser.add_argument("--exe", type=Path, default=here / "x64" / "Debug" / "Motor_Control_Suite_SIL_Env.exe")
    parser.add_argument("--result", type=Path, default=here / "pmsm_id_sil_fast_result.json")
    parser.add_argument("--csv", type=Path, default=here / "pmsm_id_sil_fast_trace.csv")
    return parser.parse_args()


def run(options: argparse.Namespace) -> int:
    exe = options.exe.resolve()
    work_dir = Path(__file__).resolve().parent
    if options.build:
        program_files_x86 = os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")
        vswhere = Path(program_files_x86) / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
        if not vswhere.is_file():
            raise FileNotFoundError(f"Visual Studio vswhere was not found: {vswhere}")
        installation = subprocess.check_output([
            str(vswhere), "-latest", "-products", "*", "-requires",
            "Microsoft.Component.MSBuild", "-property", "installationPath",
        ], text=True).strip()
        msbuild = Path(installation) / "MSBuild" / "Current" / "Bin" / "MSBuild.exe"
        solution = work_dir / "GMP_Motor_Control_simulink.sln"
        subprocess.run([
            str(msbuild), str(solution), "/m", "/t:Build", "/p:Configuration=Debug",
            "/p:Platform=x64", "/nologo", "/v:minimal",
        ], cwd=work_dir, check=True)
    if not exe.is_file():
        raise FileNotFoundError(f"SIL executable not found: {exe}")
    if options.step <= 0.0 or options.duration <= 0.0:
        raise ValueError("duration and step must be positive")

    plant = Plant(encoder_fault=options.encoder_fault)
    rows: list[list[float]] = []
    last_progress: tuple[int, int, int, int] | None = None
    completed_at: float | None = None
    process: subprocess.Popen[bytes] | None = None
    client = SilUdpClient(work_dir / "network.json")
    major_step = 0
    terminal_notified = False
    try:
        process = subprocess.Popen([str(exe)], cwd=work_dir)
        client.connect()
        client.notify_state(SIMULATION_RUNNING, major_step)

        steps = math.ceil(options.duration / options.step)
        currents = [0.0, 0.0, 0.0]
        legs = [0.0, 0.0, 0.0]
        for index in range(steps):
            time_s = index * options.step
            packet = client.exchange(plant.make_rx(time_s, currents, legs))
            major_step = index + 1
            values = TX.unpack(packet)
            enable = values[0]
            pwm_all = values[1:9]
            monitor = values[17:33]
            state = round(monitor[0])
            currents, legs = plant.step(
                enable, pwm_all[:3], options.step, state, round(monitor[6])
            )
            progress = (state, round(monitor[6]), round(monitor[7]), round(monitor[8]))
            if progress != last_progress:
                print(
                    f"t={time_s:.6f} state={state} enc={round(monitor[6])}/"
                    f"fault={round(monitor[7])} mech={round(monitor[8])} "
                    f"theta_m_pu={(plant.theta_m / (2.0 * math.pi)) % 1.0:.6f} "
                    f"Rs={monitor[1]:.6g} Ld={monitor[2]:.6g} "
                    f"Lq={monitor[3]:.6g} flux={monitor[4]:.6g} Vdt={monitor[5]:.6g}"
                )
                last_progress = progress
            if index % max(1, round(1e-3 / options.step)) == 0:
                rows.append([time_s, enable, *monitor, plant.id, plant.iq, plant.omega_m])
            if state == 8:
                completed_at = time_s
                break
            if state == 9:
                break

        final = monitor
        client.notify_state(SIMULATION_COMPLETED, major_step)
        terminal_notified = True
    finally:
        if client.connected and not terminal_notified:
            try:
                client.notify_state(SIMULATION_ABORTED, major_step)
            except OSError:
                pass
        client.close()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()

    options.csv.parent.mkdir(parents=True, exist_ok=True)
    with options.csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "time_s", "enable", "oid_state", "Rs_ohm", "Ld_H", "Lq_H", "flux_Wb", "deadtime_comp_V",
            "encoder_substate", "encoder_fault", "mechanical_substate", "identified_pole_pairs",
            "encoder_offset_pu", "inertia_kg_m2", "viscous_friction_Nm_s", "load_torque_Nm",
            "speed_pu", "running", "plant_id_A", "plant_iq_A", "plant_omega_rad_s",
        ])
        writer.writerows(rows)

    truth = {
        "Rs_ohm": plant.rs, "Ld_H": plant.ld, "Lq_H": plant.lq, "flux_Wb": plant.flux,
        "inertia_kg_m2": plant.inertia, "viscous_friction_Nm_s": plant.friction,
        "load_torque_Nm": plant.load_torque,
    }
    expected_deadtime_comp = (8.0 / 3.0) * (
        plant.dc_bus + 2.0 * plant.diode_drop
    ) * plant.deadtime * plant.switching_frequency
    estimate = {
        "Rs_ohm": final[1], "Ld_H": final[2], "Lq_H": final[3], "flux_Wb": final[4],
        "deadtime_comp_V": final[5], "pole_pairs": round(final[9]),
        "encoder_offset_pu": final[10], "inertia_kg_m2": final[11],
        "viscous_friction_Nm_s": final[12], "load_torque_Nm": final[13],
    }
    error_pct = {
        key: 100.0 * (estimate[key] - value) / value for key, value in truth.items()
    }
    error_pct["deadtime_comp_V"] = 100.0 * (
        estimate["deadtime_comp_V"] - expected_deadtime_comp
    ) / expected_deadtime_comp
    limits_pct = {
        "Rs_ohm": 5.0,
        "Ld_H": 5.0,
        "Lq_H": 5.0,
        "flux_Wb": 5.0,
        "deadtime_comp_V": 5.0,
        "inertia_kg_m2": 10.0,
        "viscous_friction_Nm_s": 10.0,
        "load_torque_Nm": 12.0,
    }
    parameter_checks = {
        key: math.isfinite(error_pct[key]) and abs(error_pct[key]) <= limit
        for key, limit in limits_pct.items()
    }
    offset_error_pu = (
        estimate["encoder_offset_pu"] - plant.encoder_offset_pu + 0.5
    ) % 1.0 - 0.5
    encoder_checks = {
        "pole_pairs": estimate["pole_pairs"] == plant.pole_pairs,
        "offset": abs(offset_error_pu) <= (1.0 / plant.encoder_counts),
    }
    expected_encoder_fault = {"random": 2, "stuck": 3, "nonuniform": 4}.get(options.encoder_fault)
    if expected_encoder_fault is None:
        passed = (
            round(final[0]) == 8
            and all(parameter_checks.values())
            and all(encoder_checks.values())
        )
    else:
        passed = round(final[0]) == 9 and round(final[7]) == expected_encoder_fault
    result = {
        "passed": passed,
        "completed": round(final[0]) == 8,
        "final_state": round(final[0]),
        "completed_at_s": completed_at,
        "simulated_until_s": time_s,
        "plant": truth,
        "encoder_truth": {"pole_pairs": plant.pole_pairs, "offset_pu": plant.encoder_offset_pu},
        "encoder_fault_injection": options.encoder_fault,
        "encoder_fault_code": round(final[7]),
        "expected_deadtime_comp_V": expected_deadtime_comp,
        "estimate": estimate,
        "relative_error_pct": error_pct,
        "acceptance_limits_pct": limits_pct,
        "parameter_checks": parameter_checks,
        "encoder_offset_error_pu": offset_error_pu,
        "encoder_checks": encoder_checks,
        "deadtime_s": plant.deadtime,
        "step_s": options.step,
        "trace_csv": str(options.csv.resolve()),
    }
    options.result.parent.mkdir(parents=True, exist_ok=True)
    options.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    try:
        sys.exit(run(parse_args()))
    except Exception as exc:  # keep batch failures concise and actionable
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
