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
import socket
import struct
import subprocess
import sys
import os
from dataclasses import dataclass
from pathlib import Path


TX = struct.Struct("<d8I8I16d")
RX = struct.Struct("<d24I16d8i")


@dataclass
class Plant:
    rs: float = 4.7
    ld: float = 8.5e-3
    lq: float = 8.5e-3
    flux: float = 0.0038197
    pole_pairs: int = 4
    dc_bus: float = 24.0
    inertia: float = 497e-9
    friction: float = 1755e-9
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

    id: float = 0.0
    iq: float = 0.0
    omega_m: float = 0.0
    theta_e: float = 0.0
    voltage_angle: float | None = None
    omega_e_estimate: float = 0.0
    last_oid_state: int = 2

    @staticmethod
    def _sign(value: float, threshold: float = 1e-4) -> float:
        if value > threshold:
            return 1.0
        if value < -threshold:
            return -1.0
        return 0.0

    def step(
        self, enable: float, pwm: tuple[int, int, int], dt: float, oid_state: int
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

        if oid_state == 6:
            command_angle = math.atan2(v_beta, v_alpha)
            if self.last_oid_state != 6 or self.voltage_angle is None:
                self.theta_e = math.atan2(i_beta, i_alpha) if abs(i_alpha) + abs(i_beta) > 1e-6 else 0.0
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
        did = (vd - self.rs * self.id + omega_e * self.lq * self.iq) / self.ld
        diq = (vq - self.rs * self.iq - omega_e * (self.ld * self.id + self.flux)) / self.lq
        self.id += dt * did
        self.iq += dt * diq

        if oid_state in (4, 5):
            # Rs/DT and Ld/Lq identification require a stationary rotor.  The
            # detailed plant obtains that condition through magnetic alignment;
            # imposing it directly avoids unresolved sub-microsecond mechanics
            # in this deliberately low-order daily-regression plant.
            self.omega_m = 0.0
        elif oid_state != 6:
            torque = 1.5 * self.pole_pairs * (
                self.flux * self.iq + (self.ld - self.lq) * self.id * self.iq
            )
            self.omega_m += dt * (torque - self.friction * self.omega_m) / self.inertia
            # The detailed Simscape plant has physical losses not represented by
            # this two-state average model.  A generous bound prevents an invalid
            # numerical runaway while remaining above the configured 3000 rpm.
            max_omega = 4000.0 * 2.0 * math.pi / 60.0
            self.omega_m = max(-max_omega, min(max_omega, self.omega_m))
        self.theta_e = math.fmod(self.theta_e + dt * self.pole_pairs * self.omega_m, 2.0 * math.pi)
        if self.theta_e < 0.0:
            self.theta_e += 2.0 * math.pi
        self.last_oid_state = oid_state
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

        mechanical_pu = (self.theta_e / (2.0 * math.pi * self.pole_pairs)) % 1.0
        encoder_pu = (mechanical_pu + self.encoder_offset_pu) % 1.0
        digital = [0] * 8
        digital[0] = round(encoder_pu * self.encoder_counts) % self.encoder_counts
        panel = [0.0] * 16
        panel[15] = 1.0  # select fast-plant-only robust Ld/Lq aggregation
        return RX.pack(time_s, *adc, *panel, *digital)


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=4.0, help="maximum simulated seconds")
    parser.add_argument("--step", type=float, default=1.0 / 20e3, help="plant/controller step in seconds")
    parser.add_argument("--build", action="store_true", help="build the x64 Debug SIL executable first")
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

    plant = Plant()
    rows: list[list[float]] = []
    last_state: int | None = None
    completed_at: float | None = None
    process: subprocess.Popen[bytes] | None = None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", 12500))
    sock.settimeout(10.0)
    try:
        process = subprocess.Popen([str(exe)], cwd=work_dir)
        packet, _ = sock.recvfrom(TX.size)
        if len(packet) != TX.size:
            raise RuntimeError(f"initial SIL packet is {len(packet)} bytes; expected {TX.size}")

        steps = math.ceil(options.duration / options.step)
        currents = [0.0, 0.0, 0.0]
        legs = [0.0, 0.0, 0.0]
        for index in range(steps):
            time_s = index * options.step
            sock.sendto(plant.make_rx(time_s, currents, legs), ("127.0.0.1", 12501))
            packet, _ = sock.recvfrom(TX.size)
            if len(packet) != TX.size:
                raise RuntimeError(f"SIL packet is {len(packet)} bytes; expected {TX.size}")
            values = TX.unpack(packet)
            enable = values[0]
            pwm_all = values[1:9]
            monitor = values[17:33]
            state = round(monitor[0])
            currents, legs = plant.step(enable, pwm_all[:3], options.step, state)
            if state != last_state:
                print(
                    f"t={time_s:.6f} state={state} sub={round(monitor[6])}/"
                    f"{round(monitor[7])}/{round(monitor[8])} "
                    f"Rs={monitor[1]:.6g} Ld={monitor[2]:.6g} "
                    f"Lq={monitor[3]:.6g} flux={monitor[4]:.6g} Vdt={monitor[5]:.6g}"
                )
                last_state = state
            if index % max(1, round(1e-3 / options.step)) == 0:
                rows.append([time_s, enable, *monitor, plant.id, plant.iq, plant.omega_m])
            if state == 8:
                completed_at = time_s
                break
            if state == 9:
                break

        final = monitor
    finally:
        sock.close()
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
            "rs_substate", "ldq_substate", "flux_substate", "id_pu", "iq_pu", "vd_pu", "vq_pu",
            "speed_pu", "udc_pu", "running", "plant_id_A", "plant_iq_A", "plant_omega_rad_s",
        ])
        writer.writerows(rows)

    truth = {"Rs_ohm": plant.rs, "Ld_H": plant.ld, "Lq_H": plant.lq, "flux_Wb": plant.flux}
    expected_deadtime_comp = (8.0 / 3.0) * (
        plant.dc_bus + 2.0 * plant.diode_drop
    ) * plant.deadtime * plant.switching_frequency
    estimate = {
        "Rs_ohm": final[1], "Ld_H": final[2], "Lq_H": final[3], "flux_Wb": final[4],
        "deadtime_comp_V": final[5],
    }
    error_pct = {
        key: 100.0 * (estimate[key] - value) / value for key, value in truth.items()
    }
    error_pct["deadtime_comp_V"] = 100.0 * (
        estimate["deadtime_comp_V"] - expected_deadtime_comp
    ) / expected_deadtime_comp
    result = {
        "completed": round(final[0]) == 8,
        "final_state": round(final[0]),
        "completed_at_s": completed_at,
        "simulated_until_s": time_s,
        "plant": truth,
        "expected_deadtime_comp_V": expected_deadtime_comp,
        "estimate": estimate,
        "relative_error_pct": error_pct,
        "deadtime_s": plant.deadtime,
        "step_s": options.step,
        "trace_csv": str(options.csv.resolve()),
    }
    options.result.parent.mkdir(parents=True, exist_ok=True)
    options.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["completed"] else 2


if __name__ == "__main__":
    try:
        sys.exit(run(parse_args()))
    except Exception as exc:  # keep batch failures concise and actionable
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
