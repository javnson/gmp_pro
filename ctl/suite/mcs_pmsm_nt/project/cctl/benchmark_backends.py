"""Run an alternating whole-system benchmark of the fixed and Eigen solvers."""

from __future__ import annotations

import argparse
import re
import statistics
import subprocess
from pathlib import Path


_WALL_RE = re.compile(r"simulated/wall: .*? / ([0-9.eE+-]+) s")
_SPEED_RE = re.compile(r"mean/final speed=([0-9.eE+-]+) / ([0-9.eE+-]+) rpm")
_CURRENT_RE = re.compile(r"maximum phase current=([0-9.eE+-]+) A")


def run_solver(
    executable: Path, priority: str
) -> tuple[float, tuple[float, float, float]]:
    priority_argument = (
        "--realtime-priority" if priority == "realtime" else "--normal-priority"
    )
    completed = subprocess.run(
        [str(executable), "--no-pause", priority_argument, "--output", "NUL"],
        cwd=executable.parent,
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        print(completed.stdout)
        print(completed.stderr)
        raise RuntimeError(f"{executable.name} exited with {completed.returncode}")

    wall = _WALL_RE.search(completed.stdout)
    speed = _SPEED_RE.search(completed.stdout)
    current = _CURRENT_RE.search(completed.stdout)
    if wall is None or speed is None or current is None:
        raise RuntimeError(f"could not parse the summary from {executable.name}")
    signature = (
        float(speed.group(1)),
        float(speed.group(2)),
        float(current.group(1)),
    )
    return float(wall.group(1)), signature


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", required=True, type=Path)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--priority", choices=("normal", "realtime"), default="realtime")
    args = parser.parse_args()
    if args.runs < 1:
        parser.error("--runs must be at least one")

    executables = {
        "fixed": args.build_dir / "mcs_pmsm_nt_cctl.exe",
        "eigen": args.build_dir / "mcs_pmsm_nt_cctl_eigen.exe",
    }
    for executable in executables.values():
        if not executable.is_file():
            raise FileNotFoundError(executable)

    timings: dict[str, list[float]] = {name: [] for name in executables}
    signatures: dict[str, tuple[float, float, float]] = {}
    print("GMP CCTL main-circuit backend benchmark")
    print(
        f"40,000,000 plant steps per run; priority={args.priority}; "
        "CSV formatting retained, output sent to NUL.\n"
    )
    for round_index in range(args.runs):
        order = ("fixed", "eigen") if round_index % 2 == 0 else ("eigen", "fixed")
        for name in order:
            wall, signature = run_solver(executables[name], args.priority)
            timings[name].append(wall)
            signatures[name] = signature
            print(f"round {round_index + 1}/{args.runs}  {name:5s}  wall={wall:.6f}s")

    fixed_mean = statistics.fmean(timings["fixed"])
    eigen_mean = statistics.fmean(timings["eigen"])
    maximum_drift = max(
        abs(left - right)
        for left, right in zip(signatures["fixed"], signatures["eigen"])
    )
    print("\nsummary")
    for name in ("fixed", "eigen"):
        values = timings[name]
        print(
            f"  {name:5s}: mean={statistics.fmean(values):.6f}s  "
            f"median={statistics.median(values):.6f}s  min={min(values):.6f}s"
        )
    print(f"  fixed/eigen speed ratio: {fixed_mean / eigen_mean:.4f}x")
    print(f"  maximum final-signature drift: {maximum_drift:.3e}")
    if maximum_drift > 1.0e-6:
        print("[FAIL] Backend numerical drift exceeded 1e-6.")
        return 1
    print("[PASS] Both backends passed the same physical checks.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
