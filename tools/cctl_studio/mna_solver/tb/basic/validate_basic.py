"""Numerical acceptance checks for every non-switching basic netlist."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np


def _solver_directory() -> Path:
    root = os.environ.get("GMP_PRO_LOCATION")
    if not root:
        raise RuntimeError("GMP_PRO_LOCATION is not defined")
    directory = Path(root) / "tools" / "cctl_studio" / "mna_solver"
    if not (directory / "mna_solver.py").is_file():
        raise RuntimeError(f"MNA solver was not found under GMP_PRO_LOCATION: {directory}")
    return directory


SOLVER_DIR = _solver_directory()
sys.path.insert(0, str(SOLVER_DIR))

import mna_solver as mna  # noqa: E402


CASE_DIR = Path(__file__).resolve().parent
PARAMETERS = {
    "example5.cir": {"R1": 1000.0, "R2": 1000.0},
    "exampleE.cir": {"Ea": 2.0},
    "exampleF.cir": {"Fa": 3.0},
    "exampleG.cir": {"Ga": 0.001},
    "exampleH.cir": {"Ha": 1000.0},
}


def _build_and_simulate(path: Path) -> mna.StateSpaceModel:
    circuit = mna.parse_netlist(path)
    symbolic = mna.assemble_symbolic_mna(circuit)
    if symbolic.E.nrows() != symbolic.A.nrows() or symbolic.A.nrows() != symbolic.A.ncols():
        raise AssertionError(f"{path.name}: symbolic descriptor dimensions are inconsistent")

    parameters = PARAMETERS.get(path.name, {})
    descriptor = mna.assemble_mna(circuit, parameters)
    state = mna.reduce_to_state_space(
        descriptor,
        mna.assemble_outputs(circuit, descriptor, parameters),
    )
    discrete = mna.discretize(state, 1e-6, "backward_euler")
    inputs = {name: 1.0 for name in state.input_names}
    result = mna.simulate(discrete, 10e-6, inputs)
    if not np.all(np.isfinite(result.states)) or not np.all(np.isfinite(result.outputs)):
        raise AssertionError(f"{path.name}: simulation produced a non-finite value")
    return state


def main() -> int:
    paths = sorted(
        {path.resolve() for pattern in ("*.cir", "*.CIR") for path in CASE_DIR.glob(pattern)},
        key=lambda path: path.name.lower(),
    )
    if len(paths) < 10:
        raise AssertionError(f"expected at least 10 basic netlists, found {len(paths)}")
    models = {}
    for path in paths:
        models[path.name] = _build_and_simulate(path)
        print(f"[OK] {path.name}")

    divider = models["0_divider.CIR"]
    np.testing.assert_allclose(divider.D[:, 0], [0.5, 0.0005], rtol=1e-12, atol=1e-12)
    divider_dc = divider.D[:, 0] * 5.0
    np.testing.assert_allclose(divider_dc, [2.5, 0.0025], rtol=1e-12, atol=1e-12)

    opamp = models["1_OPAMP.CIR"]
    np.testing.assert_allclose(opamp.A, [[-1000.0]], rtol=1e-10, atol=1e-10)
    dc_gain = mna.frequency_response(opamp, [0.0]).response[0, 0, 0]
    np.testing.assert_allclose(dc_gain, -1.0 + 0.0j, rtol=1e-12, atol=1e-12)

    print("[OK] divider DC output: 2.5 V, 2.5 mA")
    print("[OK] op-amp low-pass: DC gain -1, pole 1000 rad/s")
    print(f"All {len(paths)} basic netlists passed symbolic, numeric, and simulation checks.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
