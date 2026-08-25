from __future__ import annotations

import hashlib
import json
from pathlib import Path


def write_topology_bundle(root: Path) -> Path:
    header = root / "buckcircuit.hpp"
    archive = root / "buckcircuit.archive"
    header.write_text("#pragma once\nclass BuckCircuit {};\n", encoding="utf-8")
    archive.write_bytes(b"GMPMNA1\0test archive")

    def artifact(path: Path) -> dict:
        content = path.read_bytes()
        return {
            "path": path.name,
            "size_bytes": len(content),
            "sha256": hashlib.sha256(content).hexdigest(),
        }

    manifest = {
        "schema": {"name": "gmp.cctl.compiled_topology", "version": 1},
        "topology": {
            "name": "Buck Power Stage",
            "class_name": "BuckCircuit",
            "source": {"file": "buck.CIR", "sha256": "0" * 64},
        },
        "artifacts": {
            "header": artifact(header),
            "archive": artifact(archive),
        },
        "cpp": {
            "standard": 17,
            "include": header.name,
            "class_name": "BuckCircuit",
            "constructor": {
                "archive_path_argument": True,
                "default_archive": archive.name,
            },
            "methods": {
                "normal_step": "step_normal",
                "short_step": "step_short",
                "reset": "reset",
            },
            "dependencies": ["Eigen3"],
        },
        "interface": {
            "inputs": [
                {
                    "name": "PWM",
                    "field": "PWM",
                    "data_type": "uint32_t",
                    "role": "mosfet_gate_command",
                    "default": 0,
                },
                {
                    "name": "VS1",
                    "field": "VS1",
                    "data_type": "double",
                    "role": "analog_input",
                    "default": 5.0,
                },
            ],
            "outputs": [
                {
                    "name": "V(VF1)",
                    "field": "VF1",
                    "data_type": "double",
                    "role": "signal_output",
                    "signal_index": 0,
                }
            ],
        },
        "solver": {
            "method": "backward_euler",
            "normal_step_s": 1e-7,
            "short_step_s": 1e-9,
            "matrix_backend": "eigen",
            "matrix_tolerance": 1e-12,
        },
        "dimensions": {
            "states": 3,
            "signals": 1,
            "analog_inputs": 1,
            "command_inputs": 1,
            "outputs": 1,
            "selection_topologies": 6,
            "stored_topologies": 6,
            "calculation_states": 6,
        },
    }
    path = root / "buckcircuit.cctl-topology.json"
    path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return path
