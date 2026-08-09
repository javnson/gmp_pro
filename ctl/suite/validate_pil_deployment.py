"""Validate the maintained GMP CTL suite PIL deployment contract."""

from __future__ import annotations

import json
import sys
from pathlib import Path


SUITE_ROOT = Path(__file__).resolve().parent
PIL_COMMON_ID = "gmp_suite_pil_common"

HARDWARE_TARGETS = {
    "dps_clllc": ("f280025c_dioscuri/src",),
    "dps_fsbb": ("f280039c_Iris_node/src",),
    "mcs_pmsm_id": (
        "f280039c_Iris_node",
        "f280049c",
        "stm32f405",
        "stm32g431",
        "stm32g474_hrtim",
    ),
    "mcs_pmsm_nt": (
        "f280039c_Iris_node",
        "f280049c",
        "stm32f405",
        "stm32g431",
        "stm32g474_hrtim",
    ),
    "pgs_inv_GFL_inverter": ("f280039c_Iris_node", "f280049c", "stm32g431"),
    "pgs_inv_GFM_inverter": ("f280039c_Iris_node", "f280049c", "stm32g431"),
    "pgs_sinv_rc": ("f280039c_Iris_node",),
}

LEGACY_SKIPPED = ("mcs_acm", "mcs_pmsm")
REQUIRED_TRANSPORT_MACROS = {
    "GMP_PIL_DL_BASE_COMMAND",
    "GMP_DL_UART_BAUDRATE",
    "GMP_PIL_UDP_HOST",
    "GMP_PIL_BRIDGE_UDP_LISTEN_PORT",
    "GMP_PIL_MATLAB_UDP_LISTEN_PORT",
    "GMP_PIL_MATLAB_COMMAND_TX_PORT",
    "GMP_PIL_MATLAB_COMMAND_RX_PORT",
    "GMP_PIL_MCU_TIMEOUT_MS",
    "GMP_PIL_MATLAB_TIMEOUT_MS",
    "GMP_PIL_UDP_DIGITAL_INDEX",
}


def load_json(path: Path) -> dict[str, object]:
    """Load one UTF-8 JSON document."""
    return json.loads(path.read_text(encoding="utf-8"))


def resolved_commons(requirement_path: Path, document: dict[str, object]) -> list[Path]:
    """Resolve every portable common-requirement reference."""
    references = document.get("common_requirements", [])
    if isinstance(references, str):
        references = [references]
    return [(requirement_path.parent / str(item)).resolve() for item in references]


def selected_modules(config_path: Path) -> set[str]:
    """Return the module identifiers selected by one source-manager project."""
    document = load_json(config_path)
    return {str(item["module"]) for item in document.get("selected_modules", [])}


def main() -> int:
    """Run structural checks and report every violation in one pass."""
    failures: list[str] = []
    pil_common = SUITE_ROOT / "sdpe_pil" / "sdpe_requirement.json"
    pil_document = load_json(pil_common)
    if pil_document.get("id") != PIL_COMMON_ID:
        failures.append(f"Unexpected PIL common contract ID in {pil_common}")
    common_macros = {str(item.get("macro")) for item in pil_document.get("requirements", [])}
    missing_common = REQUIRED_TRANSPORT_MACROS - common_macros
    if missing_common:
        failures.append(f"Shared PIL contract is missing macros: {sorted(missing_common)}")

    target_count = 0
    for suite, targets in HARDWARE_TARGETS.items():
        suite_root = SUITE_ROOT / suite
        user_source = (suite_root / "src" / "user_main.c").read_text(encoding="utf-8")
        control_source = (suite_root / "src" / "ctl_main.c").read_text(encoding="utf-8")
        for token in ("GMP_PIL_DL_BASE_COMMAND", "GMP_PIL_TX_MASK", "GMP_PIL_RX_MASK"):
            if token not in user_source:
                failures.append(f"{suite}/src/user_main.c does not use {token}")
        if "gmp_pil_sim_step" not in control_source:
            failures.append(f"{suite}/src/ctl_main.c has no PIL step callback")

        simulate_requirement = suite_root / "project" / "simulate" / "sdpe_mgr" / "sdpe_requirement.json"
        if simulate_requirement.is_file():
            simulate_document = load_json(simulate_requirement)
            if pil_common.resolve() in resolved_commons(simulate_requirement, simulate_document):
                failures.append(f"{suite}/project/simulate must remain independent of hardware PIL transport")

        for target in targets:
            target_count += 1
            target_root = suite_root / "project" / target
            requirement_path = target_root / "sdpe_mgr" / "sdpe_requirement.json"
            config_path = target_root / "gmp_src_mgr" / "gmp_framework_config.json"
            if not requirement_path.is_file():
                failures.append(f"Missing SDPE requirement: {requirement_path}")
                continue
            document = load_json(requirement_path)
            if pil_common.resolve() not in resolved_commons(requirement_path, document):
                failures.append(f"{suite}/{target} does not bind the shared PIL contract")
            if not config_path.is_file():
                failures.append(f"Missing source-manager config: {config_path}")
            elif "dev|datalink|pil" not in selected_modules(config_path):
                failures.append(f"{suite}/{target} does not select the Data Link PIL module")
            generated_pil_source = target_root / "gmp_src_mgr" / "gmp_src" / "gmp_pil_core.c"
            if not generated_pil_source.is_file():
                failures.append(f"{suite}/{target} has not generated gmp_pil_core.c")
            headers = list((target_root / "sdpe_mgr").glob("*.h"))
            generated = "\n".join(path.read_text(encoding="utf-8") for path in headers)
            for macro in ("GMP_PIL_DL_BASE_COMMAND", "GMP_PIL_RX_MASK", "GMP_PIL_TX_MASK"):
                if macro not in generated:
                    failures.append(f"{suite}/{target} generated header is missing {macro}")

    dispatcher = (SUITE_ROOT.parent / "framework" / "ctl_dispatch.h").read_text(encoding="utf-8")
    if "#if !defined ENABLE_GMP_DL_PIL_SIM" not in dispatcher:
        failures.append("The framework control ISR entry is not isolated in PIL mode")

    guide = (SUITE_ROOT / "readme.md").read_text(encoding="utf-8")
    for suite in LEGACY_SKIPPED:
        if suite not in guide or "skipped by fleet PIL deployment" not in guide:
            failures.append(f"Legacy suite {suite} is not explicitly marked as skipped")

    if target_count != 19:
        failures.append(f"Expected 19 maintained hardware targets, found {target_count}")
    if failures:
        for failure in failures:
            print(f"[FAIL] {failure}")
        return 1
    print(f"[PASS] Validated {target_count} maintained hardware targets.")
    print(f"[PASS] Legacy suites skipped: {', '.join(LEGACY_SKIPPED)}.")
    print("[PASS] Simulation projects remain independent of the hardware PIL transport contract.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
